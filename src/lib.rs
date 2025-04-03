use std::collections::HashMap;
use std::net::{UdpSocket, SocketAddr};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use tracing::info;
use uuid::Uuid;
#[derive(Debug,Clone)]
pub struct CoCube {
    #[allow(dead_code)]
    robot_id: u32,
    #[allow(dead_code)]
    robot_addr: SocketAddr,
    sock_send: Arc<UdpSocket>,
    state: Arc<Mutex<RobotState>>,
}

#[derive(Debug)]
struct RobotState {
    pos_x: i32,
    pos_y: i32,
    pos_m: [i32; 2],
    yaw: i32,
    pos_direction: i32,
    uuid_func: HashMap<String, String>,
    uuid_result: HashMap<String, String>,
}

impl CoCube {
    pub fn new(robot_id: u32, gateway: &str, local_ip: &str, ip_prefix: u32, udp_port: u16) -> std::io::Result<Self> {
        let robot_ip = format!("{}.{}", gateway.split('.').take(3).collect::<Vec<&str>>().join("."), ip_prefix + robot_id);
        let robot_port = udp_port + robot_id as u16;
        
        let sock_listen = Arc::new(UdpSocket::bind((local_ip, robot_port))?);
        sock_listen.set_read_timeout(Some(Duration::from_secs(3)))?;
        
        let sock_send = Arc::new(UdpSocket::bind((local_ip, 0))?);
        sock_send.connect((robot_ip.as_str(), udp_port))?;

        let state = Arc::new(Mutex::new(RobotState {
            pos_x: 0,
            pos_y: 0,
            pos_m: [0, 0],
            yaw: 0,
            pos_direction: 0,
            uuid_func: HashMap::new(),
            uuid_result: HashMap::new(),
        }));

        let state_clone = Arc::clone(&state);
        let sock_listen_clone = Arc::clone(&sock_listen);
        
        thread::spawn(move || {
            let mut buf = [0u8; 1024];
            loop {
                match sock_listen_clone.recv_from(&mut buf) {
                    Ok((size, _)) => {
                        if let Ok(data) = String::from_utf8(buf[..size].to_vec()) {
                            let parts: Vec<&str> = data.split(',').collect();
                            let mut state = state_clone.lock().unwrap();
                            
                            match parts.first().map(|s| s.trim()) {
                                Some("res") if parts.len() >= 3 => {
                                    let uuid = parts[1].trim().to_string();
                                    let result = parts[2].trim().to_string();
                                    state.uuid_result.insert(uuid.clone(), result);
                                    state.uuid_func.remove(&uuid);
                                },
                                Some("pos") if parts.len() >= 4 => {
                                    if let (Ok(x), Ok(y), Ok(dir)) = (
                                        parts[1].trim().parse::<i32>(),
                                        parts[2].trim().parse::<i32>(),
                                        parts[3].trim().parse::<i32>(),
                                    ) {
                                        state.pos_x = (x as f64 / 64.0) as i32;
                                        state.pos_y = (y as f64 / 64.0) as i32;
                                        state.pos_m = [(state.pos_y as f64 * 1.35 * 0.001 )as i32, (state.pos_x as f64 * 1.35 * 0.001) as i32];
                                        state.pos_direction = dir;
                                        // state.yaw = if dir < 180 {
                                        //     dir as f64 * PI / 180.0
                                        // } else {
                                        //     (dir - 360) as f64 * PI / 180.0
                                        // };
                                    }
                                },
                                _ => (),
                            }
                        }
                    },
                    Err(e) => eprintln!("Error receiving data: {}", e),
                }
            }
        });

        Ok(Self {
            robot_id,
            robot_addr: format!("{}:{}", robot_ip, udp_port).parse().unwrap(),
            sock_send: Arc::clone(&sock_send),
            state,
        })
    }

    pub fn get_position(&self) -> [i32; 3] {
        let state = self.state.lock().unwrap();
        [state.pos_x, state.pos_y, state.pos_direction as i32]
    }

    fn process_data(&self, func: &str, params: &[&str], if_return_uuid: bool, timeout: Duration) -> Option<String> {
        let uuid = Uuid::new_v4().to_string().chars().take(6).collect::<String>();
        let if_return = if if_return_uuid { "1" } else { "0" };
        
        let params_str = params.iter()
            .map(|p| if p.parse::<i32>().is_ok() { p.to_string() } else { format!("\"{}\"", p) })
            .collect::<Vec<String>>()
            .join(",");

        let message = if params.is_empty() {
            format!("{},{},{}", if_return, uuid, func)
        } else {
            format!("{},{},{},{}", if_return, uuid, func, params_str)
        };
        info!("message: {}", message);
        if if_return_uuid {
            let mut state = self.state.lock().unwrap();
            state.uuid_func.insert(uuid.clone(), func.to_string());
        }

        if let Err(e) = self.sock_send.send(message.as_bytes()) {
            eprintln!("Failed to send data: {}", e);
            return None;
        }

        if if_return_uuid {
            let start = Instant::now();
            loop {
                if start.elapsed() > timeout {
                    return None;
                }
                
                let mut state = self.state.lock().unwrap();
                if let Some(result) = state.uuid_result.remove(&uuid) {
                    return Some(result);
                }
                
                thread::sleep(Duration::from_millis(100));
            }
        }
        
        None
    }

    // Motion control methods
    pub fn set_tft_backlight(&self, flag: u8) {
        self.process_data("CoCube set TFT backlight", &[&flag.to_string()], false, Duration::from_secs(3));
    }

    pub fn draw_aruco_marker_on_tft(&self, id: u8) {
        self.process_data("CoCube draw Aruco Marker on TFT", &[&id.to_string()], false, Duration::from_secs(3));
    }

    pub fn move_robot(&self, direction: &str, speed: u8) {
        self.process_data("CoCube move", &[direction, &speed.to_string()], false, Duration::from_secs(3));
    }

    pub fn rotate_robot(&self, direction: &str, speed: u8) {
        self.process_data("CoCube rotate", &[direction, &speed.to_string()], false, Duration::from_secs(3));
    }

    pub fn move_millisecs(&self, direction: &str, speed: u8, duration: u64) -> Option<String> {
        self.process_data(
            "CoCube move for millisecs",
            &[direction, &speed.to_string(), &duration.to_string()],
            true,
            Duration::from_secs(3) + Duration::from_millis(duration)
        )
    }

    pub fn rotate_millisecs(&self, direction: &str, speed: u8, duration: u64) -> Option<String> {
        self.process_data(
            "CoCube rotate for millisecs",
            &[direction, &speed.to_string(), &duration.to_string()],
            true,
            Duration::from_secs(3) + Duration::from_millis(duration)
        )
    }

    pub fn move_by_steps(&self, direction: &str, speed: u8, step: u16) -> Option<String> {
        self.process_data(
            "CoCube move by step",
            &[direction, &speed.to_string(), &step.to_string()],
            true,
            Duration::from_secs(3)
        )
    }

    pub fn rotate_by_degree(&self, direction: &str, speed: u8, degree: u16) -> Option<String> {
        self.process_data(
            "CoCube rotate by degree",
            &[direction, &speed.to_string(), &degree.to_string()],
            true,
            Duration::from_secs(3)
        )
    }

    pub fn set_wheel_speed(&self, left: u8, right: u8) {
        self.process_data(
            "CoCube set wheel",
            &[&left.to_string(), &right.to_string()],
            false,
            Duration::from_secs(3)
        );
    }

    pub fn wheels_stop(&self) {
        self.process_data("CoCube wheels stop", &[], false, Duration::from_secs(3));
    }

    pub fn wheels_break(&self) {
        self.process_data("CoCube wheels break", &[], false, Duration::from_secs(3));
    }

    pub fn rotate_to_angle(&self, angle: i16, speed: u8) {
        self.process_data(
            "CoCube rotate to angle",
            &[&angle.to_string(), &speed.to_string()],
            false,
            Duration::from_secs(3)
        );
    }

    pub fn rotate_to_target(&self, x: i32, y: i32, speed: u8) {
        self.process_data(
            "CoCube rotate to target",
            &[&x.to_string(), &y.to_string(), &speed.to_string()],
            false,
            Duration::from_secs(3)
        );
    }

    pub fn move_to(&self, x: i32, y: i32, speed: u8) {
        self.process_data(
            "CoCube move to",
            &[&x.to_string(), &y.to_string(), &speed.to_string()],
            false,
            Duration::from_secs(3)
        );
    }


    pub fn move_to_target(&self, x: i32, y: i32, speed: u8) {
        self.process_data(
            "myreturn_uuid move to target",
            &[&x.to_string(), &y.to_string(), &speed.to_string()],
            false,
            Duration::from_secs(3)
        );
    }

    // Module control methods
    pub fn power_on_module(&self) {
        self.process_data("Power on module", &[], false, Duration::from_secs(3));
    }

    pub fn power_off_module(&self) {
        self.process_data("Power off module", &[], false, Duration::from_secs(3));
    }

    pub fn gripper_open(&self) {
        self.process_data("ccmodule_gripper open", &[], false, Duration::from_secs(3));
    }

    pub fn gripper_close(&self) {
        self.process_data("ccmodule_gripper close", &[], false, Duration::from_secs(3));
    }

    pub fn gripper_degree(&self, degree: i8) {
        self.process_data("Gripper degree", &[&degree.to_string()], false, Duration::from_secs(3));
    }

    // NeoPixel control
    pub fn attach_neo_pixel(&self) {
        self.process_data("attach NeoPixel", &[], false, Duration::from_secs(3));
    }

    pub fn set_all_neo_pixels_color(&self, r: u8, g: u8, b: u8) {
        let color = (r as u32) << 16 | (g as u32) << 8 | b as u32;
        self.process_data("set all NeoPixels color", &[&color.to_string()], false, Duration::from_secs(3));
    }

    pub fn clear_neo_pixels(&self) {
        self.process_data("clear NeoPixels", &[], false, Duration::from_secs(3));
    }

    // Display control
    pub fn mb_display(&self, code: u32) -> Option<String> {
        self.process_data("[display:mbDisplay]", &[&code.to_string()], true, Duration::from_secs(3))
    }

    pub fn mb_display_off(&self) -> Option<String> {
        self.process_data("[display:mbDisplayOff]", &[], true, Duration::from_secs(3))
    }

    pub fn set_display_color(&self, r: u8, g: u8, b: u8) {
        let color = (r as u32) << 16 | (g as u32) << 8 | b as u32;
        self.process_data("set display color", &[&color.to_string()], false, Duration::from_secs(3));
    }
}