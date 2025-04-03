use cocube_rs::CoCube;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 初始化CoCube实例
    let agent = CoCube::new(
        1, 
        "192.168.31.1", 
        "192.168.31.31", 
        1,    
        6000
    )?;

    // 获取并打印位置信息
    let pos = agent.get_position();
    println!("Current position: x={:.2}, y={:.2}, yaw={:.2}rad", pos[0], pos[1], pos[2]);

    // 基础运动控制示例
    println!("Moving forward...");
    agent.move_robot("forward", 40);
    thread::sleep(Duration::from_secs(2));
    agent.wheels_stop();

    println!("Rotating left...");
    agent.rotate_robot("left", 30);
    thread::sleep(Duration::from_secs(2));
    agent.wheels_stop();

    // 目标点导航示例
    let target_x = 200.0;
    let target_y = 200.0;
    println!("Moving to target ({}, {})", target_x, target_y);
   // agent.move_to_target(target_x, target_y, 50);

    // 等待到达目标点
    // loop {
    //     let curr_pos = agent.get_position();
    //     let dx = curr_pos[0] - target_x;
    //     let dy = curr_pos[1] - target_y;
    //     let distance = (dx.powi(2) + dy.powi(2)).sqrt();
        
    //     println!("Current position: ({:.1}, {:.1}), Distance: {:.1} points", 
    //             curr_pos[0], curr_pos[1], distance);
        
    //     if distance < 10.0 {
    //         println!("Reached target!");
    //         break;
    //     }
    //     thread::sleep(Duration::from_millis(100));
    // }

    // 显示控制示例
    agent.set_display_color(0, 255, 0); // 设置绿色
    if let Some(_) = agent.mb_display(33554431) {
        // 成功处理显示命令
    }

    Ok(())
}