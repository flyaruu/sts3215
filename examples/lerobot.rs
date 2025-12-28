use sts3215::lerobot::robot::Robot;

pub fn main() {
    println!("Hello, world!");

    let mut leader = Robot::new("/dev/cu.wchusbserial5AAF2185891").unwrap();
    let mut follower = Robot::new("/dev/cu.wchusbserial5AAF2182201").unwrap();

    leader.move_to_position(1,3000, None, None).unwrap();
}