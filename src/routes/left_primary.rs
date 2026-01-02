use std::time::Duration;

use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::{
    Robot,
    subsystems::drivetrain_actions::{CONFIG, drive_to_point, turn_to_point},
};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Left,
    "Left primary",
    "Center goal > loader > long",
    route
);

async fn route(robot: &mut Robot) -> () {
    robot
        .tracking
        .set_current(Point2::new(-400.0, -1250.0), Angle::from_radians(1.84));
    robot.intake.intake();
    robot
        .drivetrain
        .action(drive_to_point(Point2::new(-1.0, -1.0), CONFIG))
        .await;
    robot
        .drivetrain
        .action(drive_to_point(Point2::new(-0.5, -0.5), CONFIG).reversed())
        .await;
    robot.intake.outtake_top_middle();
    sleep(Duration::from_millis(500)).await;
    return;
    robot.intake.intake();
    sleep(Duration::from_millis(3000)).await;
    robot.intake.brake();

    robot.intake.intake();
    robot
        .drivetrain
        .action(drive_to_point(Point2::new(-2.0, -2.0), CONFIG))
        .await;
    robot
        .drivetrain
        .action(turn_to_point(Point2::new(-2.0, -2.8), CONFIG))
        .await;

    robot
        .drivetrain
        .action(drive_to_point(Point2::new(-2.0, -2.8), CONFIG))
        .await;
    robot.intake.intake();
    sleep(Duration::from_millis(3000)).await;
    robot.intake.brake();

    // robot.drivetrain.action(reverse(-2.0, -1.0)).await;
    robot.intake.intake();
    sleep(Duration::from_millis(3000)).await;
    robot.intake.brake();
}
