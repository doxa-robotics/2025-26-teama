use std::time::Duration;

use libdoxa::subsystems::drivetrain::{DrivetrainPair, actions::VoltageAction};
use nalgebra::Point2;
use vexide::{math::Angle, prelude::Motor, time::sleep};

use crate::subsystems::drivetrain_actions::{
    CONFIG, TileToMMExt, drive_to_point, forward, rotation,
};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Other,
    "Skills safe",
    "Same setup as left primary. Safe",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: skills safe");
    super::left_primary::left_center_match_load(robot, true).await;
    robot.drivetrain.action(forward(150.0, CONFIG)).await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-0.1.tiles(), -1.5.tiles()),
            CONFIG,
        ))
        .await;
    // Park
    robot
        .drivetrain
        .action(
            drive_to_point(
                Point2::new(-0.1.tiles(), -4.0.tiles()),
                CONFIG.with_linear_limit(Motor::V5_MAX_VOLTAGE),
            )
            .reversed(),
        )
        .await;
}
