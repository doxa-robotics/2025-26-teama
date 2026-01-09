use std::time::Duration;

use libdoxa::subsystems::drivetrain::{DrivetrainPair, actions::VoltageAction};
use nalgebra::Point2;
use vexide::{math::Angle, prelude::Motor, time::sleep};

use crate::subsystems::drivetrain_actions::{
    CONFIG, TileToMMExt, drive_to_point, forward, rotation,
};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Other,
    "Skills",
    "Same setup as left primary",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: skills");
    super::left_primary::left_center_match_load(robot, true).await;
    robot.drivetrain.action(forward(150.0, CONFIG)).await;
    // robot
    //     .drivetrain
    //     .action(drive_to_point(Point2::new(-60.0, -1.5.tiles()), CONFIG))
    //     .await;
    // robot
    //     .drivetrain
    //     .action(
    //         drive_to_point(
    //             Point2::new(-60.0, -4.0.tiles()),
    //             CONFIG.with_linear_limit(Motor::V5_MAX_VOLTAGE),
    //         )
    //         .reversed(),
    //     )
    //     .await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(2.0.tiles(), -2.0.tiles()),
            CONFIG,
        ))
        .await;
    robot.match_loader.extend();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(1.8.tiles(), -2.5.tiles()),
            CONFIG,
        ))
        .await;
    robot
        .drivetrain
        .action(rotation(-Angle::QUARTER_TURN.as_radians(), CONFIG))
        .await;
    // Hold the position while loading
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::new_voltage(10.0, 10.0),
    }); // Intentionally not awaited
    robot.intake.intake();
    sleep(Duration::from_secs(3)).await;
    // Outtake into the long goal
    robot.intake.brake();
    let mut match_loader = robot.match_loader.clone();
    let intake = robot.intake.clone();
    let mut triggered = false;
    robot
        .drivetrain
        .action(
            drive_to_point(
                Point2::new(2.0.tiles(), -1.35.tiles()),
                CONFIG.with_linear_error_tolerance(100.0),
            )
            .reversed(),
        )
        .with_callback(move |tracking| {
            if tracking.offset.y > -1.6.tiles() && !triggered {
                triggered = true;
                match_loader.retract();
                let mut intake = intake.clone();
                vexide::task::spawn(async move {
                    intake.outtake_long_anti_jam().await;
                })
                .detach();
            }
        })
        .await;
    sleep(Duration::from_secs(8)).await;
    robot.drivetrain.action(forward(150.0, CONFIG)).await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(0.0.tiles(), -1.5.tiles()),
            CONFIG,
        ))
        .await;
    // Park
    robot
        .drivetrain
        .action(
            drive_to_point(
                Point2::new(0.0.tiles(), -4.0.tiles()),
                CONFIG.with_linear_limit(Motor::V5_MAX_VOLTAGE),
            )
            .reversed(),
        )
        .await;
}
