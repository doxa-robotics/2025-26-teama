use std::time::Duration;

use libdoxa::subsystems::drivetrain::{DrivetrainPair, actions::VoltageAction};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::subsystems::drivetrain_actions::{
    CONFIG, TileToMMExt, boomerang_to_point, drive_to_point, forward, turn_to_point,
};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Other,
    "Skills",
    "Same setup as left long first",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: skills");
    robot
        .tracking
        .set_current(Point2::new(-385.0, -1417.0), Angle::HALF_TURN);
    // Head to the match loader first
    robot.drivetrain.action(forward(100.0, CONFIG)).await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.8.tiles(), -2.0.tiles()),
            CONFIG,
        ))
        .await;
    robot.intake.intake();
    robot.match_loader.extend();
    robot
        .drivetrain
        .action(
            libdoxa::subsystems::drivetrain::actions::RotationAction::new(
                (-Angle::QUARTER_TURN).as_radians(),
                CONFIG,
            ),
        )
        .await;
    // Hold the position while loading
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::new_voltage(10.0, 10.0),
    }); // Intentionally not awaited
    robot.intake.intake();
    sleep(Duration::from_millis(1200)).await;

    // Outtake into the long goal
    robot.intake.brake();
    let mut match_loader = robot.match_loader.clone();
    let intake = robot.intake.clone();
    robot
        .drivetrain
        .action(
            drive_to_point(
                Point2::new(-2.0.tiles(), -1.35.tiles()),
                CONFIG.with_linear_error_tolerance(100.0),
            )
            .reversed(),
        )
        .with_once_callback(
            |tracking| tracking.offset.y > -1.6.tiles(),
            move || {
                match_loader.retract();
                let mut intake = intake.clone();
                vexide::task::spawn(async move {
                    intake.outtake_long_anti_jam().await;
                })
                .detach();
            },
        )
        .await;
    sleep(Duration::from_millis(2700)).await;
    // TODO: reset tracking context using the aligner
    robot.drivetrain.action(forward(150.0, CONFIG)).await;
    robot.intake.intake();

    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.0.tiles(), -1.0.tiles()),
            CONFIG,
        ))
        .await;
    let mut match_loader = robot.match_loader.clone();
    robot
        .drivetrain
        .action(turn_to_point(
            Point2::new(-2.0.tiles(), 3.0.tiles()),
            CONFIG,
        ))
        .with_once_callback(
            |tracking| tracking.offset.x > -1.2.tiles(),
            move || {
                match_loader.extend();
            },
        )
        .await;
    robot
        .drivetrain
        .action(boomerang_to_point(
            Point2::new(-2.0.tiles(), 3.0.tiles()),
            Angle::QUARTER_TURN,
            CONFIG,
        ))
        .await;
    // Hold the position while loading
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::new_voltage(10.0, 10.0),
    }); // Intentionally not awaited
}
