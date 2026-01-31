use std::time::Duration;

use libdoxa::subsystems::drivetrain::{DrivetrainPair, actions::VoltageAction};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::subsystems::drivetrain_actions::{
    CONFIG, TileToMMExt as _, boomerang_to_point, drive_to_point, turn_to_point,
};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Left,
    "Left elims",
    "Angle setup - Center balls > loader > long - high scoring",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: left elims");
    robot
        .tracking
        .set_current(Point2::new(-400.0, -1250.0), Angle::from_radians(1.84));
    // Drive to the left set of balls while intaking
    robot.intake.intake();
    let mut match_loader = robot.match_loader.clone();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.2.tiles(), -0.75.tiles()),
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .with_once_callback(
            |tracking| tracking.offset.y > -1.5.tiles(),
            move || {
                match_loader.extend();
            },
        )
        .await;
    // Go to the match loader
    robot.intake.intake();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-2.0.tiles(), -2.0.tiles()),
            CONFIG,
        ))
        .await;
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
    sleep(Duration::from_millis(1800)).await;
    // Outtake into the long goal
    let intake = robot.intake.clone();
    robot
        .drivetrain
        .action(
            drive_to_point(
                Point2::new(-2.05.tiles(), -1.35.tiles()),
                CONFIG.with_linear_error_tolerance(100.0),
            )
            .reversed(),
        )
        .with_once_callback(
            |tracking| tracking.offset.y > -1.6.tiles(),
            move || {
                let mut intake = intake.clone();
                vexide::task::spawn(async move {
                    intake.outtake_long_anti_jam().await;
                })
                .detach();
            },
        )
        .await;
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::new_voltage(-12.0, -12.0),
    }); // Intentionally not awaited
    sleep(Duration::from_millis(2000)).await;
    // Hold the position while loading
    robot.intake.brake();
    sleep(Duration::from_millis(500)).await;
    robot.intake.outtake_long_anti_jam().await;
    sleep(Duration::from_millis(8000)).await;
}
