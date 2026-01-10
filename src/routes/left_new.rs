use std::time::Duration;

use libdoxa::subsystems::drivetrain::{DrivetrainPair, actions::VoltageAction};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::subsystems::drivetrain_actions::{
    CONFIG, TileToMMExt as _, boomerang_to_point, drive_to_point, turn_to_point,
};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Left,
    "Left new",
    "Center balls > loader > long",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: left new");
    robot
        .tracking
        .set_current(Point2::new(-400.0, -1250.0), Angle::from_radians(1.84));
    // Drive to the left set of balls while intaking
    robot.intake.intake();
    let mut match_loader = robot.match_loader.clone();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-0.9.tiles(), -1.3.tiles()),
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .with_once_callback(
            |tracking| tracking.offset.y > -1.5.tiles(),
            move || {
                match_loader.extend();
            },
        )
        .await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.0.tiles(), -0.9.tiles()),
            CONFIG,
        ))
        .await;
    robot
        .drivetrain
        .action(
            libdoxa::subsystems::drivetrain::actions::RotationAction::new(
                (Angle::EIGHTH_TURN + Angle::HALF_TURN).as_radians(),
                CONFIG,
            ),
        )
        .await;
    sleep(Duration::from_millis(500)).await;
    // Go to the match loader
    robot.intake.intake();
    robot
        .drivetrain
        .action(boomerang_to_point(
            Point2::new(-1.87.tiles(), -2.5.tiles()),
            -Angle::QUARTER_TURN,
            CONFIG.with_boomerang_lead(0.65),
        ))
        .await;
    robot
        .drivetrain
        .action(turn_to_point(
            Point2::new(-2.0.tiles(), -4.0.tiles()),
            CONFIG,
        ))
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
                Point2::new(-1.9.tiles(), -1.35.tiles()),
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
    sleep(Duration::from_millis(3000)).await;
    // Hold the position while loading
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::new_voltage(-10.0, -10.0),
    }); // Intentionally not awaited
}
