use std::time::Duration;

use libdoxa::subsystems::drivetrain::{DrivetrainPair, actions::VoltageAction};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::subsystems::drivetrain_actions::{
    CONFIG, TileToMMExt as _, boomerang_to_point, drive_to_point, turn_to_point,
};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Left,
    "Left primary",
    "Center goal > loader > long",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: left primary");
    left_center_match_load(robot, false).await;
}

/// Left center goal to match loader route, starting with the documented corner-
/// to-corner starting position.
///
/// This is extracted to serve as a common route for both left_primary and
/// skills.
pub(super) async fn left_center_match_load(robot: &mut crate::Robot, is_skills: bool) {
    robot
        .tracking
        .set_current(Point2::new(-400.0, -1250.0), Angle::from_radians(1.84));
    // Drive to the left set of balls while intaking
    robot.intake.intake();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.0.tiles(), -1.2.tiles()),
            CONFIG,
        ))
        .await;
    // Outtake balls into the center top goal
    robot
        .drivetrain
        .action(drive_to_point(Point2::new(-0.6.tiles(), -0.6.tiles()), CONFIG).reversed())
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
    robot.intake.outtake_top_middle();
    sleep(Duration::from_millis(1300)).await;
    // Go to the match loader
    robot.intake.intake();
    let mut match_loader = robot.match_loader.clone();
    robot
        .drivetrain
        .action(boomerang_to_point(
            Point2::new(-1.8.tiles(), -2.5.tiles()),
            -Angle::QUARTER_TURN,
            CONFIG.with_boomerang_lead(0.65),
        ))
        .with_callback(move |tracking| {
            if tracking.offset.y < -1.0.tiles() {
                match_loader.extend();
            }
        })
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
    sleep(if is_skills {
        Duration::from_millis(3000)
    } else {
        Duration::from_millis(1200)
    })
    .await;
    // Outtake into the long goal
    robot.intake.brake();
    let mut match_loader = robot.match_loader.clone();
    let intake = robot.intake.clone();
    let mut triggered = false;
    robot
        .drivetrain
        .action(
            drive_to_point(
                Point2::new(-2.0.tiles(), -1.35.tiles()),
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
    sleep(if is_skills {
        Duration::from_millis(8000)
    } else {
        Duration::from_millis(3000)
    })
    .await;
}
