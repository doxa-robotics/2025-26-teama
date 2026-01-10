use std::time::Duration;

use libdoxa::subsystems::drivetrain::{DrivetrainPair, actions::VoltageAction};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::subsystems::drivetrain_actions::{
    CONFIG, TileToMMExt as _, boomerang_to_point, drive_to_point, forward, turn_to_point,
};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Right,
    "Right primary",
    "Center goal > loader > long",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    robot
        .tracking
        .set_current(Point2::new(400.0, -1250.0), Angle::from_radians(1.30));
    // Drive to the right set of balls while intaking
    robot.intake.intake();
    let mut match_loader = robot.match_loader.clone();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(1.0.tiles(), -1.2.tiles()),
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .with_once_callback(
            |tracking| tracking.offset.y > -1.5.tiles(),
            move || {
                match_loader.extend();
            },
        )
        .await;
    // Outtake balls into the center lower goal, facing forwards
    robot.match_loader.retract();
    robot.intake.brake();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(0.65.tiles(), -0.65.tiles()),
            CONFIG,
        ))
        .await;
    // RUnning and not running to un-jam the ball
    robot.intake.reverse_intake();
    robot
        .drivetrain
        .action(
            libdoxa::subsystems::drivetrain::actions::RotationAction::new(
                (Angle::HALF_TURN - Angle::EIGHTH_TURN).as_radians(),
                CONFIG,
            ),
        )
        .await;
    sleep(Duration::from_millis(400)).await;
    robot.intake.brake();
    sleep(Duration::from_millis(150)).await;
    robot.intake.reverse_intake();
    robot.drivetrain.action(forward(70.0, CONFIG)).await;
    sleep(Duration::from_millis(100)).await;
    robot.intake.intake();
    // Move backwards to avoid hitting the goal
    robot.drivetrain.action(forward(-130.0, CONFIG)).await;
    // Orient towards the match loader
    robot
        .drivetrain
        .action(
            libdoxa::subsystems::drivetrain::actions::RotationAction::new(
                (-Angle::EIGHTH_TURN + Angle::HALF_TURN).as_radians(),
                CONFIG,
            ),
        )
        .await;
    // Go to the match loader
    robot.intake.intake();
    let mut match_loader = robot.match_loader.clone();
    robot
        .drivetrain
        .action(boomerang_to_point(
            Point2::new(1.77.tiles(), -2.5.tiles()),
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
            Point2::new(2.0.tiles(), -4.0.tiles()),
            CONFIG,
        ))
        .await;
    // Hold the position while loading
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::new_voltage(10.0, 10.0),
    }); // Intentionally not awaited
    robot.intake.intake();
    sleep(Duration::from_millis(1000)).await;
    // Outtake into the long goal
    robot.intake.brake();
    let intake = robot.intake.clone();
    robot
        .drivetrain
        .action(
            drive_to_point(
                Point2::new(1.9.tiles(), -1.35.tiles()),
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
    robot.match_loader.retract();
}
