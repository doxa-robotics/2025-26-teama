use std::time::Duration;

use libdoxa::subsystems::drivetrain::{DrivetrainPair, actions::VoltageAction};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::subsystems::drivetrain_actions::{
    CONFIG, TileToMMExt as _, boomerang_to_point, drive_to_point, seeking_to_point, turn_to_point,
};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Left,
    "Left primary",
    "Center goal > loader > long",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: left primary");
    robot
        .tracking
        .set_current(Point2::new(-400.0, -1250.0), Angle::from_radians(1.84));
    // Drive to the left set of balls while intaking
    robot.intake.intake();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.0.tiles(), -1.0.tiles()),
            CONFIG,
        ))
        .await;
    // Outtake balls into the center top goal
    robot
        .drivetrain
        .action(turn_to_point(Point2::new(-0.5.tiles(), -0.5.tiles()), CONFIG).reversed())
        .await;
    robot
        .drivetrain
        // TODO: this action is timing out, probably because we miss the point
        // slightly and fail the tolerances. Maybe use dot product instead of norm
        // to determine closeness in libdoxa?
        .action(seeking_to_point(Point2::new(-0.5.tiles(), -0.5.tiles()), CONFIG).reversed())
        .await;
    robot.intake.outtake_top_middle();
    sleep(Duration::from_millis(1000)).await;
    // Go to the match loader
    robot.intake.brake();
    let mut match_loader = robot.match_loader.clone();
    robot
        .drivetrain
        .action(boomerang_to_point(
            Point2::new(-1.9.tiles(), -2.5.tiles()),
            -Angle::QUARTER_TURN + Angle::from_radians(0.1),
            CONFIG.with_boomerang_lead(0.6),
        ))
        .with_callback(move |tracking| {
            if tracking.offset.y < -1.0.tiles() {
                match_loader.extend();
            }
        })
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
    let mut triggered = false;
    robot
        .drivetrain
        .action(drive_to_point(Point2::new(-2.0.tiles(), -1.35.tiles()), CONFIG).reversed())
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
    sleep(Duration::from_millis(4000)).await;
    robot.intake.brake();
}
