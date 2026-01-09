use std::time::Duration;

use libdoxa::subsystems::drivetrain::{DrivetrainPair, actions::VoltageAction};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::subsystems::drivetrain_actions::{
    CONFIG, TileToMMExt as _, boomerang_to_point, drive_to_point, forward,
};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Left,
    "Left long first",
    "Loader > long > center goal",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: left long first");
    robot
        .tracking
        .set_current(Point2::new(-385.0, -1417.0), Angle::HALF_TURN);
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
    sleep(Duration::from_millis(2700)).await;
    robot.drivetrain.action(forward(150.0, CONFIG)).await;
    robot.intake.intake();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.0.tiles(), -1.0.tiles()),
            CONFIG,
        ))
        .await;
    sleep(Duration::from_millis(800)).await;
    robot
        .drivetrain
        .action(drive_to_point(Point2::new(-0.6.tiles(), -0.6.tiles()), CONFIG).reversed())
        .await;
    robot.intake.outtake_top_middle();
    robot
        .drivetrain
        .action(
            libdoxa::subsystems::drivetrain::actions::RotationAction::new(
                (Angle::EIGHTH_TURN + Angle::HALF_TURN).as_radians(),
                CONFIG,
            ),
        )
        .await;
}
