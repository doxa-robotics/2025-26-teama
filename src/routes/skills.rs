use std::time::Duration;

use libdoxa::{
    path_planner::cubic_parametric::CubicParametricPath,
    subsystems::drivetrain::{
        DrivetrainPair,
        actions::{PurePursuitAction, VoltageAction},
    },
};
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

async fn match_load(robot: &mut crate::Robot) {
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::new_voltage(10.0, 10.0),
    }); // Intentionally not awaited
    robot.intake.intake();
    sleep(Duration::from_millis(1500)).await;
    robot.intake.brake();
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::from(0.0),
    }); // Intentionally not awaited
    sleep(Duration::from_millis(250)).await;
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::from(0.0),
    }); // Intentionally not awaited
    robot.intake.intake();
    sleep(Duration::from_millis(1250)).await;
    robot.intake.brake();
}

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: skills");
    robot
        .tracking
        .set_current(Point2::new(-600.0 + 170.0, -1375.0), Angle::HALF_TURN);

    // MARK: match load 1
    // Head to the match loader first
    robot.drivetrain.action(forward(100.0, CONFIG)).await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.95.tiles(), -2.0.tiles()),
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
    // Load with the macro
    match_load(robot).await;
    robot
        .drivetrain
        .action(forward(-200.0, CONFIG.with_linear_error_tolerance(60.0)))
        .await;

    // MARK: goal 1
    // Drive to other side of goal
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.2.tiles(), -1.0.tiles()),
            CONFIG.with_linear_error_tolerance(60.0),
        ))
        .await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.2.tiles(), 1.0.tiles()),
            CONFIG.with_linear_error_tolerance(60.0),
        ))
        .await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-2.0.tiles(), 2.0.tiles()),
            CONFIG.with_linear_error_tolerance(60.0),
        ))
        .await;
    robot.match_loader.retract();
    robot
        .drivetrain
        .action(drive_to_point(Point2::new(-2.0.tiles(), 1.2.tiles()), CONFIG).reversed())
        .await;

    // Calibrate position against long goal
    robot.tracking.set_current(
        Point2::new(-2.0.tiles(), robot.tracking.current().offset.y),
        robot.tracking.current().heading,
    );

    robot.intake.outtake_long_anti_jam().await;
    sleep(Duration::from_millis(2750)).await;

    // MARK: match load 2
    // Head to the match loader
    robot.match_loader.extend();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.97.tiles(), 2.6.tiles()),
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .await;
    // Hold the position while loading
    // Load with the macro
    match_load(robot).await;
    robot
        .drivetrain
        .action(forward(-200.0, CONFIG.with_linear_error_tolerance(60.0)))
        .await;

    // MARK: goal 2
    // Drive backwards to goal
    robot
        .drivetrain
        .action(drive_to_point(Point2::new(-2.0.tiles(), 1.2.tiles()), CONFIG).reversed())
        .await;
    // Calibrate position against long goal
    robot.tracking.set_current(
        Point2::new(-2.0, robot.tracking.current().offset.y),
        robot.tracking.current().heading,
    );
    robot.match_loader.retract();

    robot.intake.outtake_long_anti_jam().await;
    sleep(Duration::from_millis(2750)).await;
    robot.drivetrain.action(forward(-200.0, CONFIG)).await;
}
