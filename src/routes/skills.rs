use std::time::Duration;

use libdoxa::subsystems::drivetrain::{DrivetrainPair, actions::VoltageAction};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::subsystems::drivetrain_actions::{CONFIG, TileToMMExt, drive_to_point, forward};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Other,
    "Skills",
    "Same setup as left long first",
    route
);

/// Total duration for which the robot intakes at the match loader
const MATCH_LOAD_DURATION: Duration = Duration::from_millis(2750);
/// Total duration for which the robot outtakes into the goal
///
/// Must be greater than or equal to GOAL_CALIBRATE_DURATION
const GOAL_OUTTAKE_DURATION: Duration = Duration::from_millis(2000);
/// Duration for which the robot reverses before calibrating against the goal
const GOAL_CALIBRATE_DURATION: Duration = Duration::from_millis(800);

async fn match_load(robot: &mut crate::Robot) {
    // // Version with stop and start to attempt to reduce jams
    // robot.drivetrain.action(VoltageAction {
    //     voltage: DrivetrainPair::new_voltage(10.0, 10.0),
    // }); // Intentionally not awaited
    // robot.intake.intake();
    // sleep(MATCH_LOAD_DURATION / 2).await;
    // robot.intake.brake();
    // robot.drivetrain.action(VoltageAction {
    //     voltage: DrivetrainPair::from(0.0),
    // }); // Intentionally not awaited
    // sleep(Duration::from_millis(250)).await;
    // robot.drivetrain.action(VoltageAction {
    //     voltage: DrivetrainPair::from(0.0),
    // }); // Intentionally not awaited
    // robot.intake.intake();
    // sleep(MATCH_LOAD_DURATION / 2).await;
    // robot.intake.brake();
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::new_voltage(10.0, 10.0),
    }); // Intentionally not awaited
    robot.intake.intake();
    sleep(MATCH_LOAD_DURATION).await;
    robot.intake.brake();
}

enum Goal {
    Left,
    Right,
}

enum FieldSide {
    Near,
    Far,
}

async fn calibrate_and_outtake(robot: &mut crate::Robot, goal: Goal, side: FieldSide) {
    // 20 cm from inner tile

    // Back up against goal
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::from(-10.0),
    }); // Intentionally not awaited
    robot.intake.outtake_long_anti_jam().await;
    sleep(GOAL_CALIBRATE_DURATION).await;
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::ZERO,
    }); // Intentionally not awaited
    // Calibrate position against long goal once we're all the way back
    robot.tracking.set_current(
        Point2::new(
            match goal {
                Goal::Left => -2.0.tiles(),
                Goal::Right => 2.0.tiles(),
            },
            match side {
                FieldSide::Near => -600.0 - 200.0,
                FieldSide::Far => 600.0 + 200.0,
            },
        ),
        robot.tracking.current().heading,
    );
    sleep(GOAL_OUTTAKE_DURATION - GOAL_CALIBRATE_DURATION).await;
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
            Point2::new(-1.9.tiles(), 2.0.tiles()),
            CONFIG,
        ))
        .await;
    robot.match_loader.retract();
    robot
        .drivetrain
        .action(
            drive_to_point(
                Point2::new(-1.9.tiles(), 1.2.tiles()),
                CONFIG.with_linear_error_tolerance(100.0),
            )
            .reversed(),
        )
        .await;
    calibrate_and_outtake(robot, Goal::Left, FieldSide::Far).await;
    sleep(GOAL_OUTTAKE_DURATION).await;

    // MARK: match load 2
    // Head to the match loader
    robot.match_loader.extend();
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.99.tiles(), 2.0.tiles()),
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .await;
    robot
        .drivetrain
        .action(
            libdoxa::subsystems::drivetrain::actions::RotationAction::new(
                (Angle::QUARTER_TURN).as_radians(),
                CONFIG,
            ),
        )
        .await;
    // Hold the position while loading
    // Load with the macro
    match_load(robot).await;
    robot
        .drivetrain
        .action(forward(-200.0, CONFIG.with_linear_error_tolerance(60.0)))
        .await;

    // MARK: goal 2
    // Left long goal, top
    // Drive backwards to goal
    robot
        .drivetrain
        .action(
            drive_to_point(
                Point2::new(-2.0.tiles(), 1.2.tiles()),
                CONFIG.with_linear_error_tolerance(100.0),
            )
            .reversed(),
        )
        .await;
    // Calibrate position against long goal
    calibrate_and_outtake(robot, Goal::Left, FieldSide::Far).await;
    robot.drivetrain.action(forward(200.0, CONFIG)).await;

    // MARK: match load 3
    // Top right match loader
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(1.95.tiles(), 2.0.tiles()),
            CONFIG,
        ))
        .await;
    robot.intake.intake();
    robot.match_loader.extend();
    robot
        .drivetrain
        .action(
            libdoxa::subsystems::drivetrain::actions::RotationAction::new(
                (Angle::QUARTER_TURN).as_radians(),
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

    // MARK: goal 3
    // Drive to other side of goal (bottom of right long goal)
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(1.2.tiles(), 1.0.tiles()),
            CONFIG.with_linear_error_tolerance(60.0),
        ))
        .await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(1.2.tiles(), -1.0.tiles()),
            CONFIG.with_linear_error_tolerance(60.0),
        ))
        .await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(1.87.tiles(), -2.0.tiles()),
            CONFIG,
        ))
        .await;
    robot.match_loader.retract();
    robot
        .drivetrain
        .action(
            drive_to_point(
                Point2::new(1.87.tiles(), -1.2.tiles()),
                CONFIG.with_linear_error_tolerance(100.0),
            )
            .reversed(),
        )
        .await;
    calibrate_and_outtake(robot, Goal::Right, FieldSide::Near).await;
    robot
        .drivetrain
        .action(forward(200.0, CONFIG.with_linear_error_tolerance(60.0)))
        .await;

    // TODO: should we go for a 4th match load?

    // MARK: park
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(0.0.tiles(), -1.0.tiles()),
            CONFIG,
        ))
        .await;
    robot
        .drivetrain
        .action(
            libdoxa::subsystems::drivetrain::actions::RotationAction::new(
                (Angle::QUARTER_TURN).as_radians(),
                CONFIG,
            ),
        )
        .await;
    robot.drivetrain.action(VoltageAction {
        voltage: DrivetrainPair::from(-12.0),
    }); // Intentionally not awaited
}
