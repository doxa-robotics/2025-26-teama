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

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: skills");
    robot
        .tracking
        .set_current(Point2::new(-399.0, -1375.0), Angle::HALF_TURN);
    // Head to the match loader first
    robot.drivetrain.action(forward(100.0, CONFIG)).await;
    robot
        .drivetrain
        .action(drive_to_point(
            Point2::new(-1.9.tiles(), -2.0.tiles()),
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
    sleep(Duration::from_millis(1500)).await;

    // Drive to other side of goal
    robot
        .drivetrain
        .action(
            PurePursuitAction::new(
                CubicParametricPath::new(
                    Point2::new(-1200.0, -1200.0),
                    -Angle::QUARTER_TURN,
                    2882.0,
                    Point2::new(-1200.0, 1200.0),
                    Angle::from_radians(2.4085543677521746),
                    4214.0,
                ),
                CONFIG,
            )
            .reversed(),
        )
        .await;
}
