use nalgebra::Point2;
use vexide::math::Angle;

use crate::subsystems::drivetrain_actions::{CONFIG, forward, turn_to_point};

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Other,
    "Test route",
    "Test route - do not run",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Test route");
    robot.tracking.set_current(Point2::origin(), Angle::ZERO);
    robot.drivetrain.action(forward(0.5, CONFIG)).await;
    // robot.drivetrain.action(forward(0.1, CONFIG)).await;
    robot
        .drivetrain
        .action(turn_to_point(Point2::new(0.0, 1.0), CONFIG))
        .await;
    robot.drivetrain.action(forward(0.1, CONFIG)).await;
}
