use nalgebra::Point2;
use vexide::math::Angle;

use crate::{
    Robot,
    subsystems::drivetrain_actions::{CONFIG, forward, turn_to_point},
};

pub struct TestRoute;

#[async_trait::async_trait]
impl doxa_selector::AutonRoutine<Robot> for TestRoute {
    type Return = ();

    fn name(&self) -> &'static str {
        "Test"
    }

    fn description(&self) -> &'static str {
        "Do not use"
    }

    async fn run(&self, robot: &mut Robot) -> Self::Return {
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
}
