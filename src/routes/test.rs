use nalgebra::Point2;
use vexide::math::Angle;

use crate::{
    Robot,
    subsystems::drivetrain_actions::{CONFIG, forward},
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
    }
}
