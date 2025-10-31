use libdoxa::subsystems::drivetrain::actions::{ForwardAction, config::ActionConfig};
use nalgebra::Point2;

use crate::{
    Robot,
    subsystems::drivetrain_actions::{CONFIG, forward, turn_to_point},
};

pub struct FirstRoute;

#[async_trait::async_trait]
impl doxa_selector::AutonRoutine<Robot> for FirstRoute {
    type Return = ();

    fn name(&self) -> &'static str {
        "First Auton"
    }

    fn description(&self) -> &'static str {
        "1st!!"
    }

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        robot
            .drivetrain
            .action(turn_to_point(Point2::new(-1.0, 0.0), CONFIG))
            .await;
    }
}
