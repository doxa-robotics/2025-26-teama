use std::time::Duration;

use libdoxa::subsystems::drivetrain::actions::{ForwardAction, config::ActionConfig};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::{
    Robot,
    subsystems::drivetrain_actions::{CONFIG, drive_to_point, forward, rotation, turn_to_point},
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
            .tracking
            .set_current(Point2::new(400.0, -1200.0), Angle::ZERO);
        robot.drivetrain.action(forward(1.25, CONFIG)).await;
        robot
            .drivetrain
            .action(rotation(-Angle::QUARTER_TURN.as_radians(), CONFIG))
            .await;
        robot.match_loader.extend();
        _ = robot.intake.activate_front_intake();
        sleep(Duration::from_millis(500)).await;
        robot.drivetrain.action(forward(0.4, CONFIG)).await;
        robot.drivetrain.action(forward(-0.4, CONFIG)).await;
        robot.drivetrain.action(forward(0.4, CONFIG)).await;
        robot.drivetrain.action(forward(-1.0, CONFIG)).await;
        robot.match_loader.retract();
    }
}
