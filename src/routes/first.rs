use std::time::Duration;

use libdoxa::subsystems::drivetrain::actions::{ForwardAction, config::ActionConfig};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::{
    Robot,
    subsystems::drivetrain_actions::{CONFIG, forward, rotation, turn_to_point},
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
        robot.drivetrain.action(rotation(-0.11, CONFIG)).await;
        sleep(Duration::from_millis(300)).await;
        _ = robot.intake.reverse_front_intake();
        sleep(Duration::from_millis(300)).await;
        robot.drivetrain.action(forward(4.0, CONFIG)).await;
        robot.drivetrain.action(forward(1.0, CONFIG)).await;
        sleep(Duration::from_millis(500)).await;
        robot
            .drivetrain
            .action(rotation(Angle::QUARTER_TURN.as_radians(), CONFIG))
            .await;
        sleep(Duration::from_millis(3000)).await;
        robot.drivetrain.action(forward(5.5, CONFIG)).await;
        robot.drivetrain.action(forward(0.5, CONFIG)).await;
        robot
            .drivetrain
            .action(rotation((Angle::QUARTER_TURN * 1.5).as_radians(), CONFIG))
            .await;
        robot.drivetrain.action(forward(-0.8, CONFIG)).await;
        robot.drivetrain.action(forward(-0.4, CONFIG)).await;
        _ = robot.lift.lift_to_medium();
        sleep(Duration::from_millis(3000)).await;
        _ = robot.lift.brake();
        _ = robot.intake.brake();
    }
}
