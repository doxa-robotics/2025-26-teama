use std::time::Duration;

use libdoxa::subsystems::drivetrain::actions::{ForwardAction, config::ActionConfig};
use nalgebra::Point2;
use vexide::{math::Angle, time::sleep};

use crate::{
    Robot,
    subsystems::drivetrain_actions::{CONFIG, drive_to_point, forward, turn_to_point},
};

pub struct LeftPrimaryRoute;

#[async_trait::async_trait]
impl doxa_selector::AutonRoutine<Robot> for LeftPrimaryRoute {
    type Return = ();

    fn name(&self) -> &'static str {
        "Left primary"
    }

    fn description(&self) -> &'static str {
        "Bottom left side autonomous routine."
    }

    async fn run(&self, robot: &mut Robot) -> Self::Return {
        robot
            .tracking
            .set_current(Point2::new(-400.0, -1250.0), Angle::from_radians(1.84));
        robot.intake.intake();
        robot
            .drivetrain
            .action(drive_to_point(Point2::new(-1.0, -1.0), CONFIG))
            .await;
        robot
            .drivetrain
            .action(drive_to_point(Point2::new(-0.5, -0.5), CONFIG).reversed())
            .await;
        robot.intake.outtake_top_middle();
        sleep(Duration::from_millis(500)).await;
        return;
        robot.intake.intake();
        sleep(Duration::from_millis(3000)).await;
        robot.intake.brake();

        robot.intake.intake();
        robot
            .drivetrain
            .action(drive_to_point(Point2::new(-2.0, -2.0), CONFIG))
            .await;
        robot
            .drivetrain
            .action(turn_to_point(Point2::new(-2.0, -2.8), CONFIG))
            .await;

        robot
            .drivetrain
            .action(drive_to_point(Point2::new(-2.0, -2.8), CONFIG))
            .await;
        robot.intake.intake();
        sleep(Duration::from_millis(3000)).await;
        robot.intake.brake();

        // robot.drivetrain.action(reverse(-2.0, -1.0)).await;
        robot.intake.intake();
        sleep(Duration::from_millis(3000)).await;
        robot.intake.brake();
    }
}
