use core::time::Duration;

use libdoxa::subsystems::drivetrain::DrivetrainPair;
use snafu::{ResultExt, Snafu};
use vexide::prelude::*;

use crate::Robot;

fn curve_drive(input: f64) -> f64 {
    let raw = input.powf(2.0);
    if input >= 0.0 { raw } else { -raw }
}

fn curve_turn(input: f64) -> f64 {
    let raw = input.powf(2.0);
    (if input >= 0.0 { raw } else { -raw }) / 2.0
}

#[derive(Debug, Snafu)]
pub enum OpcontrolError {
    #[snafu(display("Failed to control drivetrain: {}", source))]
    Drivetrain { source: vexide::smart::PortError },

    #[snafu(display("Failed to control intake: {}", source))]
    Intake {
        source: crate::subsystems::intake::IntakeError,
    },

    #[snafu(display("Failed to get controller state: {}", source))]
    ControllerState {
        source: vexide::controller::ControllerError,
    },
}

pub async fn opcontrol(robot: &mut Robot) -> Result<!, OpcontrolError> {
    robot.intake.update(None).context(IntakeSnafu)?;

    loop {
        let state = robot.controller.state().context(ControllerStateSnafu)?;

        let speed = curve_drive(state.left_stick.y());
        let turn = curve_turn(state.right_stick.x());

        let left_percent = (speed + turn).clamp(-1.0, 1.0);
        let right_percent = (speed - turn).clamp(-1.0, 1.0);

        robot.drivetrain.set_voltage(DrivetrainPair {
            left: Motor::V5_MAX_VOLTAGE * left_percent,
            right: Motor::V5_MAX_VOLTAGE * right_percent,
            units: libdoxa::subsystems::drivetrain::drivetrain_pair::DrivetrainUnits::Voltage,
        });

        if state.button_l1.is_pressed() {
            _ = robot.intake.outtake_long();
        } else if state.button_l2.is_pressed() {
            _ = robot.intake.outtake_top_middle();
        } else if state.button_r1.is_pressed() {
            _ = robot.intake.intake();
        } else if state.button_r2.is_pressed() {
            _ = robot.intake.reverse_intake();
        } else {
            _ = robot.intake.update(None);
        }

        // y is match load
        if state.button_a.is_now_pressed() {
            robot.match_loader.toggle();
        }

        // power button is double park
        if state.button_power.is_now_pressed() {
            robot.double_park.toggle();
        }

        println!("{:?}", robot.tracking.current());

        sleep(Duration::from_millis(10)).await;
    }
}
