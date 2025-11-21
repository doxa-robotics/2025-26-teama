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

    #[snafu(display("Failed to control lift: {}", source))]
    Lift {
        source: crate::subsystems::lift::LiftError,
    },
}

pub async fn opcontrol(robot: &mut Robot) -> Result<!, OpcontrolError> {
    robot.intake.brake().context(IntakeSnafu)?;

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

        // (r1, r2, l1, l2)
        if state.button_r1.is_pressed()
            || state.button_r2.is_pressed()
            || state.button_l1.is_pressed()
            || state.button_l2.is_pressed()
        {
            // If the left triggers are pressed, it controls the lift.
            // If the right triggers are pressed, the intake works, but the lift
            // lower motor should still spin.

            if state.button_l2.is_pressed() {
                // High lift
                robot.lift.lift_to_high().context(LiftSnafu)?;
            } else if state.button_l1.is_pressed() {
                // Low lift
                robot.lift.lift_to_medium().context(LiftSnafu)?;
            } else {
                // Follow the right triggers, which dictate the intake.
                robot
                    .lift
                    .intake(if state.button_r2.is_pressed() {
                        Direction::Forward
                    } else {
                        Direction::Reverse
                    })
                    .context(LiftSnafu)?;
            }
        } else {
            robot.lift.brake().context(LiftSnafu)?;
        }

        if state.button_r2.is_pressed() {
            robot.intake.activate_front_intake().context(IntakeSnafu)?;
        } else if state.button_r1.is_pressed() {
            robot.intake.reverse_front_intake().context(IntakeSnafu)?;
        } else {
            robot.intake.brake().context(IntakeSnafu)?;
        }

        // y is match load

        if state.button_y.is_now_pressed() {
            robot.match_loader.toggle();
        }

        println!("{:?}", robot.tracking.current());

        sleep(Duration::from_millis(10)).await;
    }
}
