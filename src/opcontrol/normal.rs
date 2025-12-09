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
    #[snafu(display("Failed to get controller state: {}", source))]
    ControllerState {
        source: vexide::controller::ControllerError,
    },
}

pub async fn opcontrol(robot: &mut Robot) -> Result<!, OpcontrolError> {
    robot.intake.brake();

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

        // L1 starts outtake long
        if state.button_l1.is_now_pressed() {
            robot.intake.outtake_long();
        }
        // L2 starts outtake top middle
        if state.button_l2.is_now_pressed() {
            robot.intake.outtake_top_middle();
        }
        // R1 starts intake
        if state.button_r1.is_now_pressed() {
            robot.intake.intake();
        }
        // R2 starts reverse intake
        if state.button_r2.is_now_pressed() {
            robot.intake.reverse_intake();
        }
        // Any button released stops intake if all intake buttons are released
        if (state.button_l1.is_now_released()
            || state.button_l2.is_now_released()
            || state.button_r1.is_now_released()
            || state.button_r2.is_now_released())
            && !(state.button_l1.is_pressed()
                || state.button_l2.is_pressed()
                || state.button_r1.is_pressed()
                || state.button_r2.is_pressed())
        {
            robot.intake.brake();
        }

        // y is match load
        if state.button_a.is_now_pressed() {
            robot.match_loader.toggle();
        }

        // power button is double park
        // I've always wanted to find a use for the power button
        if state.button_power.is_now_pressed() {
            let mut intake = robot.intake.clone();
            let mut double_park = robot.double_park.clone();
            vexide::task::spawn(async move {
                // Reverse intake until the ball is positioned correctly.
                intake.reverse_intake();
                intake.wait_for_ball(None).await;
                intake.brake(); // TODO: adjust timing based on testing
                vexide::time::sleep(Duration::from_millis(500)).await;
                // Once the ball is in, extend the double park mechanism to lift
                // the robot.
                double_park.extend();
            })
            .detach();
        }

        // println!("{:?}", robot.tracking.current());

        sleep(Duration::from_millis(10)).await;
    }
}
