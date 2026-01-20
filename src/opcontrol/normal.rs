use core::time::Duration;

use libdoxa::subsystems::drivetrain::DrivetrainPair;
use vexide::prelude::*;

use crate::{Robot, subsystems::intake::Intake};

fn curve_drive(input: f64) -> f64 {
    let raw = input.powf(2.0);
    if input >= 0.0 { raw } else { -raw }
}

fn curve_turn(input: f64) -> f64 {
    let raw = input.powf(2.0);
    (if input >= 0.0 { raw } else { -raw }) / 2.0
}

pub async fn opcontrol(robot: &mut Robot) -> ! {
    robot.intake.brake();

    let mut double_park = false;
    let mut outtake_long_start: Option<std::time::Instant> = None;
    loop {
        let state = robot.controller.state().unwrap_or_default();

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
            // robot.intake.outtake_long();
            robot
                .intake
                .set_control(Some(crate::subsystems::intake::IntakeControl {
                    reverse: true,
                    run_intake: false,
                    outtake: crate::subsystems::intake::OuttakeMode::TopMiddle,
                    ..Default::default()
                }));
            outtake_long_start = Some(std::time::Instant::now());
        }
        // If outtake long has been held for more than the anti-jam duration,
        // switch to outtake long
        if let Some(start) = outtake_long_start
            && std::time::Instant::now().duration_since(start) >= Intake::OUTTAKE_REVERSE_DURATION
        {
            robot.intake.outtake_long();
            outtake_long_start = None;
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
            outtake_long_start = None;
        }

        // y is match load
        if state.button_a.is_now_pressed() {
            robot.match_loader.toggle();
        }

        // x is descore arm
        if state.button_x.is_now_pressed() {
            robot.descore_arm.toggle();
        }

        // power button is double park
        // I've always wanted to find a use for the power button
        if state.button_power.is_now_pressed() {
            double_park = !double_park;
            if double_park {
                let mut intake = robot.intake.clone();
                let mut double_park = robot.double_park.clone();
                vexide::task::spawn(async move {
                    // Reverse intake until the ball is positioned correctly.
                    intake.set_control(Some(crate::subsystems::intake::IntakeControl {
                        reverse: true,
                        outtake: crate::subsystems::intake::OuttakeMode::None,
                        speed: 0.5,
                        ..Default::default()
                    }));
                    intake.wait_for_ball(None).await;
                    vexide::time::sleep(Duration::from_millis(50)).await; // needs tuning
                    intake.set_control(Some(crate::subsystems::intake::IntakeControl {
                        reverse: true,
                        outtake: crate::subsystems::intake::OuttakeMode::None,
                        speed: 0.25,
                        ..Default::default()
                    }));
                    vexide::time::sleep(Duration::from_millis(50)).await;
                    intake.brake();
                    // Once the ball is in, extend the double park mechanism to lift
                    // the robot.
                    double_park.extend();
                })
                .detach();
            } else {
                robot.double_park.retract();
            }
        }

        sleep(Duration::from_millis(10)).await;
    }
}
