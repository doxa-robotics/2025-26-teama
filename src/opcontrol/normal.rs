use core::time::Duration;

use libdoxa::subsystems::drivetrain::DrivetrainPair;
use snafu::{ResultExt, Snafu};
use vexide::prelude::*;

use crate::Robot;

const OUTTAKE_REVERSE_DURATION: Duration = Duration::from_millis(150);

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

    let mut double_park = false;
    let mut outtake_long_start: Option<std::time::Instant> = None;
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
            // robot.intake.outtake_long();
            robot
                .intake
                .set_control(Some(crate::subsystems::intake::IntakeControl {
                    reverse: true,
                    run_intake: false,
                    ..Default::default()
                }));
            outtake_long_start = Some(std::time::Instant::now());
        }
        // If outtake long has been held for more than 500ms, switch to outtake long
        if let Some(start) = outtake_long_start
            && std::time::Instant::now().duration_since(start) >= OUTTAKE_REVERSE_DURATION
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
                    vexide::time::sleep(Duration::from_millis(50)).await;
                    intake.set_control(Some(crate::subsystems::intake::IntakeControl {
                        reverse: true,
                        outtake: crate::subsystems::intake::OuttakeMode::None,
                        speed: 0.25,
                        ..Default::default()
                    }));
                    vexide::time::sleep(Duration::from_millis(50)).await;
                    intake.brake();
                    // vexide::time::sleep(Duration::from_millis(50)).await;
                    // Once the ball is in, extend the double park mechanism to lift
                    // the robot.
                    double_park.extend();
                    vexide::time::sleep(Duration::from_millis(100)).await;
                    // intake.intake();
                    vexide::time::sleep(Duration::from_millis(200)).await;
                    intake.brake();
                })
                .detach();
            } else {
                robot.double_park.retract();
            }
        }

        // println!("{:?}", robot.tracking.current());
        {
            let mut display = unsafe {
                // SAFETY: not safe
                vexide::display::Display::new()
            };
            display.fill(
                &vexide::display::Rect::from_dimensions(
                    vexide::math::Point2 { x: 300, y: 50 },
                    150,
                    100,
                ),
                vexide::color::Color::WHITE,
            );
            display.draw_text(
                &vexide::display::Text::from_string(
                    format!("{:.2?}", robot.tracking.current().offset),
                    vexide::display::Font::new(
                        vexide::display::FontSize::MEDIUM,
                        vexide::display::FontFamily::Monospace,
                    ),
                    vexide::math::Point2 { x: 305, y: 55 },
                ),
                vexide::color::Color::BLACK,
                None,
            );
        }

        sleep(Duration::from_millis(10)).await;
    }
}
