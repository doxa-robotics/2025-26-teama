use std::{cell::RefCell, rc::Rc};

use libdoxa::utils::unwrap_expect_report::UnwrapExpectReportExt as _;
use vexide::{
    prelude::DistanceSensor,
    smart::{SmartDevice, motor::Motor},
};

#[derive(Clone, Copy, Debug)]
pub enum OuttakeMode {
    Long,
    TopMiddle,
    None,
}

#[derive(Clone, Copy, Debug)]
pub struct IntakeControl {
    pub reverse: bool,
    pub outtake: OuttakeMode,
}

impl Default for IntakeControl {
    fn default() -> Self {
        Self {
            reverse: false,
            outtake: OuttakeMode::None,
        }
    }
}

/// A future that waits until the intake has detected a ball.
///
/// Output is `true` if a ball was detected, `false` if the timeout was reached.
///
/// # What is a future?
///
/// A future is a struct that represents a value that may not be available yet.
/// Every tick of the executor, the future is polled (asked) to see if it is
/// ready. In this case, the future checks whether the intake has detected a ball.
/// If it has, the future returns `Poll::Ready(true)`, telling the executor that
/// the value is ready and the future is complete. If the timeout is reached before
/// a ball is detected, it returns `Poll::Ready(false)` (Ready but no ball). If neither
/// condition is met, it returns `Poll::Pending`, indicating that the future is
/// still waiting and the executor should continue polling it in future ticks
/// (asking again later).
#[derive(Clone, Debug)]
pub struct BallDetectedFuture {
    intake: Intake,
    timeout: Option<std::time::Instant>,
}

impl std::future::Future for BallDetectedFuture {
    type Output = bool;

    fn poll(
        self: std::pin::Pin<&mut Self>,
        _cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Self::Output> {
        let ball_detected = *self.intake.ball_detected.borrow();

        if ball_detected {
            // Ball detected
            std::task::Poll::Ready(true)
        } else if let Some(timeout) = self.timeout
            && timeout <= std::time::Instant::now()
        {
            // There's a timeout and it has been reached
            std::task::Poll::Ready(false)
        } else {
            std::task::Poll::Pending
        }
    }
}

/// Represents the intake subsystem of the robot.
///
/// This subsystem is responsible for controlling the intake mechanism,
/// which includes the front intake motor and (later in the season) the color
/// sorter mechanism.
///
/// # Hardware
///
/// - Front Intake motor: Powers the intake mechanism and the first stage of the lift.
/// - Middle Lift motor: Powers the middle stage of the lift.
/// - Top Lift motor: Powers the top stage of the lift and controls the direction
///   for outtaking to different goals.
#[derive(Clone, Debug)]
pub struct Intake {
    /// The current control state of the intake subsystem.
    control: Rc<RefCell<Option<IntakeControl>>>,
    /// Whether there is a ball currently detected in the intake.
    ball_detected: Rc<RefCell<bool>>,

    _task: Rc<vexide::task::Task<()>>,
}

impl Intake {
    /// Creates a new instance of the Intake subsystem.
    pub fn new(
        mut intake: Motor,
        mut middle: Motor,
        mut top: Motor,
        ball_presence_sensor: DistanceSensor,
        ball_presence_threshold: u32,
    ) -> Self {
        let control = Rc::new(RefCell::new(None));
        let ball_detected = Rc::new(RefCell::new(false));

        Self {
            control: control.clone(),
            ball_detected: ball_detected.clone(),
            _task: Rc::new(vexide::task::spawn(async move {
                loop {
                    // Avoid burning the CPU
                    vexide::time::sleep(Motor::UPDATE_INTERVAL).await;

                    // If the control state is set, apply it
                    if let Some(control) = *control.borrow() {
                        // Move backwards if reverse is set, otherwise move forwards
                        let factor = if control.reverse { -1.0 } else { 1.0 };
                        intake
                            .set_voltage(factor * intake.max_voltage())
                            .unwrap_report();
                        middle
                            .set_voltage(factor * middle.max_voltage())
                            .unwrap_report();

                        match control.outtake {
                            OuttakeMode::Long => {
                                top.set_voltage(factor * top.max_voltage()).unwrap_report();
                            }
                            OuttakeMode::TopMiddle => {
                                top.set_voltage(factor * -top.max_voltage()).unwrap_report();
                            }
                            OuttakeMode::None => {
                                top.brake(vexide::smart::motor::BrakeMode::Hold)
                                    .unwrap_report();
                            }
                        }
                    } else {
                        // No control state, set all motors to coast
                        // Friction will hold balls in place
                        intake
                            .brake(vexide::smart::motor::BrakeMode::Coast)
                            .unwrap_report();
                        top.brake(vexide::smart::motor::BrakeMode::Coast)
                            .unwrap_report();
                        middle
                            .brake(vexide::smart::motor::BrakeMode::Coast)
                            .unwrap_report();
                    }

                    if let Ok(object) = ball_presence_sensor.object() {
                        // If there's an object and it's within the threshold distance, we have a ball
                        *ball_detected.borrow_mut() =
                            object.is_some_and(|object| object.distance < ball_presence_threshold);
                    }
                }
            })),
        }
    }

    /// Get the current control state
    pub fn control(&self) -> Option<IntakeControl> {
        *self.control.borrow()
    }

    /// Intake
    pub fn intake(&mut self) {
        self.control.replace(Some(IntakeControl {
            reverse: false,
            outtake: OuttakeMode::None,
        }));
    }

    /// Reverse intake
    pub fn reverse_intake(&mut self) {
        self.control.replace(Some(IntakeControl {
            reverse: true,
            outtake: OuttakeMode::None,
        }));
    }

    /// Outtake to long goal
    pub fn outtake_long(&mut self) {
        self.control.replace(Some(IntakeControl {
            reverse: false,
            outtake: OuttakeMode::Long,
        }));
    }

    /// Outtake to the top middle goal
    pub fn outtake_top_middle(&mut self) {
        self.control.replace(Some(IntakeControl {
            reverse: false,
            outtake: OuttakeMode::TopMiddle,
        }));
    }

    /// Stop all intake actions
    pub fn brake(&mut self) {
        self.control.replace(None);
    }

    /// Wait until a ball is detected or the timeout is reached.
    ////
    /// If `timeout` is `None`, waits indefinitely.
    #[must_use = "futures do nothing unless awaited"]
    pub fn wait_for_ball(&self, timeout: Option<std::time::Duration>) -> BallDetectedFuture {
        BallDetectedFuture {
            intake: self.clone(),
            timeout: timeout.map(|t| std::time::Instant::now() + t),
        }
    }
}
