use std::{cell::RefCell, rc::Rc};

use snafu::ResultExt;
use vexide::smart::{SmartDevice, motor::Motor};

use crate::utils::device_disconnected_error::DeviceDisconnectedErrorExt;

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

#[derive(Debug, snafu::Snafu)]
pub enum IntakeError {
    #[snafu(display("Failed to control front intake motor: {}", source))]
    FrontIntake { source: vexide::smart::PortError },
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
pub struct Intake {
    /// The current control state of the intake subsystem.
    control: Rc<RefCell<Option<IntakeControl>>>,

    _task: vexide::task::Task<()>,
}

impl Intake {
    /// Creates a new instance of the Intake subsystem.
    pub fn new(mut intake: Motor, mut middle: Motor, mut top: Motor) -> Self {
        let control = Rc::new(RefCell::new(None));

        Self {
            control: control.clone(),
            _task: vexide::task::spawn(async move {
                loop {
                    vexide::time::sleep(Motor::UPDATE_INTERVAL);

                    if let Some(control) = *control.borrow() {
                        let factor = if control.reverse { -1.0 } else { 1.0 };
                        intake
                            .set_voltage(factor * intake.max_voltage())
                            .report_if_error();
                        middle
                            .set_voltage(factor * middle.max_voltage())
                            .report_if_error();

                        match control.outtake {
                            OuttakeMode::Long => {
                                top.set_voltage(factor * top.max_voltage())
                                    .report_if_error();
                            }
                            OuttakeMode::TopMiddle => {
                                top.set_voltage(factor * -top.max_voltage())
                                    .report_if_error();
                            }
                            OuttakeMode::None => {
                                top.brake(vexide::smart::motor::BrakeMode::Hold)
                                    .report_if_error();
                            }
                        }
                    } else {
                        intake
                            .brake(vexide::smart::motor::BrakeMode::Coast)
                            .report_if_error();
                        top.brake(vexide::smart::motor::BrakeMode::Coast)
                            .report_if_error();
                        middle
                            .brake(vexide::smart::motor::BrakeMode::Coast)
                            .report_if_error();
                    }
                }
            }),
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
}
