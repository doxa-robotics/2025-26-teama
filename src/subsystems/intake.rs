use snafu::ResultExt;
use vexide::smart::motor::Motor;

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
pub struct Intake {
    /// The motor responsible for the front intake mechanism.
    ///
    /// It powers the intake and the first stage of the lift.
    intake: Motor,
    /// The motor responsible for the middle lift stage.
    middle: Motor,
    /// The motor responsible for the top lift stage and direction control.
    ///
    /// Forward direction results in upper long goal outtake, reverse direction
    /// results in middle goal outtake.
    top: Motor,

    /// The current control state of the intake subsystem.
    control: Option<IntakeControl>,
}

impl Intake {
    /// Creates a new instance of the Intake subsystem.
    pub fn new(intake: Motor, middle: Motor, top: Motor) -> Self {
        Self {
            intake,
            middle,
            top,
            control: None,
        }
    }

    pub fn update(&mut self, control: Option<IntakeControl>) -> Result<(), IntakeError> {
        self.control = control;

        if let Some(control) = control {
            let factor = if control.reverse { -1.0 } else { 1.0 };
            self.intake
                .set_voltage(factor * self.intake.max_voltage())
                .context(FrontIntakeSnafu {})?;
            self.middle
                .set_voltage(factor * self.middle.max_voltage())
                .context(FrontIntakeSnafu {})?;

            match control.outtake {
                OuttakeMode::Long => {
                    self.top
                        .set_voltage(factor * self.top.max_voltage())
                        .context(FrontIntakeSnafu {})?;
                }
                OuttakeMode::TopMiddle => {
                    self.top
                        .set_voltage(factor * -self.top.max_voltage())
                        .context(FrontIntakeSnafu {})?;
                }
                OuttakeMode::None => {
                    self.top
                        .brake(vexide::smart::motor::BrakeMode::Brake)
                        .context(FrontIntakeSnafu {})?;
                }
            }
        } else {
            self.intake
                .brake(vexide::smart::motor::BrakeMode::Coast)
                .context(FrontIntakeSnafu {})?;
            self.top
                .brake(vexide::smart::motor::BrakeMode::Coast)
                .context(FrontIntakeSnafu {})?;
            self.middle
                .brake(vexide::smart::motor::BrakeMode::Coast)
                .context(FrontIntakeSnafu {})?;
        }
        Ok(())
    }

    /// Get the current control state
    pub fn control(&self) -> Option<IntakeControl> {
        self.control
    }

    /// Intake
    pub fn intake(&mut self) -> Result<(), IntakeError> {
        self.update(Some(IntakeControl {
            reverse: false,
            outtake: OuttakeMode::None,
        }))
    }

    /// Reverse intake
    pub fn reverse_intake(&mut self) -> Result<(), IntakeError> {
        self.update(Some(IntakeControl {
            reverse: true,
            outtake: OuttakeMode::None,
        }))
    }

    /// Outtake to long goal
    pub fn outtake_long(&mut self) -> Result<(), IntakeError> {
        self.update(Some(IntakeControl {
            reverse: false,
            outtake: OuttakeMode::Long,
        }))
    }

    /// Outtake to the top middle goal
    pub fn outtake_top_middle(&mut self) -> Result<(), IntakeError> {
        self.update(Some(IntakeControl {
            reverse: false,
            outtake: OuttakeMode::TopMiddle,
        }))
    }
}
