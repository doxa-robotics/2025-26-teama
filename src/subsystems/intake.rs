use snafu::ResultExt;
use vexide::smart::motor::Motor;

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
    /// It is connected to the front flex wheel intake via a chain.
    front_intake: Motor,
}

impl Intake {
    /// Creates a new instance of the Intake subsystem.
    ///
    /// # Returns
    ///
    /// A new `Intake` struct with the front intake motor initialized.
    pub fn new(front_intake: Motor) -> Self {
        Self { front_intake }
    }

    /// Activates the front intake motor to intake objects.
    pub fn activate_front_intake(&mut self) -> Result<(), IntakeError> {
        self.front_intake
            .set_voltage(self.front_intake.max_voltage())
            .context(FrontIntakeSnafu)?;
        Ok(())
    }

    /// Reverse the front intake motor to outtake objects.
    pub fn reverse_front_intake(&mut self) -> Result<(), IntakeError> {
        self.front_intake
            .set_voltage(-self.front_intake.max_voltage())
            .context(FrontIntakeSnafu)?;
        Ok(())
    }

    /// Deactivates the front intake.
    pub fn brake(&mut self) -> Result<(), IntakeError> {
        // We coast to reduce thermal buildup as a result of braking
        self.front_intake
            .brake(vexide::smart::motor::BrakeMode::Coast)
            .context(FrontIntakeSnafu)?;
        Ok(())
    }
}
