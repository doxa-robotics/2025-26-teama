use snafu::ResultExt;
use vexide::smart::motor::Motor;

#[derive(Debug, snafu::Snafu)]
pub enum LiftError {
    #[snafu(display("Port error: {}", source))]
    Port { source: vexide::smart::PortError },
}

/// Represents the lift subsystem of the robot.
///
/// This subsystem is responsible for controlling the lift mechanism, which
/// lifts balls to different heights.
pub struct Lift {
    /// The motor responsible for the lower chain-driven lift mechanism.
    lower_lift_motor: Motor,
    /// The motor responsible for the upper mechanism, which determines the
    /// final height of the lift.
    ///
    /// If it spins in the same direction as the lower lift motor, it allows
    /// balls to reach the high goal. If it spins in the opposite direction,
    /// it allows squeezes balls to reach the mid goal.
    upper_lift_motor: Motor,
}

impl Lift {
    /// Creates a new instance of the Lift subsystem.
    ///
    /// # Returns
    ///
    /// A new `Lift` struct with the lower and upper lift motors initialized.
    pub fn new(lower_lift_motor: Motor, upper_lift_motor: Motor) -> Self {
        Self {
            lower_lift_motor,
            upper_lift_motor,
        }
    }

    /// Activates the lift motors to lift balls to the medium goal.
    pub fn lift_to_medium(&mut self) -> Result<(), LiftError> {
        // Lower lift motor spins forward to lift the balls.
        self.lower_lift_motor
            .set_voltage(self.lower_lift_motor.max_voltage())
            .context(PortSnafu)?;
        // Upper lift motor spins backward to position for medium goal.
        self.upper_lift_motor
            .set_voltage(-self.upper_lift_motor.max_voltage())
            .context(PortSnafu)?;
        Ok(())
    }

    /// Activates the lift motors to lift balls to the high goal.
    pub fn lift_to_high(&mut self) -> Result<(), LiftError> {
        // Lower lift motor spins forward to lift the balls.
        self.lower_lift_motor
            .set_voltage(self.lower_lift_motor.max_voltage())
            .context(PortSnafu)?;
        // Upper lift motor spins forward to position for high goal.
        self.upper_lift_motor
            .set_voltage(self.upper_lift_motor.max_voltage())
            .context(PortSnafu)?;
        Ok(())
    }

    /// Deactivates the lift motors.
    pub fn brake(&mut self) -> Result<(), LiftError> {
        // Brake to prevent balls from falling out.
        self.lower_lift_motor
            .brake(vexide::smart::motor::BrakeMode::Brake)
            .context(PortSnafu)?;
        self.upper_lift_motor
            .brake(vexide::smart::motor::BrakeMode::Brake)
            .context(PortSnafu)?;
        Ok(())
    }
}
