#![feature(never_type)]

use std::time::Duration;

use doxa_selector::{CompeteWithSelector, CompeteWithSelectorExt};
use libdoxa::subsystems::{
    drivetrain::Drivetrain,
    tracking::{TrackingSubsystem, wheel::TrackingWheel},
};
use vexide::{prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO};
use vexide_motorgroup::{SharedMotors, motor_group};

use crate::subsystems::{intake::Intake, lift::Lift, match_loader::MatchLoader};

mod opcontrol;
mod subsystems;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Category {
    Default,
}

impl std::fmt::Display for Category {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Category::Default => write!(f, "Default"),
        }
    }
}

struct Robot {
    controller: Controller,

    drivetrain: Drivetrain,
    tracking: TrackingSubsystem,

    intake: Intake,
    lift: Lift,
    match_loader: MatchLoader,
}

impl CompeteWithSelector for Robot {
    type Category = Category;
    type Return = ();

    fn autonomous_routes<'a, 'b>(
        &'b self,
    ) -> std::collections::BTreeMap<
        Self::Category,
        impl AsRef<[&'a dyn doxa_selector::AutonRoutine<Self, Return = Self::Return>]>,
    >
    where
        Self: 'a,
    {
        let mut map = std::collections::BTreeMap::new();
        map.insert(Category::Default, vec![]);
        map
    }

    async fn driver(&mut self) {
        while let Err(err) = opcontrol::normal::opcontrol(self).await {
            log::error!("Opcontrol error: {}", err);
            sleep(Duration::from_millis(100)).await;
        }
    }

    fn controller(&self) -> Option<&vexide::controller::Controller> {
        Some(&self.controller)
    }
}

#[vexide::main(banner(theme = THEME_OFFICIAL_LOGO))]
async fn main(peripherals: Peripherals) {
    // The drivetrain motors
    let left_motors = SharedMotors::new(motor_group![
        Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
    ]);
    let right_motors = SharedMotors::new(motor_group![
        Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
    ]);

    // Initialize the tracking context for odometry so we can share it with
    // Drivetrain
    let tracking = TrackingSubsystem::new::<RotationSensor, RotationSensor, InertialSensor>(
        [],
        [TrackingWheel::new(
            0.0,
            0.0,
            libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
            RotationSensor::new(peripherals.port_17, Direction::Forward),
        )],
        InertialSensor::new(peripherals.port_8),
    );

    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            left_motors,
            right_motors,
            Motor::V5_MAX_VOLTAGE,
            tracking.clone(),
            f64::INFINITY,
        ),
        intake: Intake::new(Motor::new_exp(peripherals.port_10, Direction::Forward)),
        lift: subsystems::lift::Lift::new(
            Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
            Motor::new_exp(peripherals.port_15, Direction::Forward),
        ),
        tracking: tracking.clone(),
        match_loader: MatchLoader::new([AdiDigitalOut::new(peripherals.adi_a)]),
    };

    robot.compete_with_selector(peripherals.display, None).await;
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_adds_two() {
        assert_eq!(2 + 2, 4);
    }
}
