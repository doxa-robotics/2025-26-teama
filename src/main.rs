#![feature(never_type)]

use std::time::Duration;

use doxa_selector::{CompeteWithSelector, CompeteWithSelectorExt};
use libdoxa::subsystems::{
    drivetrain::Drivetrain,
    tracking::{TrackingSubsystem, wheel::TrackingWheel},
};
use vexide::{prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO};
use vexide_motorgroup::{SharedMotors, motor_group};

use crate::{
    routes::Category,
    subsystems::{intake::Intake, lift::Lift, match_loader::MatchLoader},
};

mod opcontrol;
mod routes;
mod subsystems;

struct Robot {
    controller: Controller,

    drivetrain: Drivetrain,
    tracking: TrackingSubsystem,

    intake: Intake,
    lift: Lift,
    match_loader: MatchLoader,
}

// SAFETY: single-threaded
unsafe impl Send for Robot {}
unsafe impl Sync for Robot {}

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
        let mut map: std::collections::BTreeMap<
            Category,
            Vec<&dyn doxa_selector::AutonRoutine<Self, Return = Self::Return>>,
        > = std::collections::BTreeMap::new();
        map.insert(Category::Left, vec![&routes::FirstRoute]);
        map
    }

    async fn driver(&mut self) {
        loop {
            let Err(err) = opcontrol::normal::opcontrol(self).await;
            log::error!("Opcontrol error: {}", err);
            sleep(Duration::from_millis(100)).await;
        }
    }

    fn controller(&self) -> Option<&vexide::controller::Controller> {
        Some(&self.controller)
    }

    fn is_gyro_calibrating(&self) -> bool {
        self.tracking.is_gyro_calibrating()
    }

    fn diagnostics(&self) -> Vec<(String, String)> {
        vec![
            ("Mood of robot".to_string(), "Happy".to_string()),
            (
                "Mood of drive team".to_string(),
                "Hopefully happy".to_string(),
            ),
            (
                "Heading".to_string(),
                format!("{:?}", self.tracking.current().heading),
            ),
        ]
    }
}

#[vexide::main(banner(theme = THEME_OFFICIAL_LOGO))]
async fn main(peripherals: Peripherals) {
    // The drivetrain motors
    let left_motors = SharedMotors::new(motor_group![
        Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
    ]);
    let right_motors = SharedMotors::new(motor_group![
        Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
    ]);

    // Initialize the tracking context for odometry so we can share it with
    // Drivetrain
    let tracking = TrackingSubsystem::new::<RotationSensor, SharedMotors, InertialSensor>(
        [],
        [
            TrackingWheel::new(
                565.0,
                0.0,
                libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
                SharedMotors(left_motors.0.clone()),
            ),
            TrackingWheel::new(
                565.0,
                0.0,
                libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
                SharedMotors(right_motors.0.clone()),
            ),
        ],
        InertialSensor::new(peripherals.port_15),
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
        intake: Intake::new(Motor::new(
            peripherals.port_12,
            Gearset::Blue,
            Direction::Forward,
        )),
        lift: subsystems::lift::Lift::new(
            Motor::new_exp(peripherals.port_20, Direction::Forward),
            Motor::new_exp(peripherals.port_11, Direction::Forward),
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
