#![feature(never_type)]

use std::time::Duration;

use autons::prelude::{SelectCompete, SelectCompeteExt};
use libdoxa::{
    subsystems::{
        drivetrain::Drivetrain,
        tracking::{TrackingSubsystem, wheel::TrackingWheel},
    },
    utils::logger,
};
use vexide::{math::Angle, prelude::*, startup::banner::themes::THEME_OFFICIAL_LOGO};
use vexide_motorgroup::{SharedMotors, motor_group};

use crate::subsystems::{double_park::DoublePark, intake::Intake, match_loader::MatchLoader};

mod opcontrol;
mod routes;
mod subsystems;
mod utils;

struct Robot {
    controller: Controller,

    drivetrain: Drivetrain,
    tracking: TrackingSubsystem,

    intake: Intake,
    match_loader: MatchLoader,

    double_park: DoublePark,
}

// SAFETY: single-threaded
unsafe impl Send for Robot {}
unsafe impl Sync for Robot {}

impl SelectCompete for Robot {
    async fn driver(&mut self) {
        log::info!("Lifecycle: driver");
        loop {
            let Err(err) = opcontrol::normal::opcontrol(self).await;
            log::error!("Opcontrol error: {}", err);
            sleep(Duration::from_millis(100)).await;
        }
    }

    async fn after_route(&mut self) {
        log::info!("Lifecycle: after route");
    }

    async fn before_route(&mut self) {
        log::info!("Lifecycle: before route");
    }

    async fn connected(&mut self) {
        log::info!("Lifecycle: connected");
    }

    async fn disabled(&mut self) {
        log::info!("Lifecycle: disabled");
    }

    async fn disconnected(&mut self) {
        log::info!("Lifecycle: disconnected");
    }
}

struct DoxaSelectInterface {
    gyro_calibrating: std::rc::Rc<std::cell::RefCell<bool>>,
}

impl doxa_selector::DoxaSelectInterface for DoxaSelectInterface {
    fn calibrating_enable(&self) -> bool {
        true
    }

    fn calibrating_calibrate(&mut self) {
        log::info!("Calibrating...");
    }

    fn calibrating_calibrating(&self) -> std::rc::Rc<std::cell::RefCell<bool>> {
        self.gyro_calibrating.clone()
    }

    fn diagnostics_enable(&self) -> bool {
        true
    }

    fn diagnostics_compact(&self) -> bool {
        true
    }

    fn diagnostics_diagnostics(&self) -> Vec<(String, String)> {
        vec![]
    }
}

#[vexide::main(banner(theme = THEME_OFFICIAL_LOGO))]
async fn main(peripherals: Peripherals) {
    // Initialize logging
    logger::init("2025-26-teama.log", log::LevelFilter::Debug)
        .expect("could not set logger. already set?");

    // The drivetrain motors
    let left_motors = SharedMotors::new(motor_group![
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
    ]);
    let right_motors = SharedMotors::new(motor_group![
        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
    ]);

    // Initialize the tracking context for odometry so we can share it with
    // Drivetrain
    let tracking = TrackingSubsystem::new::<RotationSensor, RotationSensor, InertialSensor>(
        [],
        [TrackingWheel::new(
            158.0,
            10.0,
            libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
            RotationSensor::new(peripherals.port_9, Direction::Reverse),
        )],
        {
            let mut i = InertialSensor::new(peripherals.port_7);
            _ = i.set_rotation(Angle::ZERO);
            _ = i.set_heading(Angle::ZERO);
            i
        },
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
        intake: Intake::new(
            Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
            Motor::new_exp(peripherals.port_3, Direction::Forward),
            Motor::new_exp(peripherals.port_18, Direction::Reverse),
            DistanceSensor::new(peripherals.port_13),
            100,
        ),
        tracking: tracking.clone(),
        match_loader: MatchLoader::new([AdiDigitalOut::new(peripherals.adi_a)]),
        double_park: DoublePark::new([AdiDigitalOut::new(peripherals.adi_b)]),
    };

    let gyro_calibrating = robot.tracking.gyro_calibrating().clone();
    robot
        .compete(doxa_selector::DoxaSelect::new(
            peripherals.display,
            routes::ROUTES,
            DoxaSelectInterface { gyro_calibrating },
        ))
        .await;
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_adds_two() {
        assert_eq!(2 + 2, 4);
    }
}
