#![feature(never_type)]

use autons::prelude::{SelectCompete, SelectCompeteExt};
use libdoxa::{
    subsystems::{
        drivetrain::Drivetrain,
        tracking::{TrackingSubsystem, wheel::TrackingWheel},
    },
    utils::{logger, unwrap_expect_report::UnwrapExpectReportExt},
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

impl SelectCompete for Robot {
    async fn driver(&mut self) {
        log::info!("Lifecycle: driver");
        opcontrol::normal(self).await;
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
    left_motors: SharedMotors,
    right_motors: SharedMotors,
    intake_diagnostics: crate::subsystems::intake::IntakeDiagnostics,
    inertial: std::rc::Rc<std::cell::RefCell<InertialSensor>>,
}

impl doxa_selector::DoxaSelectInterface for DoxaSelectInterface {
    fn calibrating_enable(&self) -> bool {
        true
    }

    // We hold inertial over an await point. That is bad practice, because
    // immutable borrows can *panic*! However, we know that the only other place
    // that accesses the inertial is the tracking subsystem, which does a
    // `try_borrow` before accessing it, so we are safe.
    #[allow(clippy::await_holding_refcell_ref)]
    fn calibrating_calibrate(&mut self) {
        let inertial = self.inertial.clone();
        vexide::task::spawn(async move {
            log::info!("Calibrating gyro...");
            if let Ok(mut inertial) = inertial.try_borrow_mut() {
                if let Err(err) = inertial.calibrate().await {
                    log::error!("Gyro calibration failed: {}", err);
                } else {
                    log::info!("Gyro calibration complete.");
                }
            } else {
                log::error!("Could not borrow inertial for calibration.");
            }
        })
        .detach();
    }

    fn calibrating_calibrating(&self) -> bool {
        // If the inertial is unavailable, assume it's calibrating
        self.inertial
            .try_borrow()
            .map_or(true, |inertial| inertial.is_calibrating().unwrap_or(false))
    }

    fn diagnostics_enable(&self) -> bool {
        true
    }

    fn diagnostics_compact(&self) -> bool {
        true
    }

    fn diagnostics_diagnostics(&self) -> Vec<(String, String)> {
        let left_motor_temperatures = self
            .left_motors
            .temperature()
            .expect_report("couldn't read left temp");
        let right_motor_temperatures = self
            .right_motors
            .temperature()
            .expect_report("couldn't read right temp");
        let intake_temperatures = self.intake_diagnostics.motor_temperatures();
        vec![
            (
                "Battery".to_string(),
                format!("{:.0}%", vexide::battery::capacity() * 100.0),
            ),
            (
                "Left motors temperature".to_string(),
                left_motor_temperatures
                    // doxa-selector doesn't support ° symbol
                    .map_or_else(|| "Error!".to_string(), |temp| format!("{} C", temp)),
            ),
            (
                "Right motors temperature".to_string(),
                right_motor_temperatures
                    // doxa-selector doesn't support ° symbol
                    .map_or_else(|| "Error!".to_string(), |temp| format!("{} C", temp)),
            ),
            (
                "Intake motors temperature".to_string(),
                format!(
                    "{} C, {} C, {} C",
                    intake_temperatures.0, intake_temperatures.1, intake_temperatures.2,
                ),
            ),
            (
                "Competition control system".to_string(),
                vexide::competition::system().map_or_else(
                    || "Not connected".to_string(),
                    |system| format!("{:?}", system),
                ),
            ),
            (
                "Competition mode".to_string(),
                format!("{:?}", vexide::competition::mode()),
            ),
            (
                "VEXos uptime".to_string(),
                format!(
                    "{}m {}s",
                    vexide::time::system_uptime().as_secs() / 60,
                    vexide::time::system_uptime().as_secs() % 60
                ),
            ),
        ]
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

    let inertial = std::rc::Rc::new(std::cell::RefCell::new({
        let mut i = InertialSensor::new(peripherals.port_7);
        _ = i.set_rotation(Angle::ZERO);
        _ = i.set_heading(Angle::ZERO);
        i
    }));

    // Initialize the tracking context for odometry so we can share it with
    // Drivetrain
    let tracking = TrackingSubsystem::new::<
        RotationSensor,
        RotationSensor,
        std::rc::Rc<std::cell::RefCell<InertialSensor>>,
    >(
        [],
        [TrackingWheel::new(
            158.0,
            10.0,
            libdoxa::subsystems::tracking::wheel::TrackingWheelMountingDirection::Parallel,
            RotationSensor::new(peripherals.port_9, Direction::Reverse),
        )],
        inertial.clone(),
    );

    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            SharedMotors(left_motors.0.clone()), // FIXME: vexide_motorgroup didn't implement Clone
            SharedMotors(right_motors.0.clone()),
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

    let mut selector = doxa_selector::DoxaSelect::new(
        peripherals.display,
        routes::ROUTES,
        DoxaSelectInterface {
            left_motors,
            right_motors,
            intake_diagnostics: robot.intake.diagnostics(),
            inertial,
        },
    );

    // If we're connected to the old competition control system, then select the
    // route we're testing.
    if vexide::competition::system()
        == Some(vexide::competition::CompetitionSystem::CompetitionSwitch)
        && let Some(route) = routes::TESTING_ROUTE
    {
        selector.select(route);
    }
    robot.compete(selector).await;
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_adds_two() {
        assert_eq!(2 + 2, 4);
    }
}
