use nalgebra::Point2;
use vexide::math::Angle;

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Right,
    "Right primary",
    "Center goal > loader > long",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    robot
        .tracking
        .set_current(Point2::new(-400.0, -1250.0), Angle::from_radians(1.84));
}
