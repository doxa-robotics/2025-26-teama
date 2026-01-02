pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Other,
    "Skills",
    "Same setup as left primary",
    route
);

async fn route(_robot: &mut crate::Robot) -> () {}
