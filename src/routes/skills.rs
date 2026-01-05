use crate::routes::left_primary::left_center_match_load;

pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Other,
    "Skills",
    "Same setup as left primary",
    route
);

async fn route(robot: &mut crate::Robot) -> () {
    log::info!("Route: skills");
    left_center_match_load(robot).await;
}
