pub const ROUTE: doxa_selector::Route<super::Category, crate::Robot> = doxa_selector::route!(
    super::Category::Other,
    "None",
    "No-op. Nothing.",
    route // comment to prevent rustfmt from onelining
);

async fn route(_robot: &mut crate::Robot) -> () {
    log::info!("Route: none");
    log::debug!("Doing nothing...");
}
