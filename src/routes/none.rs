use crate::Robot;

pub struct NoneRoute;

#[async_trait::async_trait]
impl doxa_selector::AutonRoutine<Robot> for NoneRoute {
    type Return = ();

    fn name(&self) -> &'static str {
        "No route"
    }

    fn description(&self) -> &'static str {
        "Do nothing."
    }

    async fn run(&self, _robot: &mut Robot) -> Self::Return {
        log::info!("No-op route");
    }
}
