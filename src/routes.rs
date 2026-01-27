mod left_long_first;
mod left_new;
mod left_primary;
mod none;
mod right_primary;
mod skills;
mod test;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Category {
    Left,
    Right,
    Other,
}

impl std::fmt::Display for Category {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Category::Left => write!(f, "Left"),
            Category::Right => write!(f, "Right"),
            Category::Other => write!(f, "Other"),
        }
    }
}

/// If returning Some(index), the route at that index will be run automatically
/// when testing with the legacy competition switch.
pub fn testing_route_index() -> Option<usize> {
    // would use a const but patcher is borked
    Some(6)
}

pub const ROUTES: &[doxa_selector::Route<Category, super::Robot>] = &[
    left_primary::ROUTE,
    left_long_first::ROUTE,
    left_new::ROUTE,
    right_primary::ROUTE,
    // Misc routes
    none::ROUTE,
    test::ROUTE,
    skills::ROUTE,
];
