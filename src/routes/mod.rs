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

pub const ROUTES: [doxa_selector::Route<Category, super::Robot>; 5] = [
    left_primary::ROUTE,
    right_primary::ROUTE,
    // Misc routes
    none::ROUTE,
    test::ROUTE,
    skills::ROUTE,
];
