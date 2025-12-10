mod first;
mod none;
mod test;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum Category {
    Left,
    Other,
}

impl std::fmt::Display for Category {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Category::Left => write!(f, "Left"),
            Category::Other => write!(f, "Other"),
        }
    }
}

pub use first::FirstRoute;
pub use none::NoneRoute;
pub use test::TestRoute;
