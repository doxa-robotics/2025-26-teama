mod first;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum Category {
    Left,
}

impl std::fmt::Display for Category {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Category::Left => write!(f, "Left"),
        }
    }
}

pub use first::FirstRoute;
