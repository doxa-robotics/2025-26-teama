use vexide::smart::PortError;

/// A global store to hold ports which have had a disconnect error reported.
/// This is used to avoid spamming the logs with repeated disconnect errors.
static DEVICE_DISCONNECTED_PORTS: std::sync::Mutex<Option<std::collections::HashSet<u8>>> =
    std::sync::Mutex::new(None);

pub trait DeviceDisconnectedErrorExt<T> {
    /// Reports a disconnect if there is an error, otherwise returns the value.
    ///
    /// If there is an error, this function will log it and return None.
    ///
    /// # Panics
    ///
    /// If the port/device type is mismatched, this function will *panic* to avoid
    /// masking configuration errors.
    fn report_if_error(self) -> Option<T>;
}

impl<T> DeviceDisconnectedErrorExt<T> for Result<T, PortError> {
    fn report_if_error(self) -> Option<T> {
        match self {
            Err(err) => match err {
                PortError::Disconnected { port } => {
                    let mut locked = DEVICE_DISCONNECTED_PORTS
                        .lock()
                        .expect("could not lock mutex. this should never happen.");
                    if locked.is_none() {
                        *locked = Some(std::collections::HashSet::new());
                    }
                    let ports = locked.as_mut().unwrap();
                    if !ports.contains(&port) {
                        ports.insert(port);
                        log::error!("Device disconnected on port {}", port);
                    }
                    None
                }
                PortError::IncorrectDevice {
                    port,
                    expected,
                    actual,
                } => {
                    panic!(
                        "Mismatched device type on port {}: expected {:?}, found {:?}",
                        port, expected, actual
                    )
                }
            },
            Ok(value) => Some(value),
        }
    }
}
