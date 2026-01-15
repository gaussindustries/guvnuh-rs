use crate::models::status::{WorkerStatus, FaultCode}; // <--- Import Universal Status

pub fn run(led: &mut Pin<...>, ticks: u64) -> WorkerStatus {

    if ticks < 1000 {
        led.toggle();
        return WorkerStatus::Running; // Use the universal type
    }

    if ticks > 5000 {
         return WorkerStatus::Failed(FaultCode::Timeout);
    }

    WorkerStatus::Complete
}
