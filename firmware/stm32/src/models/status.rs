// A generic status report for ANY state worker (Calibrate, Generate, etc.)
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum WorkerStatus {
    Running,           // "I'm busy, come back next tick"
    Complete,          // "Job done, ready for transition"
    Failed(FaultCode), // "I crashed" (Can carry extra info!)
}

// Optional: Define local fault codes if you don't use the Shared ones yet
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum FaultCode {
    Timeout,
    SensorError,
    WifiLost,
}
