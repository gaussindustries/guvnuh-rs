/**
 * honestly thinking about using the yellow led as a indicator light.
 * perhaps 3 second in between the .5 intervals to count
 *  at the very least this should be done to indicate which state it was last in if in fault, certainly
 * fault red
 *
 * generate -> green
 *
 *  pll lock -> flash fast and flash slower up until we lock, then we blink green once ready
 * flash yellur
 */
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[repr(u8)]
pub enum STATE {
    BOOT = 0,
    /**
       init clocks
       gpio
       adc
       ETH interface for MODBUS
       timers,comparators,
       dma
       watchdog (look into this more)
    */
    CALIBRATE = 1,
    /**
        adc,
        ensure sensors are within tolerance,
         gpio ( calling Apolla 9 )
            {
                sending test input for expected output, vise versa
            }
        async hang until we get that the esp32 has fully booted &&
            {
                tells us that we have a lock on api.gaussindustri.es
                we run through all possible messages to confirm correct telemetry will be sent (esp32's own calibration)
                on completion of all neccessary esp32 functions we proceed
            }

    */
    IDLE = 2,
    /// RunConfig received, waiting for Start command
    /**
     * condition green, we've landed on mars
     * sends signal to esp32 to show all systems nominal, waits for esp32 to recieve 200 in order to proceed
     * once we recieve command via interface (from terminal - esp32, thus to demand to change state)
    	*/
    CONFIGURED = 3,
    SPOOLUP = 4,
    /**
    * set target voltages, phase/freq
    * spin up dc motor simulating work being performed on shaft based on below

    as for posterity, ensuring this dovetails into steam power et al:
        closed boiler sys
           fuel type:
               wood/oil
               methane (fartz)
           ensuring superheaters are within tolerant temperatures at all times
               i/o for flash steam valves
           measuring crucible temperature

    */
    EXCITE = 5,
    /**
    * ensure our self excited induction generator (we would upgrade to vfd to not have this mess about in the future)

    */
    PLL_LOCK = 6,
    /**
    * PLL (def): ogase-locked loop has matched it';s output clock's frequency and phase ot the ref (VFD)
        Phase/Frequency detector -> loop filter -> VCO / DCO -> dividers
    */
    READY = 7,
    /**
     * gate not open yet to start using power
     */
    GENERATE = 8,
    /**
     * once signal has been recieved we open gate (or auto if set on spoolup)
     */
    LOAD_REJECTION = 9,
    /// Graceful ramp-down in progress (from any running state via Stop command)
    /**
     * react to drastic change (delta change of amps/load) from 100% to 0%, vise versa
     */
    RAMP_DOWN = 10,
    /// Live manual control from desktop terminal
    MANUAL = 11,
    FAULT = 12,
    /**
    * based on priority from sensors && || gather loop/func of course write the fault enum to be read and sent to overlay
    this should of course send as much info as possible for debugging
    */
    ESTOP = 13,
}

/**
 * once button is pressed (detected in real time)
 * set state back to idle, or rather go down safely or spool down of course send proper telemetry
 */
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum Fault {
    OverVoltage,
    UnderVoltage,
    OverCurrent,
    OverTemp,
    NoExcitation,
    PllUnlock,
    SensorOutOfRange,
    CommLost,
    Overspeed,
    DeadlineMiss,
}

impl STATE {
    pub fn as_str(self) -> &'static str {
        match self {
            STATE::BOOT => "Boot",
            STATE::CALIBRATE => "Calibrate",
            STATE::IDLE => "Idle",
            STATE::CONFIGURED => "Configured",
            STATE::SPOOLUP => "Spool Up",
            STATE::EXCITE => "Excite",
            STATE::PLL_LOCK => "PLL Lock",
            STATE::READY => "Ready",
            STATE::GENERATE => "Generate",
            STATE::LOAD_REJECTION => "Load Rejection",
            STATE::RAMP_DOWN => "Ramp Down",
            STATE::MANUAL => "Manual",
            STATE::FAULT => "FAULT",
            STATE::ESTOP => "ESTOP",
        }
    }

    /// Is the system in a state where the motor/generator is energized?
    pub fn is_running(self) -> bool {
        matches!(
            self,
            STATE::SPOOLUP
                | STATE::EXCITE
                | STATE::PLL_LOCK
                | STATE::READY
                | STATE::GENERATE
                | STATE::LOAD_REJECTION
                | STATE::CALIBRATE
                | STATE::MANUAL
                | STATE::RAMP_DOWN
        )
    }

    /// Is it safe to accept a Configure command?
    pub fn accepts_configure(self) -> bool {
        matches!(self, STATE::IDLE | STATE::CONFIGURED)
    }

    /// Is it safe to accept a Start command?
    pub fn accepts_start(self) -> bool {
        matches!(self, STATE::CONFIGURED)
    }
}
