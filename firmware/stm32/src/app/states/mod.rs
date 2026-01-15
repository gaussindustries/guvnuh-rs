#[path="0_boot.rs"]
pub mod boot;

#[path="1_calibrate.rs"]
pub mod calibrate;

#[path="2_idle.rs"]
pub mod idle;

#[path="3_spoolup_turbines.rs"]
pub mod spoolup_turbines;

#[path="4_excite.rs"]
pub mod excite;

#[path="5_pll_lock.rs"]
pub mod ;

#[path="6_ready.rs"]
pub mod ready;

#[path="7_generate.rs"]
pub mod generate;

#[path="8_load_rejection.rs"]
pub mod load_rejection;

#[path="9_fault.rs"]
pub mod fault;

#[path="10_estop.rs"]
pub mod estop;
