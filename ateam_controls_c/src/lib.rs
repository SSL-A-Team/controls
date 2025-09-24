use ateam_controls;

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_add(left: u64, right: u64) -> u64 {
  ateam_controls::add(left, right)
}
