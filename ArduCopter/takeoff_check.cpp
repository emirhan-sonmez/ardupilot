#include "Copter.h"

//
// pre-takeoff checks
//

// detects if the vehicle should be allowed to takeoff or not and sets the motors.blocked flag
void Copter::takeoff_check()
{
#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    // if motors have become unblocked return immediately
    // this ensures the motors can only be blocked immediately after arming
    uint32_t now_ms = AP_HAL::millis();
    if (!motors->get_spoolup_block()) {
        takeoff_check_warning_ms = now_ms;
        takeoff_check_state.warning_ms = now_ms;
        return;
    }

    // Motors Library has enabled the spool up block.

    // Immediately clear the spool up block if not landed
    if (!ap.land_complete) {
        motors->set_spoolup_block(false);
        return;
    }

    // Run the common motor checks (called early so it can clear its warning timer when disarmed)
    const bool motor_check_passed = motors_takeoff_check(g2.takeoff_rpm_min, g2.takeoff_rpm_max);

    // Check system load
    float avg_load, peak_load;
    bool load_adequate = true;
    if (hal.util->get_system_load(avg_load, peak_load)) {
        if (avg_load > 95.0f || peak_load > 99.5f) {
            load_adequate = false;
        }
    }

    // Clear block if all checks passed
    if (motor_check_passed && load_adequate) {
        motors->set_spoolup_block(false);
        return;
    }

    // warn about CPU load every 2 seconds
    if (now_ms - takeoff_check_warning_ms > 2000) {
        takeoff_check_warning_ms = now_ms;
        const char* prefix_str = "Takeoff blocked:";
        if (!load_adequate) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "%s CPU overload (%4.1f%%)", prefix_str, avg_load);
        }
    }
#else
    /*
      No ESC telemetry on this build, so there are no takeoff checks to run --
      but the block still has to be released, and nothing else releases it.

      AP_MotorsMulticopter raises it unconditionally when spin-up completes
      ("Enable spoolup block to hold the aircraft in GROUND_IDLE. Main code
      should start checks when the block is enabled and remove the lock when
      ready"), and the only remover is the branch above. With the whole body
      compiled out the vehicle sits in GROUND_IDLE for ever: armed, motors at
      MOT_SPIN_ARM, throttle ignored and NO attitude mixing, with nothing
      reported anywhere. From the outside that is indistinguishable from
      "stabilisation does not work".

      Boards with ESC telemetry or CAN never see this because
      HAL_WITH_ESC_TELEM is true for them (AP_ESC_Telem_config.h). It bites
      minimal HALs -- here, AP_HAL_ChibiOS_K3, which has neither
      HAL_SUPPORT_RCOUT_SERIAL nor CAN drivers.

      Diagnosed 2026-08-03 from desired=2 (THROTTLE_UNLIMITED) with
      intlk=1 and spool stuck at 1 (GROUND_IDLE).
    */
    motors->set_spoolup_block(false);
#endif
}
