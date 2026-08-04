#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "bench_passthrough.h"
#include "RCOutput.h"           // ChibiOS_K3::RCOutput::write_exclusive()
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <hal.h>                // AM67_EPWM0_BASE (board.h)
#include <am67_epwm.h>          // ehrpwm_read_cmp (shadow readback, diagnostics only)
#include "hwdef/boot/trace.h"

extern const AP_HAL::HAL& hal;

/*
  Bench RC->PWM passthrough for the four quad-X outputs, ported line-for-line
  in structure (interlocks and all) from the hardware-verified ChibiOS demo:
  demos/various/RT-GEMSTONE-O1-R5F/main.c, the GEMSTONE_IBUS_TEST block.
  iBus decode itself is not reimplemented -- ChibiOS_K3::RCInput already
  feeds AP_RCProtocol, this just reads the decoded channels back out.

  Deliberately independent of ArduPilot's own vehicle/scheduler/AP_Motors/
  SRV_Channels path: the real motor mixer is explicitly out of scope for
  this milestone (Empty::Storage cannot persist SERVOn_FUNCTION/
  FRAME_CLASS/FRAME_TYPE parameters). This is NOT a flight controller: no
  attitude stabilisation, no vehicle arming/failsafe logic beyond what's
  implemented below. Do not fly this.

  SAFETY, all mandatory:
    - PROPELLERS OFF.
    - ESC power from a separate BEC/battery, never a board header rail.
    - ESC supply ground bonded to board ground.
    - Airframe restrained.

  Interlocks (identical to the ChibiOS demo):
    - outputs held at PT_IDLE_US until explicitly armed;
    - arming requires an OFF->ON edge on the arm switch AND throttle already
      at minimum at that instant, so holding the switch on with the
      throttle up cannot arm;
    - loss of valid frames for PT_FAILSAFE_MS disarms and idles;
    - throttle at/below PT_FS_THR_US while armed disarms (RC failsafe --
      requires transmitter-side failsafe configured to drive throttle below
      1000us on signal loss; this FS-iA10B otherwise holds the last value
      and keeps streaming at full rate with the transmitter off, so this
      alone cannot catch transmitter loss).
    - No watchdog exists (M9): a faulted R5F latches the last commanded
      pulse width indefinitely.
*/

namespace
{

constexpr uint16_t PT_IDLE_US         = 1000;
constexpr uint16_t PT_MIN_US          = 1000;
constexpr uint16_t PT_MAX_US          = 2000;
constexpr uint16_t PT_ARM_HIGH_US     = 1700;   // arm switch considered ON above
constexpr uint16_t PT_ARM_LOW_US      = 1300;   // ...and OFF below (hysteresis)
constexpr uint16_t PT_THR_MIN_GATE_US = 1050;   // throttle must be under this to arm
constexpr uint16_t PT_FS_THR_US       = 950;    // at/below this, transmitter link is gone
constexpr uint32_t PT_FAILSAFE_MS     = 200;    // no valid frame for this -> idle

// Slew-rate limit, climbing only -- not in the ChibiOS demo this file ports
// (checked: main.c's GEMSTONE_IBUS_TEST block has no ramp logic either), new
// for the first-ever powered-ESC test. Cuts (failsafe, disarm, stick pulled
// down) stay instant; only the climb toward higher throttle is smoothed, so
// a fast stick movement right after arming can't snap a motor from idle to
// max in one ~7.7ms iBus frame.
//
// DISABLED 2026-07-30 (PT_RAMP_ENABLED 0): retuned 333 -> 600 us/s (~3s -> ~1.7s
// for a full 1000->2000 stroke) and the throttle response was still judged too
// slow to work with on the bench, so the limiter is off and the commanded value
// is applied instantly. Rate authority moves to ArduCopter's own mixer/PID once
// QGroundControl parameter configuration exists; this limiter was only ever a
// stand-in for that, for the first powered-ESC test. Re-enable by setting
// PT_RAMP_ENABLED to 1 -- the rate below is kept tuned and ready.
//
// Consequence while disabled: a full stick slam steps a motor from idle to max
// in a single tick, with nothing between the receiver and the ESC. The arm
// interlocks and the instant-cut paths (disarm, RC failsafe, frame timeout) are
// unaffected -- they never went through the limiter.
#define PT_RAMP_ENABLED 0
constexpr uint16_t PT_RAMP_US_PER_SEC = 600;

// Debounce for the throttle-idle gate, 2026-07-30: a single bad iBus frame
// occasionally decoded throttle below PT_THR_MIN_GATE_US even while the
// stick was held at max, snapping all four motors to idle for one tick --
// confirmed on hardware, not explained by mixing. Not in the ChibiOS demo
// this file ports (checked -- no filtering there either), new. Costs a
// ~2-tick (~13ms at this port's loop rate) delay before a genuine
// throttle-down is honored; deliberately small, and only applies to this
// specific gate -- everything else (RC failsafe, frame timeout) stays
// instant.
constexpr uint8_t PT_THR_LOW_DEBOUNCE_TICKS = 2;

// iBus channel indices (0-based). Confirmed on hardware in the ChibiOS demo.
constexpr uint8_t IB_ROLL  = 0;
constexpr uint8_t IB_PITCH = 1;
constexpr uint8_t IB_THR   = 2;
constexpr uint8_t IB_YAW   = 3;
constexpr uint8_t IB_ARM   = 4;   // iBus channel 5
constexpr uint8_t IB_MIN_CHANNELS = 5;

constexpr uint8_t PT_NUM_MOTORS   = 4;
constexpr int32_t PT_MIX_GAIN_PCT = 30;   // % of full stick deflection
constexpr int32_t PT_ROLL_SIGN    = 1;
constexpr int32_t PT_PITCH_SIGN   = 1;
constexpr int32_t PT_YAW_SIGN     = 1;

struct MotorMix {
    int16_t roll_f;    // mixing factors, scaled x1000
    int16_t pitch_f;
    int16_t yaw_f;
    uint8_t out;        // RCOutput channel index
};

// ArduPilot Quad X numbering, matching the working ChibiOS demo and the
// six-channel ChibiOS_K3::RCOutput pin map.
constexpr MotorMix pt_motor[PT_NUM_MOTORS] = {
    { -707,  707,  1000, 0 },  // M1 front-right, CCW -> out0, pin 29
    {  707, -707,  1000, 1 },  // M2 back-left,   CCW -> out1, pin 31
    {  707,  707, -1000, 2 },  // M3 front-left,  CW  -> out2, pin 33
    { -707, -707, -1000, 3 },  // M4 back-right,  CW  -> out3, pin 32
};

bool started;
bool armed;
bool arm_gate_ok;       // an unconsumed OFF->ON switch edge is available
bool last_armed;
uint32_t last_good_frame_ms;
uint32_t last_report_ms;
uint32_t last_ramp_ms;  // for the climb-only slew limiter, below
uint8_t thr_low_count;  // consecutive ticks with throttle below the idle gate
int32_t last_thr = -1000, last_roll = -1000, last_pitch = -1000, last_yaw = -1000;
uint16_t motor_us[PT_NUM_MOTORS] = { PT_IDLE_US, PT_IDLE_US, PT_IDLE_US, PT_IDLE_US };
uint16_t g_ch[IB_MIN_CHANNELS] = { 1500, 1500, 1000, 1500, 1000 };

// Q-34: writes must go through the owner-only path, otherwise the exclusive
// mask set in HAL_ChibiOS_K3::run() would drop this module's own writes too.
// hal.rcout is always the ChibiOS_K3::RCOutput instance on this board (see
// HAL_ChibiOS_K3_Class.cpp) -- there is no other backend to be.
void pt_set(uint8_t out, uint16_t us)
{
    if (us < PT_MIN_US) {
        us = PT_MIN_US;
    }
    if (us > PT_MAX_US) {
        us = PT_MAX_US;
    }
    static_cast<ChibiOS_K3::RCOutput *>(hal.rcout)->write_exclusive(out, us);
}

void pt_all_idle()
{
    for (uint8_t i = 0; i < PT_NUM_MOTORS; i++) {
        motor_us[i] = PT_IDLE_US;
        pt_set(pt_motor[i].out, PT_IDLE_US);
    }
}

}  // namespace

namespace ChibiOS_K3
{

void bench_passthrough_update()
{
    const uint32_t now_ms = AP_HAL::millis();

    if (!started) {
        started = true;
        last_good_frame_ms = now_ms;
        last_report_ms = now_ms;
        last_ramp_ms = now_ms;

        // ArduCopter's own AP_Motors/RC_SPEED init calls
        // hal.rcout->set_freq() during callbacks->loop()'s first pass
        // through vehicle setup, with an ESC-oriented rate -- observed on
        // hardware 2026-07-30 as 400 Hz, not the 50 Hz every us<->register
        // calculation in this file assumes (see the tbprd=62500 comment
        // below). ArduPlane's equivalent servo-output init used 50 Hz,
        // which is why this was never visible before the vehicle switch:
        // bench_passthrough never called set_freq() itself, it silently
        // depended on whatever the vehicle's own init picked.
        //
        // set_freq() re-programs the timebase of any already-started
        // peripheral (RCOutput.cpp:41-62), not just ones starting fresh,
        // so this reliably wins regardless of init order -- same "runs
        // last, unconditionally overwrites" principle as the motor_us[]
        // reassertion below, extended to frequency. All 6 physical
        // channels are forced, not just the 4 this module drives: the
        // boot-time safe-init in HAL_ChibiOS_K3_Class.cpp starts all 6,
        // and any of them could have something real connected.
        hal.rcout->set_freq(0x3F, 50);
        trace_printf("pt: forced RCOutput to 50 Hz (was whatever AP_Motors requested)\n");

        trace_printf("pt: DISARMED. To arm: throttle DOWN, arm switch OFF then ON. "
                     "PROPELLERS OFF.\n");
    }

    // Time-based, not tick-count-based -- immune to main loop rate jitter.
    // Computed once per call and reused below, whether or not the armed
    // mixing branch actually runs this tick.
    const uint32_t ramp_dt_ms = now_ms - last_ramp_ms;
    last_ramp_ms = now_ms;
    const int32_t ramp_max_step =
        (int32_t)(((uint64_t)PT_RAMP_US_PER_SEC * ramp_dt_ms) / 1000U);

    // Deliberately NOT gated on AP::RC().new_input(): that flag is
    // consume-on-read (AP_RCProtocol::new_input() sets it false as soon as
    // anyone calls it), and Plane::read_radio() -- an AP_Scheduler task
    // inside callbacks->loop(), which runs before this in the main loop --
    // already consumes it every tick for its own purposes. Gating on it
    // here meant this block simply never ran with a receiver actually
    // connected: chans=14 and rate=16kB/s confirmed on hardware in
    // RCInput's own diagnostic, while thr/r/p/y stayed frozen at their
    // startup defaults. AP_RCProtocol::read() is a pure getter (returns the
    // backend's latest stored value, no side effects) -- safe to call every
    // tick unconditionally. This does mean the frame-timeout failsafe below
    // is best-effort (num_channels() staying >= 5 is treated as "still
    // receiving", not "a specific new frame arrived this tick"), not a
    // regression in practice: DR-011 already established that a
    // frame-timeout failsafe cannot catch this receiver's actual failure
    // mode anyway (it keeps streaming held values with the transmitter
    // off) -- the throttle-value failsafe just below is the real
    // protection here, and it does not depend on this distinction at all.
    if (AP::RC().num_channels() >= IB_MIN_CHANNELS) {
        last_good_frame_ms = now_ms;
        g_ch[IB_ROLL]  = AP::RC().read(IB_ROLL);
        g_ch[IB_PITCH] = AP::RC().read(IB_PITCH);
        g_ch[IB_THR]   = AP::RC().read(IB_THR);
        g_ch[IB_YAW]   = AP::RC().read(IB_YAW);
        g_ch[IB_ARM]   = AP::RC().read(IB_ARM);

        // RC failsafe by throttle threshold.
        if (armed && (g_ch[IB_THR] <= PT_FS_THR_US)) {
            armed = false;
            arm_gate_ok = false;
            pt_all_idle();
            trace_printf("pt: RC FAILSAFE (thr=%u <= %u) -> DISARMED, outputs idle\n",
                         (uint32_t)g_ch[IB_THR], (uint32_t)PT_FS_THR_US);
        }

        // Arm state machine, edge triggered: arming happens only on a fresh
        // OFF->ON transition of the switch, and only if the throttle is at
        // minimum at that instant. The edge is consumed either way.
        if (g_ch[IB_ARM] < PT_ARM_LOW_US) {
            if (armed) {
                armed = false;
                pt_all_idle();
                trace_printf("pt: DISARMED (arm switch off)\n");
            }
            arm_gate_ok = true;
        } else if ((g_ch[IB_ARM] > PT_ARM_HIGH_US) && !armed && arm_gate_ok) {
            arm_gate_ok = false;
            if (g_ch[IB_THR] < PT_THR_MIN_GATE_US) {
                armed = true;
                trace_printf("pt: ARMED. outputs now follow the sticks.\n");
            } else {
                trace_printf("pt: ARM REFUSED, throttle %u not at minimum (<%u). "
                             "Lower throttle, switch OFF, then ON again.\n",
                             (uint32_t)g_ch[IB_THR], (uint32_t)PT_THR_MIN_GATE_US);
            }
        }

        if (armed) {
            if (g_ch[IB_THR] < PT_THR_MIN_GATE_US) {
                if (thr_low_count < 255) {
                    thr_low_count++;
                }
                if (thr_low_count >= PT_THR_LOW_DEBOUNCE_TICKS) {
                    // Throttle idle: hold every motor at idle, no mixing --
                    // otherwise a stick alone could raise a motor above
                    // idle with the throttle closed.
                    for (uint8_t m = 0; m < PT_NUM_MOTORS; m++) {
                        motor_us[m] = PT_IDLE_US;
                        pt_set(pt_motor[m].out, PT_IDLE_US);
                    }
                }
                // else: fewer than PT_THR_LOW_DEBOUNCE_TICKS consecutive
                // low readings -- treat as a single bad frame, not a real
                // throttle-down. motor_us[] is left as-is and re-asserted
                // unconditionally below, same as every other tick.
            } else {
                thr_low_count = 0;
                const int32_t thr_off = (int32_t)g_ch[IB_THR] - (int32_t)PT_MIN_US;
                const int32_t r = ((int32_t)g_ch[IB_ROLL]  - 1500) * PT_ROLL_SIGN;
                const int32_t p = ((int32_t)g_ch[IB_PITCH] - 1500) * PT_PITCH_SIGN;
                const int32_t y = ((int32_t)g_ch[IB_YAW]   - 1500) * PT_YAW_SIGN;

                for (uint8_t m = 0; m < PT_NUM_MOTORS; m++) {
                    int32_t mix = ((r * pt_motor[m].roll_f) +
                                   (p * pt_motor[m].pitch_f) +
                                   (y * pt_motor[m].yaw_f)) / 1000;
                    mix = (mix * PT_MIX_GAIN_PCT) / 100;
                    int32_t target = (int32_t)PT_MIN_US + thr_off + mix;
                    if (target < (int32_t)PT_MIN_US) {
                        target = (int32_t)PT_MIN_US;
                    }
                    if (target > (int32_t)PT_MAX_US) {
                        target = (int32_t)PT_MAX_US;
                    }

                    // Climb-only slew limit -- see PT_RAMP_US_PER_SEC above.
                    // A drop in target (stick pulled back) is applied
                    // instantly; only a rise is capped per tick. With
                    // PT_RAMP_ENABLED 0 the rise is instant too.
                    int32_t us = (int32_t)motor_us[m];
                    if (PT_RAMP_ENABLED && (target > us)) {
                        const int32_t remaining = target - us;
                        us += (ramp_max_step < remaining) ? ramp_max_step : remaining;
                    } else {
                        us = target;
                    }
                    motor_us[m] = (uint16_t)us;
                    pt_set(pt_motor[m].out, (uint16_t)us);
                }
            }
            // out4/out5 are held at idle unconditionally below.
        } else {
            for (uint8_t m = 0; m < PT_NUM_MOTORS; m++) {
                motor_us[m] = PT_IDLE_US;
            }
        }
    }

    // Frame-timeout failsafe. Catches the receiver being unplugged or
    // dying, NOT transmitter-off (this receiver keeps streaming held
    // values) -- transmitter-side failsafe must be configured separately.
    if (armed && (now_ms - last_good_frame_ms >= PT_FAILSAFE_MS)) {
        armed = false;
        arm_gate_ok = false;
        pt_all_idle();
        trace_printf("pt: FAILSAFE, no valid frame for %ums -> DISARMED, idle\n",
                     (uint32_t)PT_FAILSAFE_MS);
    }

    // Unconditionally re-assert motor_us[] to hardware every call, not just
    // when new_input() was true above: with no receiver connected chans=0 and
    // the block above never runs, so without this the outputs would never
    // actually be idle-held at all. Must stay last in the function, after
    // every branch that can change motor_us[].
    //
    // This is no longer how the write conflict with the vehicle's own output
    // path is won -- PT_EXCLUSIVE_MASK does that in the HAL, and re-asserting
    // every tick cannot (Q-34: the losing writer only has to touch the shadow
    // register at the wrong moment within a 20ms PWM period, not last within
    // the loop iteration; see RCOutput.h set_exclusive_mask()).
    for (uint8_t m = 0; m < PT_NUM_MOTORS; m++) {
        pt_set(pt_motor[m].out, motor_us[m]);
    }
    // ch4/ch5 have no function this milestone (the pusher motor is out of
    // scope) but SRV_Channels::push() writes them every tick too, so they are
    // held explicitly at idle rather than left to whatever wrote last.
    pt_set(4, PT_IDLE_US);
    pt_set(5, PT_IDLE_US);

    // Change-triggered logging plus a slow heartbeat -- the RemoteProc trace
    // buffer is 16 KiB and does not wrap (trace.c stops accepting once full).
    const uint32_t since = now_ms - last_report_ms;
    constexpr int32_t move_us = 20;
    const int32_t d_thr   = (int32_t)g_ch[IB_THR]   - last_thr;
    const int32_t d_roll  = (int32_t)g_ch[IB_ROLL]  - last_roll;
    const int32_t d_pitch = (int32_t)g_ch[IB_PITCH] - last_pitch;
    const int32_t d_yaw   = (int32_t)g_ch[IB_YAW]   - last_yaw;
    const bool moved = (d_thr   >  move_us) || (d_thr   < -move_us) ||
                       (d_roll  >  move_us) || (d_roll  < -move_us) ||
                       (d_pitch >  move_us) || (d_pitch < -move_us) ||
                       (d_yaw   >  move_us) || (d_yaw   < -move_us);
    const bool changed = (armed != last_armed) || moved;

    if (since < 500) {
        return;                        // rate limit while sticks move
    }
    if (!changed && (since < 5000)) {
        return;                        // idle: heartbeat only
    }
    last_report_ms = now_ms;
    last_armed  = armed;
    last_thr    = (int32_t)g_ch[IB_THR];
    last_roll   = (int32_t)g_ch[IB_ROLL];
    last_pitch  = (int32_t)g_ch[IB_PITCH];
    last_yaw    = (int32_t)g_ch[IB_YAW];

    // cmpa_shadow is EPWM0's CMPA *shadow* register, read back from hardware.
    // Scale, at the 3.125 MHz TBCLK: 1000us -> 3125, 1500us -> 4687,
    // 2000us -> 6250, tbprd 62500 throughout.
    //
    // Read this as "our last write landed", NOT as "the pin is doing this".
    // CMPA is in shadow mode (load at CTR=ZERO) and a read of the CMPA address
    // returns the shadow, so this value cannot disagree with what this function
    // just wrote a few lines above -- which is exactly why a whole session of
    // "registers always read correct" never caught Q-34's competing writer.
    // blocked= is the useful number now: RCOutput drops of foreign writes,
    // reported in HAL_ChibiOS_K3::run()'s 5s alive line.
    trace_printf("pt: %s thr=%u | m1=%u m2=%u m3=%u m4=%u | r=%u p=%u y=%u | "
                 "cmpa_shadow=%u chans=%u\n",
                 armed ? "ARMED " : (arm_gate_ok ? "disarm/rdy" : "disarm/cyc"),
                 (uint32_t)g_ch[IB_THR],
                 (uint32_t)motor_us[0], (uint32_t)motor_us[1],
                 (uint32_t)motor_us[2], (uint32_t)motor_us[3],
                 (uint32_t)g_ch[IB_ROLL], (uint32_t)g_ch[IB_PITCH], (uint32_t)g_ch[IB_YAW],
                 (uint32_t)ehrpwm_read_cmp(AM67_EPWM0_BASE, false),
                 (uint32_t)AP::RC().num_channels());
}

}  // namespace ChibiOS_K3

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
