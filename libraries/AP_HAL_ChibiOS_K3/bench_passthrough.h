#pragma once

namespace ChibiOS_K3 {
    // Bench RC->PWM passthrough for the four quad-X outputs. Called every
    // main-loop tick from HAL_ChibiOS_K3::run(), after rcinDriver.update()
    // so fresh iBus data is available. See bench_passthrough.cpp for scope
    // and safety notes.
    void bench_passthrough_update();
}
