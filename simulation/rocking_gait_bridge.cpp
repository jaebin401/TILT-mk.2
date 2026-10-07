#include <cstdint>
#include <new>

#include "tilt/gait/RockingGait.h"

extern "C" {

struct TiltRockingOutput {
    float position_rad[6];
    std::uint8_t phase;
    std::uint8_t support_side;
    std::uint64_t completed_events;
    std::uint8_t running;
};

void* tilt_rocking_create(int preset) {
    const auto selected = preset == 1
        ? tilt::gait::Preset::kVisualization
        : tilt::gait::Preset::kConservative;
    return new (std::nothrow)
        tilt::gait::RockingGait(tilt::gait::parametersForPreset(selected));
}

void tilt_rocking_destroy(void* handle) {
    delete static_cast<tilt::gait::RockingGait*>(handle);
}

void tilt_rocking_start(void* handle, int first_support) {
    if (handle == nullptr) return;
    static_cast<tilt::gait::RockingGait*>(handle)->start(
        first_support == 1 ? tilt::gait::Side::kRight
                           : tilt::gait::Side::kLeft);
}

void tilt_rocking_stop(void* handle) {
    if (handle == nullptr) return;
    static_cast<tilt::gait::RockingGait*>(handle)->stop();
}

std::uint32_t tilt_rocking_cycle_duration_ms(void* handle) {
    if (handle == nullptr) return 0;
    return static_cast<tilt::gait::RockingGait*>(handle)->cycleDurationMs();
}

int tilt_rocking_update(void* handle,
                        std::uint32_t dt_ms,
                        TiltRockingOutput* output) {
    if (handle == nullptr || output == nullptr) return 0;
    const auto result =
        static_cast<tilt::gait::RockingGait*>(handle)->update(dt_ms);
    for (int joint = 0; joint < 6; ++joint) {
        output->position_rad[joint] = result.position_rad[joint];
    }
    output->phase = static_cast<std::uint8_t>(result.phase);
    output->support_side = static_cast<std::uint8_t>(result.support_side);
    output->completed_events = result.completed_events;
    output->running = result.running ? 1 : 0;
    return 1;
}

}  // extern "C"
