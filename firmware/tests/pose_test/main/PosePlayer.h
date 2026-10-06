#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "PoseTable.h"

namespace tilt::pose {

struct Segment {
    JointArray target{};
    std::uint32_t duration_ms = 0;
    const char* name = nullptr;
};

class PosePlayer {
public:
    static constexpr std::size_t kQueueCapacity = 4;

    void reset(const JointArray& current) {
        clear();
        last_command_ = current;
        segment_start_ = current;
    }

    bool enqueue(const Segment& segment) {
        const std::size_t total = queued_count_ + (active_ ? 1u : 0u);
        if (segment.duration_ms == 0 || total >= kQueueCapacity) {
            return false;
        }
        const std::size_t tail = (queue_head_ + queued_count_) %
                                 kQueueCapacity;
        queue_[tail] = segment;
        ++queued_count_;
        return true;
    }

    void clear() {
        queue_head_ = 0;
        queued_count_ = 0;
        active_ = false;
        elapsed_ms_ = 0;
        active_segment_ = {};
    }

    JointArray step(std::uint32_t dt_ms) {
        if (!active_ && !startNext()) {
            return last_command_;
        }

        std::uint32_t remaining_dt = dt_ms;
        while (active_) {
            const std::uint32_t remaining_segment =
                active_segment_.duration_ms - elapsed_ms_;
            const std::uint32_t advance =
                std::min(remaining_dt, remaining_segment);
            elapsed_ms_ += advance;
            remaining_dt -= advance;

            const float s = static_cast<float>(elapsed_ms_) /
                            static_cast<float>(active_segment_.duration_ms);
            const float smooth = s * s * (3.0f - 2.0f * s);
            for (int joint = 0; joint < NUM_JOINTS; ++joint) {
                last_command_[joint] = segment_start_[joint] +
                    (active_segment_.target[joint] - segment_start_[joint]) *
                        smooth;
            }

            if (elapsed_ms_ < active_segment_.duration_ms) {
                break;
            }

            last_command_ = active_segment_.target;
            active_ = false;
            elapsed_ms_ = 0;
            if (!startNext() || remaining_dt == 0) {
                break;
            }
        }
        return last_command_;
    }

    bool idle() const {
        return !active_ && queued_count_ == 0;
    }

    const char* activeName() const {
        return active_ ? active_segment_.name : nullptr;
    }

    const JointArray& lastCommand() const {
        return last_command_;
    }

private:
    bool startNext() {
        if (queued_count_ == 0) {
            return false;
        }
        segment_start_ = last_command_;
        active_segment_ = queue_[queue_head_];
        queue_head_ = (queue_head_ + 1) % kQueueCapacity;
        --queued_count_;
        elapsed_ms_ = 0;
        active_ = true;
        return true;
    }

    std::array<Segment, kQueueCapacity> queue_{};
    std::size_t queue_head_ = 0;
    std::size_t queued_count_ = 0;
    bool active_ = false;
    Segment active_segment_{};
    std::uint32_t elapsed_ms_ = 0;
    JointArray segment_start_{};
    JointArray last_command_{};
};

enum class Posture { UNKNOWN, HOME, STAND, LIFT_L, LIFT_R };
enum class PoseCommand { HOME, STAND, LIFT_L, LIFT_R };

struct Sequence {
    std::array<Segment, PosePlayer::kQueueCapacity> segments{};
    std::size_t count = 0;
    Posture final_posture = Posture::UNKNOWN;
    const char* error = nullptr;
};

inline bool appendSegment(Sequence& sequence,
                          const JointArray& target,
                          std::uint32_t duration_ms,
                          const char* name) {
    if (sequence.count >= sequence.segments.size() || duration_ms == 0) {
        sequence.error = "segment queue capacity exceeded";
        return false;
    }
    sequence.segments[sequence.count++] = {target, duration_ms, name};
    return true;
}

inline bool appendStandReturn(Posture current, Sequence& sequence) {
    if (current == Posture::LIFT_L) {
        return appendSegment(sequence, shiftPose(Side::RIGHT),
                             kLiftDurationMs, "SHIFT_R") &&
               appendSegment(sequence, standPose(),
                             kShiftDurationMs, "STAND");
    }
    if (current == Posture::LIFT_R) {
        return appendSegment(sequence, shiftPose(Side::LEFT),
                             kLiftDurationMs, "SHIFT_L") &&
               appendSegment(sequence, standPose(),
                             kShiftDurationMs, "STAND");
    }
    return true;
}

inline bool appendLift(Side support, Sequence& sequence) {
    const bool right_support = support == Side::RIGHT;
    return appendSegment(sequence,
                         shiftPose(support),
                         kShiftDurationMs,
                         right_support ? "SHIFT_R" : "SHIFT_L") &&
           appendSegment(sequence,
                         liftPose(support),
                         kLiftDurationMs,
                         right_support ? "LIFT_L" : "LIFT_R");
}

inline bool makeSequence(Posture current,
                         PoseCommand command,
                         Sequence& sequence) {
    sequence = {};

    if (command == PoseCommand::HOME) {
        if (!appendStandReturn(current, sequence) ||
            !appendSegment(sequence, homePose(), kHomeDurationMs, "HOME")) {
            return false;
        }
        sequence.final_posture = Posture::HOME;
        return true;
    }

    if (command == PoseCommand::STAND) {
        if (current == Posture::LIFT_L || current == Posture::LIFT_R) {
            if (!appendStandReturn(current, sequence)) {
                return false;
            }
        } else if (!appendSegment(sequence,
                                  standPose(),
                                  kStandDurationMs,
                                  "STAND")) {
            return false;
        }
        sequence.final_posture = Posture::STAND;
        return true;
    }

    if (current == Posture::HOME || current == Posture::UNKNOWN) {
        sequence.error = "press STAND(s) before lifting a foot";
        return false;
    }

    const bool lift_left = command == PoseCommand::LIFT_L;
    if ((lift_left && current == Posture::LIFT_L) ||
        (!lift_left && current == Posture::LIFT_R)) {
        sequence.error = "requested foot is already lifted";
        return false;
    }

    if ((lift_left && current == Posture::LIFT_R) ||
        (!lift_left && current == Posture::LIFT_L)) {
        if (!appendStandReturn(current, sequence)) {
            return false;
        }
    }

    if (!appendLift(lift_left ? Side::RIGHT : Side::LEFT, sequence)) {
        return false;
    }
    sequence.final_posture = lift_left ? Posture::LIFT_L : Posture::LIFT_R;
    return true;
}

}  // namespace tilt::pose
