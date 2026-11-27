/**
 * Error computations for the two PID controllers.
 *
 * These are pure functions kept apart from main.cpp so they can be unit
 * tested without CARLA (see tests/control_errors_test.cpp). Both errors use
 * the convention "desired minus actual", so that positive gains push the
 * control command in the direction that reduces the error.
 */

#ifndef CONTROL_ERRORS_H
#define CONTROL_ERRORS_H

#include <cmath>

namespace control_errors {

// Below this distance (in metres) the car is treated as standing on the
// target point, where the direction towards it is undefined.
const double kMinTargetDistance = 0.05;

/**
 * Wraps an angle in radians into [-pi, pi].
 */
inline double WrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

/**
 * Speed error: desired speed minus actual speed.
 * Positive when the car is too slow (so the controller accelerates),
 * negative when it is too fast (so the controller brakes).
 */
inline double SpeedError(double desired_speed, double actual_speed) {
    return desired_speed - actual_speed;
}

/**
 * Heading error: desired heading minus yaw, wrapped into [-pi, pi].
 *
 * The desired heading is the direction from the car's position to the target
 * point. Wrapping matters because headings are angles: without it, a car at
 * yaw +179 degrees aiming at -179 degrees would see an error of almost a full
 * turn instead of 2 degrees, and would steer the long way round.
 *
 * Returns 0 when the car is already on the target point, because there is no
 * direction to steer towards.
 */
inline double HeadingError(double x_position, double y_position, double yaw,
                           double target_x, double target_y) {
    const double dx = target_x - x_position;
    const double dy = target_y - y_position;
    if (std::hypot(dx, dy) < kMinTargetDistance) {
        return 0.0;
    }

    const double desired_heading = std::atan2(dy, dx);
    return WrapAngle(desired_heading - yaw);
}

}  // namespace control_errors

#endif  // CONTROL_ERRORS_H
