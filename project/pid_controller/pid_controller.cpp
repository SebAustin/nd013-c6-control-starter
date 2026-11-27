/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"

#include <algorithm>
#include <cmath>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
    kp_ = Kpi;
    ki_ = Kii;
    kd_ = Kdi;
    // The upper limit is the first of the two arguments, which is easy to get
    // backwards, so accept the pair in either order.
    output_lim_max_ = std::max(output_lim_maxi, output_lim_mini);
    output_lim_min_ = std::min(output_lim_maxi, output_lim_mini);

    // Start from a clean state so a controller can be re-initialised between
    // tuning runs without carrying over the previous run's integral.
    proportional_error_ = 0.0;
    integral_error_ = 0.0;
    derivative_error_ = 0.0;
    has_previous_error_ = false;
    delta_time_ = 0.0;
}

void PID::UpdateError(double cte) {
    // A NaN or infinite error would stick in the integral and derivative and
    // pin the command to a limit for the rest of the run. Ignore it and keep
    // the last valid state, so the command holds its previous value.
    if (!std::isfinite(cte)) {
        return;
    }

    const double previous_error = proportional_error_;
    const bool had_previous_error = has_previous_error_;

    proportional_error_ = cte;
    has_previous_error_ = true;

    // main.cpp measures time in whole seconds, so the time step is often
    // exactly zero. Without a usable time step the integral cannot advance
    // and the rate of change is undefined, so keep the integral as it is and
    // reuse the last derivative instead of dividing by zero.
    if (!HasUsableTimeStep()) {
        return;
    }

    // Discrete derivative: change in error over the time step. The very
    // first update has no earlier error, so its derivative is zero.
    derivative_error_ = had_previous_error ? (cte - previous_error) / delta_time_ : 0.0;

    // Discrete integral (rectangle rule), limited by anti-windup.
    integral_error_ = LimitedIntegral(integral_error_ + cte * delta_time_);
}

double PID::TotalError() const {
    const double command = UnboundedCommand(integral_error_);
    return std::max(output_lim_min_, std::min(command, output_lim_max_));
}

double PID::UpdateDeltaTime(double new_delta_time) {
    delta_time_ = new_delta_time;
    return delta_time_;
}

double PID::UnboundedCommand(double integral_error) const {
    return kp_ * proportional_error_ + ki_ * integral_error + kd_ * derivative_error_;
}

bool PID::HasUsableTimeStep() const {
    // Written as "finite and positive" so that NaN is rejected too.
    return std::isfinite(delta_time_) && delta_time_ > 0.0;
}

double PID::LimitedIntegral(double candidate_integral) const {
    // Anti-windup. Compare the command with and without this step's
    // accumulation. The step is only restricted when it pushes the command
    // further past a limit; a step that moves the command back towards the
    // allowed range is always taken, whatever the signs of the gains.
    const double current_command = UnboundedCommand(integral_error_);
    const double candidate_command = UnboundedCommand(candidate_integral);

    const bool pushes_past_max =
        candidate_command > output_lim_max_ && candidate_command > current_command;
    const bool pushes_past_min =
        candidate_command < output_lim_min_ && candidate_command < current_command;
    if (!pushes_past_max && !pushes_past_min) {
        return candidate_integral;
    }

    const double limit = pushes_past_max ? output_lim_max_ : output_lim_min_;
    const bool already_saturated =
        pushes_past_max ? current_command >= limit : current_command <= limit;
    if (already_saturated) {
        return integral_error_;
    }

    // The step crosses the limit: take the fraction of it that brings the
    // command exactly onto the limit. The command is linear in the integral,
    // so a linear interpolation is exact. The denominator is non-zero here
    // because the candidate lies strictly beyond the current command.
    const double fraction = (limit - current_command) / (candidate_command - current_command);
    return integral_error_ + fraction * (candidate_integral - integral_error_);
}
