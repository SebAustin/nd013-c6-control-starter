/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * A discrete PID controller with a bounded control command.
 *
 * Usage per control cycle: UpdateDeltaTime(), then UpdateError(), then
 * TotalError() to read the control command.
 */
class PID {
public:
    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID: set the gains and the output limits, and clear all
    * accumulated state. Note the order: the upper limit comes first.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given the current error
    * (the speed error or the heading error; named cte by the starter).
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error: the control command, held inside
    * [output_lim_min, output_lim_max].
    */
    double TotalError() const;

    /*
    * Update the delta time (the time step, in seconds) and return it.
    */
    double UpdateDeltaTime(double new_delta_time);

private:
    /*
    * Sum of the three weighted terms, before the output limits are applied.
    */
    double UnboundedCommand(double integral_error) const;

    /*
    * True when the delta time is finite and positive, so it can be used to
    * integrate and differentiate.
    */
    bool HasUsableTimeStep() const;

    /*
    * Anti-windup: the integral to keep, given the integral this update
    * would produce if it accumulated the whole step.
    */
    double LimitedIntegral(double candidate_integral) const;

    /*
    * Errors
    */
    double proportional_error_ = 0.0;
    double integral_error_ = 0.0;
    double derivative_error_ = 0.0;
    // False until the first error arrives, so the first update has nothing
    // to differentiate against and produces no derivative kick.
    bool has_previous_error_ = false;

    /*
    * Coefficients
    */
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;

    /*
    * Output limits
    */
    double output_lim_max_ = 0.0;
    double output_lim_min_ = 0.0;

    /*
    * Delta time
    */
    double delta_time_ = 0.0;
};

#endif //PID_CONTROLLER_H
