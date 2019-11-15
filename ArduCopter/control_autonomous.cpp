#include <iostream>
#include "Copter.h"

using namespace std;

/*
 * Init and run calls for autonomous flight mode (largely based off of the AltHold flight mode)
 */

// autonomous_init - initialise autonomous controller
bool Copter::autonomous_init(bool ignore_checks)
{
    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    // reset integrators for roll and pitch controllers
    g.pid_roll.reset_I();
    g.pid_pitch.reset_I();

    return true;
}

// autonomous_run - runs the autonomous controller
// should be called at 100hz or more
void Copter::autonomous_run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // desired roll, pitch, and yaw_rate
    float target_roll = 0.0f, target_pitch = 0.0f, target_yaw_rate = 0.0f;

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif
    target_climb_rate = 0.0f;

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming=true;
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming=false;
        }
#endif
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming=false;
            }
        }
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#endif
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        // compute the target climb rate, roll, pitch and yaw rate
        // land if autonomous_controller returns false
        if (!autonomous_controller(target_climb_rate, target_roll, target_pitch, target_yaw_rate)) {
            // switch to land mode
            set_mode(LAND, MODE_REASON_MISSION_END);
            break;
        }

        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}

#define AUTON_VERSION_NUM 0.2;

// autonomous_controller - computes target climb rate, roll, pitch, and yaw rate for autonomous flight mode
// returns true to continue flying, and returns false to land
bool Copter::autonomous_controller(float &target_climb_rate, float &target_roll, float &target_pitch, float &target_yaw_rate)
{

    // PARAM 1: How close to ge to the front wall
    // PARAM 2: How lose to get to walls when holding left and right
    // PARAM 3: How long to hold each state

    enum States
    {
        HOLD_CENTER,
        HOLD_RIGHT,
        HOLD_LEFT,
        MOVING_FORWARD
    };

    static States state = HOLD_CENTER;

    static int stateCounter = 0;

    static float moveForwardRightHold = 0;
    static float moveForwardLeftHold = 0;
    
    // get downward facing sensor reading in meters
    float rangefinder_alt = (float)rangefinder_state.alt_cm / 100.0f;

    // get horizontal sensor readings in meters
    float dist_forward, dist_right, dist_backward, dist_left;
    g2.proximity.get_horizontal_distance(0, dist_forward);
    g2.proximity.get_horizontal_distance(90, dist_right);
    g2.proximity.get_horizontal_distance(180, dist_backward);
    g2.proximity.get_horizontal_distance(270, dist_left);

    // set desired climb rate in centimeters per second
    target_climb_rate = 0.0f;

    // set desired roll and pitch in centi-degrees
    // Old target_pitch: 
	//target_pitch = 0.0f;
	g.pid_pitch.set_input_filter_all (g.e100_param1 - dist_forward);
	target_pitch = 100.0f * g.pid_pitch.get_pid();

    // If we can move forward and we take the oppurtunity and try to hold horizontal pos
    if(abs(g.e100_param1 - dist_forward) > 20 && state!=MOVING_FORWARD)
    {
        state = MOVING_FORWARD
        moveForwardRightHold = dist_right;
        moveForwardLeftHold = dist_left;
    }
    // When we get out of moving forward, center again
    else if(g.e100_param1 - dist_forward) < 20 && state == MOVING_FORWARD)
        state = HOLD_CENTER;

    // State machine
    switch(state)
    {
        case HOLD_CENTER:
            g.pid_roll.set_input_filter_all( dist_right-dist_left );
            if(stateCounter > g.e100_param3 * 400)
            {
                state = HOLD_RIGHT;
                stateCounter = 0;
            }
            stateCounter++;
            break;
        case HOLD_RIGHT:
            g.pid_roll.set_input_filer_all(dist_right - g.e200_param2);
            if(stateCounter > g.e100_param3 * 400)
            {
                state = HOLD_LEFT;
                stateCounter = 0;
            }
            stateCounter++;
            break;
        case HOLD_LEFT:
            g.pid_roll.set_input_filer_all(g.e200_param2 - dist_left);
            if(stateCounter > g.e100_param3 * 400)
            {
                state = HOLD_CENTER;
                stateCounter = 0;
            }
            stateCounter++;
            break;
        case MOVING_FORWARD:
            g.pid_roll.set_input_filter_all((dist_right - moveForwardRightHold) + 
                                            (moveForwardLeftHold - dist_left))
            break;
    }
	
    target_roll =  100.0f * g.pid_roll.get_pid();


    // set desired yaw rate in centi-degrees per second (set to zero to hold constant heading)
    target_yaw_rate = 0.0f;

	// send logging messages to Mission Planner once every half-second because 400 is one second
	static int counter = 0;
	if (counter++ > 200) {
        char string[80] = "Autonomous Flight Version: AUTON_VERSION_NUM - Intelligent Flight: \n";
        switch(state){
            case HOLD_CENTER:
                strcat(string, "HOLD_CENTER")
                break;
            case HOLD_RIGHT:
                strcat(string, "HOLD_RIGHT")
                break;
            case HOLD_LEFT:
                strcat(string, "HOLD_LEFT")
                break;
            case MOVING_FORWARD:
                str(string, "MOVING_FORWARD")
                break;
        }
		gcs_send_text(MAV_SEVERITY_INFO, string);
		counter = 0;
	}
    return true;
}
