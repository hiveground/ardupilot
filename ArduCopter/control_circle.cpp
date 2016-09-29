/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_circle.pde - init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool Copter::circle_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete and current control mode has manual throttle control,
    // as this will force the helicopter to descend.
    if (!ignore_checks && mode_has_manual_throttle(control_mode) && !motors.rotor_runup_complete()){
        return false;
    }
#endif

    if (position_ok() || ignore_checks) {
        circle_pilot_yaw_override = false;

        // initialize speeds and accelerations
        pos_control.set_speed_xy(wp_nav.get_speed_xy());
        pos_control.set_accel_xy(wp_nav.get_wp_acceleration());
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise circle controller including setting the circle center based on vehicle speed
        circle_nav.init();

        return true;
    }else{
        return false;
    }
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void Copter::circle_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    //K-hack
    float target_orbit_rate = 0;
    float target_pitch_rate =0;

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
        // To-Do: add some initialisation of position controllers
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        pos_control.set_alt_target_to_current_alt();
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in*0.05);
        if (!is_zero(target_yaw_rate)) {
            circle_pilot_yaw_override = true;
        }

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }

        //K-hack
        //20160929 - Change form 0.004 -> 0.001
        //         - Added RC6 switch for auto/manual orbit
        if(read_3pos_switch(g.rc_6.radio_in) >= AUX_SWITCH_MIDDLE) {

            target_orbit_rate = (-((float)(channel_roll->control_in))*0.001f);
         
            if(!is_zero(target_orbit_rate) && (fabs(target_orbit_rate) > 3)) {

                circle_nav.set_rate(target_orbit_rate);

                //reset heading to center if no yaw applied
                if(is_zero(target_yaw_rate)) {
                    circle_pilot_yaw_override = false;
                }
            } else {
                circle_nav.set_rate(0);
            }
        } else {
            circle_nav.set_original_rate();
        }

        //20160929 - Change form 0.001 -> 0.0005
        target_pitch_rate = (((float)(channel_pitch->control_in))*0.0005f);
        if(!is_zero(target_pitch_rate)) {
            circle_nav.change_radius(target_pitch_rate);
        }
    } else {
        circle_nav.set_rate(0);
    }

    // run circle controller
    circle_nav.update();

    // call attitude controller
    if (circle_pilot_yaw_override) {
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), target_yaw_rate);
    }else{
        attitude_control.angle_ef_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw(),true);
    }

    // run altitude controller
    if (sonar_enabled && (sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        // if sonar is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
    }
    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    pos_control.update_z_controller();
}
