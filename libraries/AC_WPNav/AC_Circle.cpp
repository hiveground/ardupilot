/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include "AC_Circle.h"
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Circle::var_info[] PROGMEM = {
    // @Param: RADIUS
    // @DisplayName: Circle Radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: cm
    // @Range: 0 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("RADIUS",  0,  AC_Circle, _radius, AC_CIRCLE_RADIUS_DEFAULT),

    // @Param: RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE",    1, AC_Circle, _rate,    AC_CIRCLE_RATE_DEFAULT),

    //K-hack
    // @Param: DIR_ANGLE
    // @DisplayName: Direction angle
    // @Description: Ellipse mode's rotation angle of the ellipse
    // @Units: deg
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DIR_ANGLE",    2, AC_Circle, _dir_angle,    AC_CIRCLE_DIR_ANGLE_DEFAULT),

    //K-hack
    // @Param: EX_RADIUS
    // @DisplayName: Extend Radius
    // @Description: Ellipse mode's extending radius from the original circle, minimum value equals to original radius
    // @Units: cm
    // @Range: 0 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("EX_RADIUS",    3, AC_Circle, _ex_radius,    AC_CIRCLE_EX_RADIUS_DEFAULT),

    //K-hack
    // @Param: ZO_HEIGHT
    // @DisplayName: Zone Height
    // @Description: Height limit the split the low and high zone
    // @Units: cm
    // @Range: 0 100000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("ZO_HEIGHT",    4, AC_Circle, _zone_height,    AC_CIRCLE_ZO_HEIGHT_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Circle::AC_Circle(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _last_update(0),
    _yaw(0.0f),
    _angle(0.0f),
    _angle_total(0.0f),
    _angular_vel(0.0f),
    _angular_vel_max(0.0f),
    _angular_accel(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.panorama = false;
}

/// init - initialise circle controller setting center specifically
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init(const Vector3f& center)
{
    _center = center;

    set_rate(0);


    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // calculate velocities
    calc_velocities(true);

    // set start angle from position
    init_start_angle(false);
}

/// init - initialise circle controller setting center using stopping point and projecting out based on the copter's heading
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init()
{
    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // get stopping point
    const Vector3f& stopping_point = _pos_control.get_pos_target();

    // set circle center to circle_radius ahead of stopping point
    _center.x = stopping_point.x + (_radius) * _ahrs.cos_yaw();
    _center.y = stopping_point.y + (_radius) * _ahrs.sin_yaw();
    _center.z = stopping_point.z;

    // calculate velocities
    calc_velocities(true);

    // set starting angle from vehicle heading
    init_start_angle(true);
}

/// set_circle_rate - set circle rate in degrees per second
void AC_Circle::set_rate(float deg_per_sec)
{
    if (!is_equal(deg_per_sec,_rate)) {
        _rate = deg_per_sec;
        calc_velocities(false);
    }
}

//K-hack
//change radius : increase/decrase at linear rate
void AC_Circle::change_radius(float change_value)
{
    if(change_value<=0 && _radius <= 0) {
        _radius = 0;
        calc_velocities(false);
    } else {
        _radius += change_value/fabsf(change_value);
        calc_velocities(false);
    }

}

/// update - update circle controller
void AC_Circle::update()
{
    // calculate dt
    float dt = _pos_control.time_since_last_xy_update();

    // update circle position at poscontrol update rate
    if (dt >= _pos_control.get_dt_xy()) {

        // double check dt is reasonable
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // ramp angular velocity to maximum
        if (_angular_vel < _angular_vel_max) {
            _angular_vel += fabsf(_angular_accel) * dt;
            _angular_vel = min(_angular_vel, _angular_vel_max);
        }
        if (_angular_vel > _angular_vel_max) {
            _angular_vel -= fabsf(_angular_accel) * dt;
            _angular_vel = max(_angular_vel, _angular_vel_max);
        }

        // update the target angle and total angle traveled
        float angle_change = _angular_vel * dt;
        _angle += angle_change;
        _angle = wrap_PI(_angle);
        _angle_total += angle_change;

        //K-hack
        float dir_angle = _dir_angle *PI/180.0f;
        float init_angle;

        // if the circle_radius is zero we are doing panorama so no need to update loiter target
        if (!is_zero(_radius)) {
            // calculate target position
            Vector3f target;
            
            //K-hack 
            // insert parametric formula for path rotation + extender radius
            const Vector3f &curr_pos = _inav.get_position();

            float ex_radius;
            if(_ex_radius<_radius || curr_pos.z<_zone_height){ex_radius=_radius;} else {ex_radius = _ex_radius;}

            init_angle = -wrap_PI(_angle - ToRad(_dir_angle));

            target.x = _center.x + (((_radius + ex_radius)/2) * cosf(init_angle)*cosf(dir_angle)) + ((_radius)*sinf(init_angle)*sinf(dir_angle)) + ((ex_radius-_radius)/2*cosf(dir_angle));
            target.y = _center.y - ((_radius) * sinf(init_angle)*cosf(dir_angle)) + (((_radius + ex_radius)/2)*cosf(init_angle)*sinf(dir_angle)) + ((ex_radius-_radius)/2*sinf(dir_angle));

            target.z = _pos_control.get_alt_target();

            // update position controller target
            _pos_control.set_xy_target(target.x, target.y);

            // heading is 180 deg from vehicles target position around circle
            //_yaw = wrap_PI(_angle-PI) * AC_CIRCLE_DEGX100;
            
            //K-hack
            //center-heading using trigonometric approach.
            _yaw = (atan2f(curr_pos.y-_center.y,curr_pos.x-_center.x)-PI) * AC_CIRCLE_DEGX100;
 
        }else{
            // set target position to center
            Vector3f target;
            target.x = _center.x;
            target.y = _center.y;
            target.z = _pos_control.get_alt_target();

            // update position controller target
            _pos_control.set_xy_target(target.x, target.y);

            // heading is same as _angle but converted to centi-degrees
            _yaw = _angle * AC_CIRCLE_DEGX100;
        }

        // update position controller
        _pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_ONLY, 1.0f);
    }
}

// get_closest_point_on_circle - returns closest point on the circle
//  circle's center should already have been set
//  closest point on the circle will be placed in result
//  result's altitude (i.e. z) will be set to the circle_center's altitude
//  if vehicle is at the center of the circle, the edge directly behind vehicle will be returned
void AC_Circle::get_closest_point_on_circle(Vector3f &result)
{
    // return center if radius is zero
    if (_radius <= 0) {
        result = _center;
        return;
    }

    // get current position
    const Vector3f &curr_pos = _inav.get_position();

    // calc vector from current location to circle center
    Vector2f vec;   // vector from circle center to current location
    vec.x = (curr_pos.x - _center.x);
    vec.y = (curr_pos.y - _center.y);
    float dist = pythagorous2(vec.x, vec.y);

    // if current location is exactly at the center of the circle return edge directly behind vehicle
    if (is_zero(dist)) {

        //K-hack
        //
        float dir_angle = (_dir_angle *PI/180.0f);

        float ex_radius;
        if(_ex_radius<_radius || curr_pos.z<_zone_height){ex_radius=_radius;} else {ex_radius = _ex_radius;}

        //result.x = _center.x - ((_radius) * _ahrs.cos_yaw());
        result.x = _center.x - (((_radius + ex_radius)/2) * _ahrs.cos_yaw()*cosf(dir_angle)) + ((_radius)*_ahrs.sin_yaw()*sinf(dir_angle)) + ((ex_radius-_radius)/2*cosf(dir_angle));

        //result.y = _center.y - ((_radius) * _ahrs.sin_yaw());
        result.y = _center.y - ((_radius) * _ahrs.sin_yaw()*cosf(dir_angle)) + (((_radius + ex_radius)/2)*_ahrs.cos_yaw()*sinf(dir_angle)) + ((ex_radius-_radius)/2*sinf(dir_angle));

        result.z = _center.z;
        return;
    }

    // calculate closest point on edge of circle
    result.x = _center.x + vec.x / dist * (_radius);
    result.y = _center.y + vec.y / dist * (_radius);
    result.z = _center.z;
}

// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void AC_Circle::calc_velocities(bool init_velocity)
{
    // if we are doing a panorama set the circle_angle to the current heading
    if (_radius <= 0) {
        _angular_vel_max = ToRad(_rate);
        _angular_accel = max(fabsf(_angular_vel_max),ToRad(AC_CIRCLE_ANGULAR_ACCEL_MIN));  // reach maximum yaw velocity in 1 second
    }else{
        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        float velocity_max = min(_pos_control.get_speed_xy(), safe_sqrt(0.5f*_pos_control.get_accel_xy()*_radius));

        // angular_velocity in radians per second
        _angular_vel_max = velocity_max/_radius;
        _angular_vel_max = constrain_float(ToRad(_rate),-_angular_vel_max,_angular_vel_max);

        // angular_acceleration in radians per second square
        _angular_accel = max(_pos_control.get_accel_xy()/_radius, ToRad(AC_CIRCLE_ANGULAR_ACCEL_MIN));
    }

    // initialise angular velocity
    if (init_velocity) {
        _angular_vel = 0;
    }
}

// init_start_angle - sets the starting angle around the circle and initialises the angle_total
//  if use_heading is true the vehicle's heading will be used to init the angle causing minimum yaw movement
//  if use_heading is false the vehicle's position from the center will be used to initialise the angle
void AC_Circle::init_start_angle(bool use_heading)
{
    // initialise angle total
    _angle_total = 0;

    // if the radius is zero we are doing panorama so init angle to the current heading
    if (_radius <= 0) {
        _angle = _ahrs.yaw;
        return;
    }

    // if use_heading is true
    if (use_heading) {
        _angle = wrap_PI(_ahrs.yaw-PI);
    } else {
        // if we are exactly at the center of the circle, init angle to directly behind vehicle (so vehicle will backup but not change heading)
        const Vector3f &curr_pos = _inav.get_position();
        if (is_equal(curr_pos.x,_center.x) && is_equal(curr_pos.y,_center.y)) {
            _angle = wrap_PI(_ahrs.yaw-PI);
        } else {
            // get bearing from circle center to vehicle in radians
            float bearing_rad = atan2f(curr_pos.y-_center.y,curr_pos.x-_center.x);
            _angle = wrap_PI(bearing_rad);
        }
    }
}

/*void AC_Circle::circ_reposition(float des_angle)
{
    float repos_rate = (des_angle-_angle>0) ? 1.0f : -1.0f;
    set_rate(repos_rate);
    update();
}*/