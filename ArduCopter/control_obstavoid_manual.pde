/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */
#ifndef SONAR_SAFE
#define SONAR_SAFE 300
#endif

// stabilize_init - initialise stabilize controller
static bool abstavoid_manual_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);

    // stabilize should never be made to fail
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
static void abstavoid_manual_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;
	int16_t my_rc_2_control_in;

	//增加代码对pitch进行操作
	//第一种方法是在sonar_alt小于3m时对g.rc_2.control_in进行操作
	//现阶段只是将超声波传感器的方向改为向前，所以超声波传感器采集得到的数据依然是sonar_alt
	if (sonar_alt < SONAR_SAFE){
		if (g.rc_2.control_in > 0){
			//遥控器不允许控制无人机前进，且无人机自动后退，调整到3m之外,方法是让无人机以10度的方式后退，直道3米以外
			my_rc_2_control_in = -1000;
		}
		else{
			//在前方有障碍的情况下，允许遥控器控制无人机后退
			my_rc_2_control_in = g.rc_2.control_in;
		}
	}
	else
	{
		//若无人机在3米以外，那么无人机与普通稳定模式一模一样，完全受遥控器控制
		my_rc_2_control_in = g.rc_2.control_in;
	}

	//To-Do:第二种方法是存下sonar=300的时候的possition，并将无人机设置至这个possition位置,暂时没写，涉及到位置控制




    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
	get_pilot_desired_lean_angles(g.rc_1.control_in, my_rc_2_control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);
}
