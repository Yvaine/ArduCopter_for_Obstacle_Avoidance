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

	//���Ӵ����pitch���в���
	//��һ�ַ�������sonar_altС��3mʱ��g.rc_2.control_in���в���
	//�ֽ׶�ֻ�ǽ��������������ķ����Ϊ��ǰ�����Գ������������ɼ��õ���������Ȼ��sonar_alt
	if (sonar_alt < SONAR_SAFE){
		if (g.rc_2.control_in > 0){
			//ң����������������˻�ǰ���������˻��Զ����ˣ�������3m֮��,�����������˻���10�ȵķ�ʽ���ˣ�ֱ��3������
			my_rc_2_control_in = -1000;
		}
		else{
			//��ǰ�����ϰ�������£�����ң�����������˻�����
			my_rc_2_control_in = g.rc_2.control_in;
		}
	}
	else
	{
		//�����˻���3�����⣬��ô���˻�����ͨ�ȶ�ģʽһģһ������ȫ��ң��������
		my_rc_2_control_in = g.rc_2.control_in;
	}

	//To-Do:�ڶ��ַ����Ǵ���sonar=300��ʱ���possition���������˻����������possitionλ��,��ʱûд���漰��λ�ÿ���




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
