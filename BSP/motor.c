#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "motor.h"
#include "usbd_cdc_if.h"
#include "wit_c_sdk.h"
#include "hal_hard_i2c.h"
#include <math.h>



Vision_Data_t vision_data;
Gimbal_Ctrl_t gimbal_ctrl;

PID_t SteadyState_PID_Yaw;
PID_t SteadyState_PID_Pitch;


	float fGyro_ang[3] = {0, 0, 0};

// ===== Internal helper function ======

/**
************************************************************************
* @brief:      	Motor_Init: Motor Initialization Function
* @param:      	void
* @retval:     	void
* @details:    	Initialized CAN,the CAN filter,
*               Set default parameters and control system
************************************************************************
**/
void Motor_Init()
{
	bsp_can_init();
	dm_motor_init();
}

/**
************************************************************************
* @brief:      	Motor_Enable: Motor Enable
* @param:      	void
* @retval:     	void
* @details:    	Enable Motor 1 and Motor 2 in MIT mode             
************************************************************************
**/

void Motor_Enable()
{
	dm_motor_enable(&hcan1, &motor[Motor1]);
	HAL_Delay(1000);
	dm_motor_enable(&hcan2, &motor[Motor2]);
	HAL_Delay(1000);
}


/**
************************************************************************
* @brief:      	Motor_Enable: Motor Enable
* @param:      	void
* @retval:     	void
* @details:    	Enable Motor 1 and Motor 2 in MIT mode             
************************************************************************
**/

void Motor_Disable()
{
	dm_motor_disable(&hcan1, &motor[Motor1]);
	HAL_Delay(1000);
	dm_motor_disable(&hcan2, &motor[Motor2]);
	HAL_Delay(1000);
}


/**
************************************************************************
* @brief:      	Motor_One_Ctrl: Control the motor1
* @param:      	pos: Position set value
* @param:      	vel: Speed setpoint
* @param:      	kp: Position Ratio Factor
* @param:      	kd: Position differential coefficient
* @param:      	tor: Torque command value
* @retval:     	void
* @details:    	Given control parameters for motor 1, such as position and speed
************************************************************************
**/

void Motor_One_Ctrl(float pos, float vel,float kp, float kd, float tor) ///The big motor
{
	mit_ctrl(&hcan1, &motor[Motor1], motor[Motor1].id, pos, vel, kp, kd, tor);
}

/**
************************************************************************
* @brief:      	Motor_Two_Ctrl: Control the motor2
* @param:      	pos: Position set value
* @param:      	vel: Speed setpoint
* @param:      	kp: Position Ratio Factor
* @param:      	kd: Position differential coefficient
* @param:      	tor: Torque command value
* @retval:     	void
* @details:    	Given control parameters for motor 2, such as position and speed
************************************************************************
**/

void Motor_Two_Ctrl(float pos, float vel,float kp, float kd, float tor)
{
	mit_ctrl(&hcan2, &motor[Motor2], motor[Motor2].id, pos, vel, kp, kd, tor);
}




/**
************************************************************************
* @brief:      	Motor_One_Spd_Ctrl: Control the motor1 using speed_ctrl
* @param:      	vel: Speed setpoint
* @retval:     	void
* @details:    	Given control parameters for motor 1, such speed
************************************************************************
**/

void Motor_One_Spd_Ctrl(float vel)
{
	spd_ctrl(&hcan1, motor[Motor1].id,vel);
}

/**
************************************************************************
* @brief:      	Motor_Two_Spd_Ctrl: Control the motor2 using speed_ctrl
* @param:      	vel: Speed setpoint
* @retval:     	void
* @details:    	Given control parameters for motor 2, such speed
************************************************************************
**/

void Motor_Two_Spd_Ctrl(float vel)
{
	spd_ctrl(&hcan2, motor[Motor2].id,vel);
}



/**
************************************************************************
* @brief:      	PID_Init: PID Initial Value Setting
* @param:      	Kp, Ki, Kd, Output limiting, integral limiting
* @retval:     	void
* @details:    	Set Kp, Ki, Kd, output limit, and integral limit              
************************************************************************
**/
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_iout) {
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
    pid->max_out = max_out; pid->max_iout = max_iout;
    pid->err = 0; pid->last_err = 0; pid->i_err = 0; pid->out = 0;
}


/**
************************************************************************
* @brief:      	PID_Calc: PID Calculation
* @param:      	PID_t *pid:  Motor PID structure:PID_t
* @param:      	float error: The difference between the target pixel and the disparity-corrected pixel
* @retval:     	Gimbal Angle Correction Value
* @details:    	PID Calculation (Positional PID, Output Angle Increment)static         
************************************************************************
**/
float PID_Calc(PID_t *pid, float error) {
	
    pid->err = error;
    
    float p_term = pid->Kp * pid->err;
    
    pid->i_err += pid->err;
    /// Integral Clamping
    if (pid->i_err > pid->max_iout) 
			pid->i_err = pid->max_iout;
    if (pid->i_err < -pid->max_iout)
			pid->i_err = -pid->max_iout;
    float i_term = pid->Ki * pid->i_err;
    
    float d_term = pid->Kd * (pid->err - pid->last_err);
    
    pid->out = p_term + i_term + d_term;
    pid->last_err = pid->err;
    
    // Output Limiting
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    if (pid->out < -pid->max_out) pid->out = -pid->max_out;
    
    return pid->out;
}

/**
************************************************************************
* @brief:      	PID_Calc_Ada: PID Calculation
* @param:      	PID_t *pid:  Motor PID structure:PID_t
* @param:      	float error: The difference between the target pixel and the disparity-corrected pixel
* @retval:     	Gimbal Angle Correction Value
* @details:    	PID Calculation (Positional PID, Output Angle Increment)static         
************************************************************************
**/

float PID_Calc_Ada(PID_t *pid, float error, float distance) {
    
    pid->err = error;
    
    float p_term = pid->Kp * pid->err;
    
		float INT_SEPARATION_THRESH = 60.0f * PIXEL_TO_RAD_K;
	
    if (fabs(pid->err) < INT_SEPARATION_THRESH) {
        pid->i_err += pid->err;
    }
    
    if (pid->i_err > pid->max_iout) 
        pid->i_err = pid->max_iout;
    if (pid->i_err < -pid->max_iout) 
        pid->i_err = -pid->max_iout;
        
    float i_term = pid->Ki * pid->i_err;
		
		if (i_term > pid->max_iout) {
        i_term = pid->max_iout;
        if (pid->Ki != 0.0f) {
            pid->i_err = pid->max_iout / pid->Ki;
        }
    } else if (i_term < -pid->max_iout) {
        i_term = -pid->max_iout;
        if (pid->Ki != 0.0f) {
            pid->i_err = -pid->max_iout / pid->Ki;
        }
    }
    
    float d_term = pid->Kd * (pid->err - pid->last_err);
    
    float raw_output = p_term + i_term + d_term;
    
    float adaptive_gain = 1.0f;
    
    if (distance <= DIST_MIN) {
        adaptive_gain = ADAPT_GAIN_MAX;
    } 
    else if (distance >= DIST_MAX) {
        adaptive_gain = ADAPT_GAIN_MIN;
    } 
    else {
        float ratio = (distance - DIST_MIN) / (DIST_MAX - DIST_MIN);
        
        adaptive_gain = ADAPT_GAIN_MAX - ratio * (ADAPT_GAIN_MAX - ADAPT_GAIN_MIN);
    }
    
    pid->out = raw_output * adaptive_gain;
    
    pid->last_err = pid->err;
    
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    if (pid->out < -pid->max_out) pid->out = -pid->max_out;
    
    return pid->out;
}



// Parallax compensation lookup/formula (requires actual measurement!)
// Return value: Pixel compensation amount on the Y-axis
static float Get_Parallax_Offset(float dist) {
// Example: The closer the distance, the more the Y-axis center should move down (assuming the laser is below the camera)
// Here we use a linear formula to simulate it, but in practice it's best to measure a few points and fit them
// Assuming an offset of 30 pixels at 50cm, and 10 pixels at 100cm
    if (dist <= 0) return 0;
    float z;
		z = (dist - 1075.32) / 336.00;
		float offset = 0.83127110*z*z*z - 1.80107887*z*z + 4.53415416*z - 7.04480269;
// y = kx + b simple simulation
// This is a sample formula, please make sure to modify it according to actual conditions!!!
    return offset;
}



// ================= core function =================


/**
************************************************************************
* @brief:      	Gimbal_Init: Gimbal Initialization
* @param:      	void
* @retval:     	void
* @details:    	Initialize and enable the motor, set the PID data, 
								set the initial zero point, and start aiming mode
************************************************************************
**/
void Gimbal_Init(void) {
    Motor_Init();
		HAL_Delay(500);

    PID_Init(&gimbal_ctrl.pid_yaw, 0.0f, 0.0f, 0.0f, 10.0f, 2.0f);	
	
    PID_Init(&gimbal_ctrl.pid_pitch, 0.00f, 0.0f, 0.00f, 10.0f, 1.0f);    //0.004f, 0.0f, 0.000f!!!!!!!
    
		
		PID_Init(&SteadyState_PID_Yaw, 0.8f, 0.0f, 0.0f, 5.0f, 2.0f);     //0.8!!!!!!!
		//PID_Init(&SteadyState_PID_Pitch, 0.5, 0.0, 0.0, 5.0, 2.0);
	
    gimbal_ctrl.yaw_target_angle = 0; // Assume facing forward when powered on
    gimbal_ctrl.pitch_target_angle = 0;
    gimbal_ctrl.mode = 0;             // Default aiming mode
}


/**
************************************************************************
* @brief:      	Gimbal_Vision_Rx_Handler: Visual raw data processing
* @param:      	uint8_t *rx_buf: Visual data packet
* @retval:     	void
* @details:    	Call this function in the UART receive interrupt callback,
************************************************************************
**/
/*void Gimbal_Vision_Rx_Handler(uint8_t *rx_buf) {
    // Simple validation
    if (rx_buf[0] != 0x5A || rx_buf[1] != 0x5A) return;
    
// Type casting parsing (corresponds to Python's struct.pack)
// Order in buffer: 5A 5A [X_L X_H] [Y_L Y_H] [D_L D_H] CS
    int16_t x_raw = (int16_t)(rx_buf[2] | (rx_buf[3] << 8));
    int16_t y_raw = (int16_t)(rx_buf[4] | (rx_buf[5] << 8));
    int16_t d_raw = (int16_t)(rx_buf[6] | (rx_buf[7] << 8));
    uint8_t checksum = rx_buf[8];
    
    // Checksum and verification (sum of the first 8 bytes)
    uint8_t calc_sum = 0;
    for(int i=0; i<8; i++) calc_sum += rx_buf[i];
    
    if (calc_sum == checksum) {
        vision_data.cur_x = (float)x_raw;
        vision_data.cur_y = (float)y_raw;
        vision_data.dist_cm = (float)d_raw / 10.0f; // The passed value is dist*10
        vision_data.last_update_tick = HAL_GetTick();
        vision_data.is_tracking = 1;
    }
}
*/

/**
************************************************************************
* @brief:      	Gimbal_Set_Zero: Gimbal Zero Position Setting
* @param:      	void
* @retval:     	void
* @details:    	Gimbal Zero Position Setting
************************************************************************
**/

void Gimbal_Set_Zero(void)
{
    save_pos_zero(&hcan1, motor[Motor1].id, MIT_MODE);
    HAL_Delay(10);
    save_pos_zero(&hcan2, motor[Motor2].id, MIT_MODE);
}

/**
************************************************************************
* @brief:      	Gimbal_Control_Loop: Motor Task using spd_mode
* @param:      	void
* @retval:     	void
* @details:    	Placed in a 1ms timer interrupt (1kHz)
************************************************************************
**/



/*#define MAX_COAST_TIME_MS  50 


void Gimbal_Control_Loop2(void)
{
		uint32_t time_since_update = HAL_GetTick() - vision_data.last_update_tick;
	
	    if (fabs(vision_data.cur_x - Interpolated_SetPoint) > 0.5f){
				Interpolated_SetPoint += Pos_Increment;
				if (Pos_Increment > 0 && Interpolated_SetPoint > vision_data.cur_x) {
            Interpolated_SetPoint = vision_data.cur_x;
        }
				else if(Pos_Increment < 0 && Interpolated_SetPoint < vision_data.cur_x){
					Interpolated_SetPoint = vision_data.cur_x;
				}
			} else if(time_since_update < MAX_COAST_TIME_MS){
				float coast_step = (vision_data.vel_x_pps / 1000.0f); 
        if (vision_data.is_tracking) {
            Interpolated_SetPoint += coast_step;
        }
			}
			else{
				Interpolated_SetPoint = vision_data.cur_x;
        //vision_data.vel_x_pps = 0.0f;
			}
	    if (vision_data.is_tracking && !vision_data.last_is_tracking) {
        gimbal_ctrl.pid_yaw.i_err = 0;
        gimbal_ctrl.pid_yaw.last_err = 0;
        gimbal_ctrl.pid_pitch.i_err = 0;
        gimbal_ctrl.pid_pitch.last_err = 0;
				Interpolated_SetPoint = vision_data.cur_x;
				Pos_Increment = 0.0f; 
		}
    vision_data.last_is_tracking = vision_data.is_tracking;
		
		
		float aim_x = IMG_CENTER_X; 
    float aim_y = IMG_CENTER_Y + Get_Parallax_Offset(vision_data.dist_cm);
		
    float pix_err_x = aim_x - Interpolated_SetPoint;
    float pix_err_y = aim_y - vision_data.cur_y;

		float rad_err_x = pix_err_x * PIXEL_TO_RAD_K;
    float rad_err_y = pix_err_y * PIXEL_TO_RAD_K;

    float delta_yaw_spd = PID_Calc(&gimbal_ctrl.pid_yaw, rad_err_x) * YAW_DIR_COEFF;

    //float delta_yaw = PID_Calc(&gimbal_ctrl.pid_yaw, rad_err_x) * YAW_DIR_COEFF;
    float delta_pitch_spd = PID_Calc(&gimbal_ctrl.pid_pitch, rad_err_y) * PITCH_DIR_COEFF;
		
		float Kff_yaw = 8.0;
		float vel_yaw_ff = vision_data.vel_x_pps * PIXEL_TO_RAD_K  * Kff_yaw * -1;
		 if (vision_data.is_tracking && gimbal_ctrl.mode == 2) {
				gimbal_ctrl.yaw_target_speed = delta_yaw_spd + vel_yaw_ff;
				gimbal_ctrl.pitch_target_speed = delta_pitch_spd;
				
//				if (gimbal_ctrl.pitch_target_speed > 5.0f) {
//						gimbal_ctrl.pitch_target_speed = 5.0f;
//				} else if (gimbal_ctrl.pitch_target_speed < -5.0f) {
//						gimbal_ctrl.pitch_target_speed = -5.0f;
//				}
    }
		 else{
			  gimbal_ctrl.yaw_target_speed = 0.0f;
				Motor_One_Spd_Ctrl(0.0f);
		 }
		Motor_One_Spd_Ctrl(gimbal_ctrl.yaw_target_speed);
		//Motor_Two_Spd_Ctrl(delta_pitch_spd);
}
*/


/*float Interpolated_SetPoint = 0.0f;
float Correction_Increment = 0.0f;
float w_feedforward = 0;

uint32_t Vision_Frame_Tick = 0;

uint8_t is_target_locked = 0; // 0: Unlocked, 1: Locked

		
float last_feedforward = 0.0f;


void Gimbal_Control_Loop2(void)
{
    if (vision_data.is_tracking && !vision_data.last_is_tracking) {
        gimbal_ctrl.pid_yaw.i_err = 0;
        gimbal_ctrl.pid_yaw.last_err = 0;
        
        Interpolated_SetPoint = vision_data.cur_x; 
        Correction_Increment = 0.0f;
    }
    vision_data.last_is_tracking = vision_data.is_tracking;

    float final_speed_cmd = 0.0f;
		
		float max_delta = 1.0f;
		w_feedforward = -1 * fGyro[2]/180.0f * 3.1416f;
		
		if (w_feedforward - last_feedforward > max_delta)
			last_feedforward += max_delta;
		else if (w_feedforward - last_feedforward < -max_delta)
			last_feedforward -= max_delta;
		else
			last_feedforward = w_feedforward;
		
		//float final_speed_cmd_corner = 0.0f;
		float pitch_cmd_final = gimbal_ctrl.pitch_target_angle;
    if (vision_data.is_tracking && gimbal_ctrl.mode == 2) {
        
        float base_velocity_step = vision_data.vel_x_pps / 1000.0f;
        Interpolated_SetPoint += base_velocity_step + Correction_Increment;

        float aim_x = IMG_CENTER_X; 
				float aim_y = IMG_CENTER_Y + Get_Parallax_Offset(vision_data.dist_cm);
			
        float pix_err_x = aim_x - Interpolated_SetPoint;
        float rad_err_x = pix_err_x * PIXEL_TO_RAD_K;
			
				float pix_err_y = aim_y - vision_data.cur_y;
				float rad_err_y = pix_err_y * PIXEL_TO_RAD_K;
			
        float pid_correction_spd = PID_Calc(&gimbal_ctrl.pid_yaw, rad_err_x) * YAW_DIR_COEFF;
				float delta_pitch = PID_Calc(&gimbal_ctrl.pid_pitch, rad_err_y) * PITCH_DIR_COEFF;
			
//        float Kff_yaw = 22.0;
//        float feedforward_spd = vision_data.vel_x_pps * PIXEL_TO_RAD_K * Kff_yaw * -1.0f;
        final_speed_cmd = pid_correction_spd;
				//final_speed_cmd_corner = feedforward_spd;
				
				gimbal_ctrl.pitch_target_angle += delta_pitch;
				pitch_cmd_final = gimbal_ctrl.pitch_target_angle;
			
				if (gimbal_ctrl.pitch_target_angle > 1.05f) {
						gimbal_ctrl.pitch_target_angle = 1.05f;
				} else if (gimbal_ctrl.pitch_target_angle < -1.05f) {
						gimbal_ctrl.pitch_target_angle = -1.05f;
				}
				pitch_cmd_final = gimbal_ctrl.pitch_target_angle;
				
				if (vision_data.is_tracking && is_target_locked == 0) {
				_Bool is_pos_stable = (fabs(pix_err_x) < LOCK_ON_ERROR_THRESHOLD_PIXELS) && 
                          (fabs(pix_err_y) < LOCK_ON_ERROR_THRESHOLD_PIXELS);
				_Bool is_speed_stable = (fabs(vision_data.vel_x_pps) < LOCK_ON_SPEED_THRESHOLD_PPS) &&
                            (fabs(vision_data.vel_y_pps) < LOCK_ON_SPEED_THRESHOLD_PPS);
					if (is_pos_stable && is_speed_stable) {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
						is_target_locked = 1; 
					}
			  }
				
    }
		gimbal_ctrl.yaw_target_speed = final_speed_cmd; 
    Motor_One_Spd_Ctrl(gimbal_ctrl.yaw_target_speed + last_feedforward);
		Motor_Two_Ctrl(pitch_cmd_final, 0.0f, 4.0f, 0.03f, 0.0f);
}
*/
	

void Gimbal_Control_Steady(float gimbal_target_angle_Yaw, float gimbal_target_angle_Pitch, float yaw_target_vel)
{
		
		
		float errorY = gimbal_target_angle_Yaw - fGyro_ang[2];
		while (errorY > 180.0f) errorY -= 360.0f;
		while (errorY < -180.0f) errorY += 360.0f;
	
		float errorP = gimbal_target_angle_Pitch - fGyro_ang[0];
		while (errorP > 180.0f) errorP -= 360.0f;
		while (errorP < -180.0f) errorP += 360.0f;

		float target_speed_Yaw = PID_Calc(&SteadyState_PID_Yaw, errorY);
		float target_speed_Pitch = PID_Calc(&SteadyState_PID_Pitch, errorP);
	
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		Motor_One_Spd_Ctrl(target_speed_Yaw + yaw_target_vel);
		//Motor_Two_Spd_Ctrl(target_speed_Pitch);
//	
//		Motor_One_Spd_Ctrl(0);
//		Motor_Two_Spd_Ctrl(0);
}



float Interpolated_SetPoint = 0.0f;
float Correction_Increment = 0.0f;
float w_feedforward = 0;

uint32_t Vision_Frame_Tick = 0;

uint8_t is_target_locked = 0; // 0: Unlocked, 1: Locked

		
float last_feedforward = 0.0f;


void Gimbal_Control_Loop2(void)
{
    if (vision_data.is_tracking && !vision_data.last_is_tracking) {
        gimbal_ctrl.pid_yaw.i_err = 0;
        gimbal_ctrl.pid_yaw.last_err = 0;
        
        Interpolated_SetPoint = vision_data.cur_x; 
        Correction_Increment = 0.0f;
    }
    vision_data.last_is_tracking = vision_data.is_tracking;

    float final_angle_cmd = 0.0f;
		
		
		float pitch_cmd_final = gimbal_ctrl.pitch_target_angle;
    if (vision_data.is_tracking && gimbal_ctrl.mode == 2) {
        
        float base_velocity_step = vision_data.vel_x_pps / 1000.0f;
        Interpolated_SetPoint += base_velocity_step + Correction_Increment;

        float aim_x = IMG_CENTER_X; 
				float aim_y = IMG_CENTER_Y + Get_Parallax_Offset(vision_data.dist_cm);
			
        float pix_err_x = aim_x - Interpolated_SetPoint;
        float rad_err_x = pix_err_x * PIXEL_TO_RAD_K;
			
				float pix_err_y = aim_y - vision_data.cur_y;
				float rad_err_y = pix_err_y * PIXEL_TO_RAD_K;
				
        float pid_correction_spd = PID_Calc(&gimbal_ctrl.pid_yaw, rad_err_x) * YAW_DIR_COEFF;
				float delta_pitch = PID_Calc(&gimbal_ctrl.pid_pitch, rad_err_y) * PITCH_DIR_COEFF;
			
        //final_angle_cmd = pid_correction_spd;
				
				gimbal_ctrl.pitch_target_angle += delta_pitch;
				pitch_cmd_final = gimbal_ctrl.pitch_target_angle;
			
				if (gimbal_ctrl.pitch_target_angle > 1.05f) {
						gimbal_ctrl.pitch_target_angle = 1.05f;
				} else if (gimbal_ctrl.pitch_target_angle < -1.05f) {
						gimbal_ctrl.pitch_target_angle = -1.05f;
				}
				pitch_cmd_final = gimbal_ctrl.pitch_target_angle;
				
				if (vision_data.is_tracking && is_target_locked == 0) {
				_Bool is_pos_stable = (fabs(pix_err_x) < LOCK_ON_ERROR_THRESHOLD_PIXELS) && 
                          (fabs(pix_err_y) < LOCK_ON_ERROR_THRESHOLD_PIXELS);
				_Bool is_speed_stable = (fabs(vision_data.vel_x_pps) < LOCK_ON_SPEED_THRESHOLD_PPS) &&
                            (fabs(vision_data.vel_y_pps) < LOCK_ON_SPEED_THRESHOLD_PPS);
					if (is_pos_stable && is_speed_stable) {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
						is_target_locked = 1; 
					}
			  }
				
    
			//gimbal_ctrl.yaw_target_angle = Gimbal_Control_Steady(pid_correction_spd);
			Motor_One_Spd_Ctrl(gimbal_ctrl.yaw_target_angle);
		
			Motor_Two_Ctrl(pitch_cmd_final, 0.0f, 4.0f, 0.03f, 0.0f);//4.0 0.03
		}
}


/**
************************************************************************
* @brief:      	Gimbal_Control_Loop: Motor Task using MIT_mode
* @param:      	void
* @retval:     	void
* @details:    	Placed in a 1ms timer interrupt (1kHz)
************************************************************************
**/
void Gimbal_Control_Loop(void) {
	
		WitReadReg(Roll, 3);
    fGyro_ang[0] = sReg[Roll] / 32768.0f * 180.0f;
    fGyro_ang[1] = sReg[Pitch] / 32768.0f * 180.0f;
		fGyro_ang[2] = sReg[Yaw] / 32768.0f * 180.0f;

	  if (gimbal_ctrl.mode != 3 || vision_data.is_tracking == 0) {
        gimbal_ctrl.circle_t = 0.0f;
    }
		static float smoothed_cos_gamma = 1.0f;
    if (vision_data.is_tracking && !vision_data.last_is_tracking) {
        gimbal_ctrl.pid_yaw.i_err = 0;
        gimbal_ctrl.pid_yaw.last_err = 0;
        gimbal_ctrl.pid_pitch.i_err = 0;
        gimbal_ctrl.pid_pitch.last_err = 0;
			
				gimbal_ctrl.last_yaw_target_angle = gimbal_ctrl.yaw_target_angle;
        gimbal_ctrl.last_pitch_target_angle = gimbal_ctrl.pitch_target_angle;
			
			  smoothed_cos_gamma = fabsf(cosf(fGyro_ang[2] * PI / 180.0f));
        if (smoothed_cos_gamma < 0.2f) smoothed_cos_gamma = 0.2f;
        if (smoothed_cos_gamma > 1.0f) smoothed_cos_gamma = 1.0f;
			
    }
    vision_data.last_is_tracking = vision_data.is_tracking;

    float yaw_offset_rad = 0.0f;
    float pitch_offset_rad = 0.0f;
    float yaw_circle_vel_rads = 0.0f;
    float pitch_circle_vel_rads = 0.0f;
		
		uint8_t mode3_laser_state = 0;
		
    if (gimbal_ctrl.mode == 3 && vision_data.dist_cm > 0) {
			
				float DRAW_CYCLE_TIME = 1.0f / CIRCLE_SPEED_HZ;
				float RAMP_TIME = 2.0f;
				float TOTAL_MODE3_TIME = RAMP_TIME + DRAW_CYCLE_TIME;
				
				float START_PHASE = -PI / 2.0f;
				if (gimbal_ctrl.circle_t <= TOTAL_MODE3_TIME) {
            gimbal_ctrl.circle_t += 0.001f; // 1ms
        }
        
        float alpha_rad_pitch = atan2f(CIRCLE_RADIUS_CM * 10.0f, vision_data.dist_cm);
        
				float cos_gamma = cosf(fGyro_ang[2] * PI / 180.0f); 
        cos_gamma = fabsf(cos_gamma);
        if (cos_gamma < 0.2f) cos_gamma = 0.2f;
        if (cos_gamma > 1.0f) cos_gamma = 1.0f;
			
				smoothed_cos_gamma = 0.005f * cos_gamma + 0.995f * smoothed_cos_gamma;
			
				float alpha_rad_yaw = alpha_rad_pitch * smoothed_cos_gamma;
        float omega = 2.0f * PI * CIRCLE_SPEED_HZ;
			
				
				if (gimbal_ctrl.circle_t < RAMP_TIME) {
            float r_scale = gimbal_ctrl.circle_t / RAMP_TIME; // 0 -> 1
            
            yaw_offset_rad   = alpha_rad_yaw * r_scale * cosf(START_PHASE);
            pitch_offset_rad = alpha_rad_pitch * r_scale * sinf(START_PHASE);
            
            yaw_circle_vel_rads   = (alpha_rad_yaw * cosf(START_PHASE)) / RAMP_TIME;
            pitch_circle_vel_rads = (alpha_rad_pitch * sinf(START_PHASE)) / RAMP_TIME;
            
            mode3_laser_state = 0;
        }else if (gimbal_ctrl.circle_t < TOTAL_MODE3_TIME) {
            float t_draw = gimbal_ctrl.circle_t - RAMP_TIME;
            float phase = omega * t_draw + START_PHASE;
            
            yaw_offset_rad   = alpha_rad_yaw * cosf(phase);
            pitch_offset_rad = alpha_rad_pitch * sinf(phase);
            
            yaw_circle_vel_rads   = -alpha_rad_yaw * omega * sinf(phase);
            pitch_circle_vel_rads =  alpha_rad_pitch * omega * cosf(phase);
            
            mode3_laser_state = 1;
        } 
				
				else {
            float phase = omega * DRAW_CYCLE_TIME + START_PHASE;
            
            yaw_offset_rad   = alpha_rad_yaw * cosf(phase);
            pitch_offset_rad = alpha_rad_pitch * sinf(phase);
            
            yaw_circle_vel_rads   = 0.0f;
            pitch_circle_vel_rads = 0.0f; 
            
            mode3_laser_state = 0;
        }
        float r_scale = 1.0f;
        float ramp_time = 2.0f;
        if (gimbal_ctrl.circle_t < ramp_time) {
            r_scale = gimbal_ctrl.circle_t / ramp_time;
        }
        
    }

    float aim_x = IMG_CENTER_X; 
    float aim_y = IMG_CENTER_Y + Get_Parallax_Offset(vision_data.dist_cm);
    
    float pix_err_x = aim_x - vision_data.cur_x;
    float pix_err_y = aim_y - vision_data.cur_y;
    
    float rad_err_x = pix_err_x * PIXEL_TO_RAD_K;
    float rad_err_y = pix_err_y * PIXEL_TO_RAD_K;
    
    if (gimbal_ctrl.mode == 3) {
        rad_err_x += yaw_offset_rad * YAW_DIR_COEFF;
        rad_err_y += pitch_offset_rad * PITCH_DIR_COEFF;
    }

    float delta_yaw = PID_Calc_Ada(&gimbal_ctrl.pid_yaw, rad_err_x, vision_data.dist_cm) * YAW_DIR_COEFF;
    float delta_pitch = PID_Calc_Ada(&gimbal_ctrl.pid_pitch, rad_err_y, vision_data.dist_cm) * PITCH_DIR_COEFF;

    float yaw_target_vel = 0.0f;
    float pitch_target_vel = 0.0f;

    if ((vision_data.is_tracking && gimbal_ctrl.mode == 2 ) || gimbal_ctrl.mode == 3) {
        
        gimbal_ctrl.yaw_target_angle += delta_yaw;
        gimbal_ctrl.pitch_target_angle += delta_pitch;
        
        if (gimbal_ctrl.pitch_target_angle > 1.05f) {
            gimbal_ctrl.pitch_target_angle = 1.05f;
        } else if (gimbal_ctrl.pitch_target_angle < -1.05f) {
            gimbal_ctrl.pitch_target_angle = -1.05f;
        }

        yaw_target_vel = (gimbal_ctrl.yaw_target_angle - gimbal_ctrl.last_yaw_target_angle) * 1000.0f;  
        gimbal_ctrl.last_yaw_target_angle = gimbal_ctrl.yaw_target_angle;
        
        if (vision_data.is_tracking && is_target_locked == 0) {
            _Bool is_pos_stable = (fabs(pix_err_x) < LOCK_ON_ERROR_THRESHOLD_PIXELS) && 
                                  (fabs(pix_err_y) < LOCK_ON_ERROR_THRESHOLD_PIXELS);
            _Bool is_speed_stable = (fabs(vision_data.vel_x_pps) < LOCK_ON_SPEED_THRESHOLD_PPS) &&
                                    (fabs(vision_data.vel_y_pps) < LOCK_ON_SPEED_THRESHOLD_PPS);
            if (is_pos_stable && is_speed_stable) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
                is_target_locked = 1; 
            }
        }
    }

    float yaw_cmd_final = gimbal_ctrl.yaw_target_angle * 180.0f / PI;
    float pitch_cmd_final = gimbal_ctrl.pitch_target_angle;

    if (gimbal_ctrl.mode == 3) {
        yaw_cmd_final = (gimbal_ctrl.yaw_target_angle + yaw_offset_rad) * 180.0f / PI;
        pitch_cmd_final = gimbal_ctrl.pitch_target_angle + pitch_offset_rad;
        
        yaw_target_vel += yaw_circle_vel_rads; 
        pitch_target_vel += pitch_circle_vel_rads; 
        
        if (mode3_laser_state == 1) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // ???
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // ???
        }
    }

    Gimbal_Control_Steady(yaw_cmd_final, pitch_cmd_final, yaw_target_vel);                          
    
    Motor_Two_Ctrl(pitch_cmd_final, pitch_target_vel, 11.0f, 0.06f, 0.0f);  //13.0 0.06      11.7 0.06    
}

/*				float yaw_target_vel = 0.0;

void Gimbal_Control_Loop(void) {
    if (vision_data.is_tracking && !vision_data.last_is_tracking) {
        gimbal_ctrl.pid_yaw.i_err = 0;
        gimbal_ctrl.pid_yaw.last_err = 0;
        gimbal_ctrl.pid_pitch.i_err = 0;
        gimbal_ctrl.pid_pitch.last_err = 0;
		}
    vision_data.last_is_tracking = vision_data.is_tracking;
		
    float yaw_cmd_final = gimbal_ctrl.yaw_target_angle * 180.0f / 3.14156f;
    float pitch_cmd_final = gimbal_ctrl.pitch_target_angle;

		    // --- Step A: Calculate the set point (including parallax compensation) ---
        float aim_x = IMG_CENTER_X; 
        float aim_y = IMG_CENTER_Y + Get_Parallax_Offset(vision_data.dist_cm);
			
        // --- Step B: Calculate Pixel Error ---
        float pix_err_x = aim_x - vision_data.cur_x;
        float pix_err_y = aim_y - vision_data.cur_y;
        
        // --- Step C: K value conversion (pixels -> radians) ---
        float rad_err_x = pix_err_x * PIXEL_TO_RAD_K;
        float rad_err_y = pix_err_y * PIXEL_TO_RAD_K;
        
        // --- Step D: PID Calculation (Output Increment) ---
				// Note Direction Correction YAW_DIR_COEFF
        float delta_yaw = PID_Calc_Ada(&gimbal_ctrl.pid_yaw, rad_err_x, vision_data.dist_cm) * YAW_DIR_COEFF;
        float delta_pitch = PID_Calc(&gimbal_ctrl.pid_pitch, rad_err_y) * PITCH_DIR_COEFF;
				
				//float yaw_target_vel = 0.0;
				
			  float rad_vel_x = vision_data.vel_x_pps * PIXEL_TO_RAD_K;
        float rad_vel_y = vision_data.vel_y_pps * PIXEL_TO_RAD_K;
        
//				float Kff_yaw = 0.0f;       //8.0    12.0
//        float Kff_pitch = 0.0f;


//			  float delta_yaw_ff = rad_vel_x * 0.001f * Kff_yaw * -1;
//        //float delta_pitch_ff = rad_vel_y * 0.001f * PITCH_DIR_COEFF * Kff_pitch * -1;
//					
//				float yaw_rate = rad_vel_x *  Kff_yaw * -1;
					
				if ((vision_data.is_tracking && gimbal_ctrl.mode == 2 )|| gimbal_ctrl.mode == 3) {
        // --- Step E: Integral Position (Update Reference Target) ---
        gimbal_ctrl.yaw_target_angle += delta_yaw;// + delta_yaw_ff;
        gimbal_ctrl.pitch_target_angle += delta_pitch;
			
			
			  yaw_cmd_final = gimbal_ctrl.yaw_target_angle * 180.0f / 3.1415926f;
        pitch_cmd_final = gimbal_ctrl.pitch_target_angle;
        
				
				if (gimbal_ctrl.pitch_target_angle > 1.05f) {
						gimbal_ctrl.pitch_target_angle = 1.05f;
				} else if (gimbal_ctrl.pitch_target_angle < -1.05f) {
						gimbal_ctrl.pitch_target_angle = -1.05f;
				}

				yaw_cmd_final = gimbal_ctrl.yaw_target_angle * 180.0f / 3.1415926f;
        pitch_cmd_final = gimbal_ctrl.pitch_target_angle;
				
				yaw_target_vel = (gimbal_ctrl.yaw_target_angle - gimbal_ctrl.last_yaw_target_angle) * 1000.0f * 1.0f;  //Angular velocity feedforward
				
				gimbal_ctrl.last_yaw_target_angle = gimbal_ctrl.yaw_target_angle;
				
				if (vision_data.is_tracking && is_target_locked == 0) {
					_Bool is_pos_stable = (fabs(pix_err_x) < LOCK_ON_ERROR_THRESHOLD_PIXELS) && 
                          (fabs(pix_err_y) < LOCK_ON_ERROR_THRESHOLD_PIXELS);
					_Bool is_speed_stable = (fabs(vision_data.vel_x_pps) < LOCK_ON_SPEED_THRESHOLD_PPS) &&
                            (fabs(vision_data.vel_y_pps) < LOCK_ON_SPEED_THRESHOLD_PPS);
					if (is_pos_stable && is_speed_stable) {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
						is_target_locked = 1; 
					}
			  }
    }
		
	// --- Step F: Draw Overlapping Circles (Optional Section) ---
		 float pitch_target_vel = 0.0f;
    if (gimbal_ctrl.mode == 3 && vision_data.dist_cm > 0) {
        // Update time t
        gimbal_ctrl.circle_t += 0.001f; // Called once every 1ms
        
        // Calculate angular radius alpha = atan(R / D)
        float alpha_rad = atan2f(CIRCLE_RADIUS_CM, vision_data.dist_cm);
        
				float r_scale = 1.0f;
        float ramp_time = 2.0f;
        if (gimbal_ctrl.circle_t < ramp_time) {
            r_scale = gimbal_ctrl.circle_t / ramp_time;
        }
        // Superimposed sine waves
        float omega = 2.0f * PI * CIRCLE_SPEED_HZ;
				float phase = omega * gimbal_ctrl.circle_t;
				
				
				float yaw_offset_rad   = alpha_rad * r_scale * cosf(phase);
        float pitch_offset_rad = alpha_rad * r_scale * sinf(phase);
				
				float yaw_circle_vel_rads   = -alpha_rad * r_scale * omega * sinf(phase);
        float pitch_circle_vel_rads =  alpha_rad * r_scale * omega * cosf(phase);

        yaw_cmd_final = (gimbal_ctrl.yaw_target_angle + yaw_offset_rad) * 180.0f / PI;
        pitch_cmd_final = gimbal_ctrl.pitch_target_angle + pitch_offset_rad;
				
				yaw_target_vel += yaw_circle_vel_rads; 
				pitch_target_vel += pitch_circle_vel_rads; 
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    }

			// --- Step G: Send to Dmiao Motor (MIT Mode) ---
			// MIT parameters: Pos, Vel, Kp, Kd, Torq
			// The motor's internal Kp and Kd are used to maintain stiffness. Recommended values

		Gimbal_Control_Steady(yaw_cmd_final, pitch_cmd_final, yaw_target_vel);  						
    Motor_Two_Ctrl(pitch_cmd_final, 0.0f, 4.0f, 0.03f, 0.0f);
}
*/


void Motor_Move_By_Angle(uint8_t motor_id, Motor_Direction_e direction, float angle_deg)
{
    float angle_rad = angle_deg * (PI / 180.0f);
    float *target_ptr;
	

    if (motor_id == 1) //  Motor1 (Yaw)
    {
        target_ptr = &gimbal_ctrl.yaw_target_angle;

    }
    else if (motor_id == 2) //  Motor2 (Pitch)
    {
        target_ptr = &gimbal_ctrl.pitch_target_angle;
    }
		
    else 
    {
        return;
    }

    if (direction == DIR_CW) 
    {
        *target_ptr += angle_rad; 
    }
    else // DIR_CCW
    {
        *target_ptr -= angle_rad;
    }
		
		if(motor_id == 1)
			Motor_One_Ctrl(*target_ptr, 0.0f, 2.0f, 0.05f, 0.0f);
		if(motor_id == 2)
			Motor_Two_Ctrl(*target_ptr, 0.0f, 2.0f, 0.05f, 0.0f);

}


void On_Vision_Data_Received(float new_target_pos) {
    uint32_t dt_ms = HAL_GetTick() - Vision_Frame_Tick; 
    Vision_Frame_Tick = HAL_GetTick();
	
    if (dt_ms == 0 || dt_ms > 100) dt_ms = 15; 
	
    float drift = new_target_pos - Interpolated_SetPoint;
    
    Correction_Increment = drift / (float)dt_ms;
}