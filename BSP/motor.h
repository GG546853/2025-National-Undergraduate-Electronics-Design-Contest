#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "main.h"

#define PI 3.14159265f

// Image parameters (According to your camera resolution 640x480)
#define IMG_CENTER_X 320.0f
#define IMG_CENTER_Y 240.0f

// Pixel-to-radian conversion factor K (needs to be measured! Assume 100 pixels = 0.1 radians)
#define PIXEL_TO_RAD_K  0.00146f 

// Motor installation reverse correction (1 or -1)
#define YAW_DIR_COEFF   1.0f  
#define PITCH_DIR_COEFF -1.0f

// Circle drawing parameters
#define CIRCLE_RADIUS_CM 6.0f
#define CIRCLE_SPEED_HZ  (1.0f / 17.0f)   // Draw a circle in 1 second

#define LOCK_ON_ERROR_THRESHOLD_PIXELS  15.0f   //Position error threshold: allows a fluctuation of 2 pixels up, down, left, and right from the center point
#define LOCK_ON_SPEED_THRESHOLD_PPS     60.0f   //Speed threshold: The target is considered stationary when its speed is less than 5 pixels/second

//Distance Range Definition (Unit: mm)
#define DIST_MIN 500.0f   
#define DIST_MAX 1500.0f  

#define ADAPT_GAIN_MAX 1.0f  // Gain ratio at 500mm (100% intensity)
#define ADAPT_GAIN_MIN 0.99f  // Gain ratio at 1500mm (40% strength to prevent remote shaking)

typedef enum {
    DIR_CCW = 0, // Clockwise
    DIR_CW  = 1  // Counter-Clockwise
} Motor_Direction_e;


// Visual data reception structure (corresponding to Python struct.pack)
typedef struct {
    uint8_t header[2]; // 0x5A 0x5A
    int16_t x_raw;
    int16_t y_raw;
    int16_t dist_raw;  // dist * 10
    uint8_t checksum;
} __attribute__((packed)) Vision_Packet_t;

// Data after visual unpacking
typedef struct {
    float cur_x;      // Pixel
    float cur_y;      // Pixel
	   float vel_x_pps;  // <-- New: Velocity in pixels per second
    float vel_y_pps;  // <-- New: Velocity in pixels per second
    float dist_cm;    // Distance
    uint32_t last_update_tick; // Disconnect Detection
    uint8_t is_tracking;       // Is there a target
		uint8_t last_is_tracking; // <-- Added: Tracking status from the previous frame
} Vision_Data_t;


typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float max_out;    // Output limiting (maximum single increment)
    float max_iout;   // Integral Clamping
    
    float err;
    float last_err;
    float i_err;
    float out;
} PID_t;

// PTZ main control structure
typedef struct {
    // Target state
    float yaw_target_angle;   // Yaw angle sent to the motor (Global)
    float pitch_target_angle; /// Final Pitch angle sent to the motor (Global)
    
		float last_yaw_target_angle;
	
		float last_pitch_target_angle;
	
		float yaw_target_speed;
		float pitch_target_speed;		
	
    uint8_t mode; // 0: Aim mode, 1: Draw circle mode
    float circle_t; // Circle drawing time accumulator

    // PID
    PID_t pid_yaw;
    PID_t pid_pitch;
	
	
} Gimbal_Ctrl_t;


extern Vision_Data_t vision_data;
extern Vision_Data_t vision_data;
extern Gimbal_Ctrl_t gimbal_ctrl;
extern float Interpolated_SetPoint;
extern float Pos_Increment;
extern uint8_t is_target_locked ;
extern float fGyro_ang[3];
	

void Gimbal_Init(void);
void Gimbal_Control_Loop(void);
void Gimbal_Control_Loop2(void);
void Motor_Enable();
void Motor_Disable();
void Gimbal_Set_Zero(void);
void Motor_One_Ctrl(float pos, float vel,float kp, float kd, float tor);
void Motor_Two_Ctrl(float pos, float vel,float kp, float kd, float tor);
void Motor_One_Spd_Ctrl(float vel);
void Motor_Move_By_Angle(uint8_t motor_id, Motor_Direction_e direction, float angle_deg);
void On_Vision_Data_Received(float new_target_pos);
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_iout);
void Gimbal_Control_Steady(float gimbal_target_angle_Yaw, float gimbal_target_angle_Pitch, float yaw_target_vel);

#endif

