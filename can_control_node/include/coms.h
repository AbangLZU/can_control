#ifndef __COMS_H__
#define __COMS_H__

#define PI 3.14159


#define STEERLING_TX 0x215
#define STEERLING_RX 0x322
#define BREAK_TX 0x130
#define BREAK_RX 0x100
#define CH1_CH2 0x600
#define CH3_CH4 0x602
#define THROTTLE_TX 0x620
#define THROTTLE_RX 0x630

#define THROTTLE_TX NULL
#define THROTTLE_RX NULL

// safety parameters
#define SPEED_LIMIT 4.0 //2.78  // m/s, IMPORTANT: this is safty limit
#define _NOBRAKE_CONTROL_MAX_VELOCITY 4.0 // 2.77 m/s = 10 km/hr, it must be less than _SPEED_LIMIT
#define H_NOBRAKE _NOBRAKE_CONTROL_MAX_VELOCITY * 0.1 // 10% of _NOBRAKE_CONTROL_MAX_VELOCITY
#define H_ACCEL_BRAKE 0.83 // 0.83 m/s = 3 km/hr, _NOBRAKE_CONTROL_MAX_VELOCITY - H_ACCEL_BRAKE "must" > 0

// coms parameters
#define WHEEL_BASE 2.66  // tire-to-tire size of COMS.
#define WHEEL_TRACK 1.50 // front wheel distance in meter
#define WHEEL_ANGLE_MAX 31.28067 // max angle of front tires.
#define STEERING_ANGLE_MAX 666 // max angle of steering
#define STEERING_ANGLE_LIMIT 550 // could be STEERING_ANGLE_MAX but...
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX/WHEEL_ANGLE_MAX) //number of steering angle to true unit wheel angle
#define STEERING_INTERNAL_PERIOD 20 // ms (10ms is too fast for COMS)

// accel/brake parameters
#define _K_ACCEL_P 230.0
#define _K_ACCEL_I 12.0
#define _K_ACCEL_D 50.0
#define _K_ACCEL_I_CYCLES 20
#define _ACCEL_MAX_I 1000
#define _ACCEL_STROKE_DELTA_MAX 100  // Autoware default 1000
#define _ACCEL_RELEASE_STEP 400
#define _ACCEL_PEDAL_MAX 2000

#define _K_BRAKE_P 1000.0
#define _K_BRAKE_I 20.0
#define _K_BRAKE_D 50.0
#define _K_BRAKE_I_CYCLES 20
#define _BRAKE_MAX_I 200
#define _BRAKE_STROKE_DELTA_MAX 2000
#define _BRAKE_RELEASE_STEP 500
#define _BRAKE_PEDAL_MAX 4095
#define _BRAKE_PEDAL_MED 3000

// missing parameters
#define _BRAKE_PEDAL_OFFSET 1000
#define _ACCEL_PEDAL_OFFSET 200
#define _BRAKE_PEDAL_STOPPING_MED 2000
#define _BRAKE_PEDAL_STOPPING_MAX 2500
#define _K_ACCEL_P_UNTIL20 230.0
#define _K_ACCEL_I_UNTIL20 12.0
#define _K_ACCEL_D_UNTIL20 50.0
#define _K_ACCEL_P_UNTIL10 230.0
#define _K_ACCEL_I_UNTIL10 12.0
#define _K_ACCEL_D_UNTIL10 50.0
#define USE_ACCEL_STROKE_DELTA_MAX


// steering parameters
#define _STEERING_MAX_ANGVELSUM 1000
#define _K_STEERING_TORQUE 10
#define _K_STEERING_TORQUE_I 0.5
#define _STEERING_MAX_TORQUE 2000
#define _STEERING_MAX_SUM 100 //deg*0.1s for I control

#define _K_STEERING_P 60
#define _K_STEERING_I 13
#define _K_STEERING_D 10

#define _STEERING_ANGLE_ERROR 0 // deg

#endif
