/*
 * pid_controller.c
 *
 *  Created on: Oct 11, 2023
 *      Author: Rijin
 */

#include "pid.h"
#include <math.h>
#include <stdbool.h>


/* Initialize all calculated controller variables to zero. */
void PIDController_Init(PIDController * controller)
{
	controller->integral = 0.0f;
	controller->derivative = 0.0f;
	controller->prev_error = 0.0f;
	controller->motor_output = 0.0f;
}

/* Calculate all Proportional, Integral, and Derivative values based on
   set pitch angle and IMU reading every loop. */
void PIDController_Update(PIDController *controller, float setpoint, float imu_reading)
{
	// Error from current IMU reading and set pitch angle
	float error = setpoint - imu_reading;

	// Compute proportional
	float proportional = controller->kp * error;
	controller->proportional = proportional;

	// Compute integral
	controller->integral += error;
	float integral = controller->ki * controller->integral;
	/* Integral Clamping/Windup */
	if (integral < -150) {
	    integral = -150;
	    controller->integral = -150;
	} else if (integral > 150) {
	    integral = 150;
	    controller->integral = 150;
	}

	// Compute derivative
	controller->derivative = (error - controller->prev_error)/((controller->sampling_time)/1000.0f);
	float derivative = controller->kd * controller->derivative;
	controller->motor_output = proportional+ integral + derivative ;
	// int control_output = controller->motor_output;

	// if(control_output>0){
	// 	if(control_output<100){
	// 		controller->motor_output= 100;
	// 	}else if(control_output>1000){
			
	// 		controller->motor_output = 1000;
	// 	}else{
	// 		controller->motor_output = controller->motor_output;
	// 	}
	// }else{
	// 	if(control_output>-100){
	// 		controller->motor_output= -100;
	// 	}else if(control_output<-1000){
			
	// 		controller->motor_output = -1000;
	// 	}else{
	// 		controller->motor_output = controller->motor_output;
	// 	}
	// }
	// Store current error as previous
	controller->prev_error = error;
}
