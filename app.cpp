#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/int32.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef ESP_PLATFORM
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
// #include "driver/gpio.h"
#include <Wire.h>
#include <SPI.h>
// #include <ESP32Servo.h>
#include <SparkFunLSM9DS1.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define MAX_VAL 	100
#define MIN_VAL 	0

#define MOTOR0_GPIO 13
#define LEDC_FREQ	50000

rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_mag;

rcl_subscription_t pwm_subscriber;


ledc_channel_config_t ledc_channel;
ledc_timer_config_t ledc_timer;

sensor_msgs__msg__Imu msg_imu;
sensor_msgs__msg__MagneticField msg_mag;
std_msgs__msg__Int32 msg_pwm;

LSM9DS1 imu;
float i = 10.0;
bool isSet = false;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{	
	RCLC_UNUSED(last_call_time);
	if (timer != NULL){
		
		
        if(imu.accelAvailable())
		{
			imu.readAccel();
		}
		if(imu.gyroAvailable())
		{
			imu.readGyro();
		}
		if(imu.magAvailable())
		{
			imu.readMag();
		}
		
		double ax = imu.calcAccel(imu.ax)*9.797;
		double ay = imu.calcAccel(imu.ay)*9.797;
		double az = imu.calcAccel(imu.az)*9.797;
		msg_imu.linear_acceleration.x = ax;
        msg_imu.linear_acceleration.y = ay;
        msg_imu.linear_acceleration.z = az;

		double gx = imu.calcGyro(imu.gx)*3.142/180;
		double gy = imu.calcGyro(imu.gy)*3.142/180;
		double gz = imu.calcGyro(imu.gz)*3.142/180;
		msg_imu.angular_velocity.x = gx;
        msg_imu.angular_velocity.y = gy;
        msg_imu.angular_velocity.z = gz;

		double mx = imu.calcMag(imu.mx);
		double my = imu.calcMag(imu.my);
		double mz = imu.calcMag(imu.mz);
		msg_mag.magnetic_field.x = mx;
        msg_mag.magnetic_field.y = my;
        msg_mag.magnetic_field.z = mz;

		RCSOFTCHECK(rcl_publish(&publisher_imu, (const void *) &msg_imu, NULL));
		RCSOFTCHECK(rcl_publish(&publisher_mag, (const void *) &msg_mag, NULL));
	}
}

void motor_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg_pwm = (const std_msgs__msg__Int32 *)msgin;
	ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 125); 
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}

extern "C" void appMain(void * arg)
{
	Wire.begin();
	imu.begin();

	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .freq_hz = LEDC_FREQ,                // frequency of PWM signal  
     };
     ledc_timer_config(&ledc_timer);
 
    ledc_channel_config_t ledc_channel = {
		 	.gpio_num   = MOTOR0_GPIO,
		 	.speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,        
     	};
     ledc_channel_config(&ledc_channel);

	ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
	ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
	
	///////////////////////////////////////////////////////////////////////////////////////////////////
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "pi_esp_communicator", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher_imu,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"/copto/imu"));

	RCCHECK(rclc_publisher_init_default(
		&publisher_mag,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
		"/copto/mag"));


	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&pwm_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/copto/motor0_pwm"));

	// create timer,
	rcl_timer_t timer;
	
	const unsigned int timer_timeout = 100;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));
	
	sensor_msgs__msg__Imu__init(&msg_imu);
	sensor_msgs__msg__MagneticField__init(&msg_mag);
	std_msgs__msg__Int32__init(&msg_pwm);

	// create executor
	rclc_executor_t executor;
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	rclc_executor_t motor_executor;
	motor_executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&motor_executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_subscription(&motor_executor, &pwm_subscriber, &msg_pwm, &motor_callback, ON_NEW_DATA));
	
	
	while(1){
		rclc_executor_spin(&executor);
		rclc_executor_spin(&motor_executor);
	}
	vTaskDelete(NULL);
	// free resources
	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rclc_executor_fini(&motor_executor));
	RCCHECK(rcl_publisher_fini(&publisher_imu, &node));
	RCCHECK(rcl_publisher_fini(&publisher_mag, &node));
	RCCHECK(rcl_subscription_fini(&pwm_subscriber, &node));
	RCCHECK(rcl_timer_fini(&timer));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rclc_support_fini(&support));

	sensor_msgs__msg__Imu__fini(&msg_imu);
	sensor_msgs__msg__MagneticField__fini(&msg_mag);
	std_msgs__msg__Int32__fini(&msg_pwm);
}