#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_mag;

sensor_msgs__msg__Imu msg_imu;
sensor_msgs__msg__MagneticField msg_mag;

LSM9DS1 imu;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		
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

extern "C" void appMain(void * arg)
{
	Wire.begin();
	imu.begin();

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "pi_esp_communicator", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_best_effort(
		&publisher_imu,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"/copto/imu"));

	RCCHECK(rclc_publisher_init_best_effort(
		&publisher_mag,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
		"/copto/mag"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher_imu, &node))
	RCCHECK(rcl_publisher_fini(&publisher_mag, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}