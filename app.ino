#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/int32.h>

#include <Arduino_LSM9DS1.h>

rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_mag;

rcl_subscription_t pwm_subscriber;

rclc_executor_t executor;
rclc_executor_t motor_executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

sensor_msgs__msg__Imu msg_imu;
sensor_msgs__msg__MagneticField msg_mag;
std_msgs__msg__Int32 msg_pwm;

int motor0_pin = 14;

void motor_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg_pwm = (const std_msgs__msg__Int32 *)msgin;
  //analogWrite(MOTOR0_PIN, 1000);
  
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    if(IMU.accelerationAvailable())
    {
      IMU.readAcceleration(ax,ay,az);
    }
    if(IMU.gyroscopeAvailable())
    {
      IMU.readGyroscope(gx, gy, gz);
    }
    if(IMU.magneticFieldAvailable())
    {
      IMU.readMagneticField(mx, my, mz);
    }
    
    //ax = ax*9.797;
    //ay = ay*9.797;
    //az = az*9.797;
    msg_imu.linear_acceleration.x = ax;
    msg_imu.linear_acceleration.y = ay;
    msg_imu.linear_acceleration.z = az;

    gx = gx*3.142/180;
    gy = gy*3.142/180;
    gz = gz*3.142/180;
    msg_imu.angular_velocity.x = gx;
    msg_imu.angular_velocity.y = gy;
    msg_imu.angular_velocity.z = gz;

    msg_mag.magnetic_field.x = mx;
    msg_mag.magnetic_field.y = my;
    msg_mag.magnetic_field.z = mz;

    rcl_publish(&publisher_imu, (const void *) &msg_imu, NULL);
    rcl_publish(&publisher_mag, (const void *) &msg_mag, NULL);
  }
}

void setup() {
  set_microros_transports();
  IMU.begin();

  pinMode(motor0_pin, OUTPUT);
  
  delay(5000);
  analogWriteResolution(12);
  analogWriteFrequency(motor0_pin, 250);
  delay(1000);
  analogWrite(motor0_pin, 4095);
  delay(1000);
  analogWrite(motor0_pin, 0);
  delay(1000);
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  allocator = rcl_get_default_allocator();

  // create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "pi_esp_communicator", "", &support);

  // create publisher
  rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/copto/imu");

  rclc_publisher_init_default(
    &publisher_mag,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "/copto/mag");


  // create subscriber
  rclc_subscription_init_default(
    &pwm_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/copto/motor0_pwm");

  // create timer,
  const unsigned int timer_timeout = 100;
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback);

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  motor_executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&motor_executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&motor_executor, &pwm_subscriber, &msg_pwm, &motor_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin(&executor);
  delay(100);
  rclc_executor_spin(&motor_executor);
  delay(100);
  analogWrite(motor0_pin, 1000);
  delay(100);
}