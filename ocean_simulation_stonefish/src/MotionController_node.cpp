#include <ros/ros.h>
#include "pid/pid.h"
#include <sensor_msgs/Imu.h>

typedef enum {NONE_MODE, MANUAL_MODE, STABILIZE_MODE, ALTITUDE_MODE, POSITION_MODE, DOCK_MODE} USV_STATE;
typedef enum {NORMAL_MODE, AMERICAN_MODE} JOY_STATE;

double roll_set = 0.0f, pitch_set = 0.0f, yaw_set = 0.0f;
double roll_raw, pitch_raw, yaw_raw, depth_raw;
double roll_offset, pitch_offset, yaw_offset, depth_offset;
double usv_roll, usv_pitch, usv_yaw, usv_depth;

double yaw_now, yaw_manual_set, depth_now, depth_manual_set;

int cnt_printf = 0;
int button_buf[6] = {0};

USV_STATE STATE;
JOY_STATE joy_mode;

std_msgs::Float64 PWMX;
std_msgs::Float64 PWMY;
std_msgs::Float64 PWMZ;
std_msgs::Float64 PWMD;

std_msgs::Float64 std_usv_roll;
std_msgs::Float64 std_usv_pitch;
std_msgs::Float64 std_usv_yaw;
std_msgs::Float64 std_usv_depth;

sensor_msgs::Imu px4_imu;

PID_AbsoluteType PID_Angle_ROLL, PID_Angle_PITCH, PID_Angle_YAW, PID_Depth;

void PID_Init_Angle_ROLL(float P,float I,float D);
void PID_Init_Angle_PITCH(float P,float I,float D);
void PID_Init_Angle_YAW(float P,float I,float D);
void PID_Init_Depth(float P,float I,float D);

void PID_Calculate(PID_AbsoluteType* PID, float limit_out);
double angle_mapping(double angle_raw);
double find_min_error(double sp, double val);
void usv_info_printf(void);
double error_correct(double data, double data_offset);

void PID_Init(const geometry_msgs::Vector3::ConstPtr& msg);
void IMU_CALLBACK(const sensor_msgs::Imu::ConstPtr &msg);
void Joy_Callback(const sensor_msgs::Joy::ConstPtr& msg);
void Depth_Sensor_Callback(const mz_usv::MS5837::ConstPtr& msg);
void Yaw_set_callback(const std_msgs::Float64::ConstPtr& msg);
void Usv_mode_callback(const std_msgs::Int8::ConstPtr& msg);
void Usv_joy_mode_callback(const std_msgs::Int8::ConstPtr& msg);
void Usv_set_pitch_callback(const std_msgs::Float64::ConstPtr& msg);
void Usv_set_roll_callback(const std_msgs::Float64::ConstPtr& msg);

#define ERROR_IF_NEAR_ZERO(err, limit) ((err) < limit && (err) > -limit ? (err) = 0.0 : (err))
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "dowmup_PID");
  ros::NodeHandle nh;
 	ros::Subscriber sub1 = nh.subscribe("/joy", 2,Joy_Callback);
 	// ros::Subscriber sub2 = nh.subscribe("/imu/rpy",2, PID_Init);
 	ros::Subscriber sub3 = nh.subscribe("depth", 20, Depth_Sensor_Callback);
  ros::Subscriber sub4 = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 50, IMU_CALLBACK);
  ros::Subscriber sub5 = nh.subscribe("/mother_usv/imu/yaw", 50, Yaw_set_callback);
  ros::Subscriber sub6 = nh.subscribe("/under_usv/usv_mode", 10, Usv_mode_callback);
  ros::Subscriber sub7 = nh.subscribe("/under_usv/joy_mode", 10, Usv_joy_mode_callback);
  ros::Subscriber sub8 = nh.subscribe("/under_usv/joy/ptich_set", 10, Usv_set_pitch_callback);
  ros::Subscriber sub9 = nh.subscribe("/under_usv/joy/roll_set", 10, Usv_set_roll_callback);


 	ros::Publisher pwm_X = nh.advertise<std_msgs::Float64>("/PID/X", 5); // 映射： X -> ROLL 
 	ros::Publisher pwm_Y = nh.advertise<std_msgs::Float64>("/PID/Y", 5); // 映射： Y -> PITCH 
  ros::Publisher pwm_Z = nh.advertise<std_msgs::Float64>("/PID/Z", 5); // 映射： Z -> YAW
  ros::Publisher pwm_D = nh.advertise<std_msgs::Float64>("/PID/depth", 5);

  ros::Publisher usv_roll_pub  = nh.advertise<std_msgs::Float64>("son/usv_roll" , 5);
 	ros::Publisher usv_pitch_pub = nh.advertise<std_msgs::Float64>("son/usv_pitch", 5); 
  ros::Publisher usv_yaw_pub   = nh.advertise<std_msgs::Float64>("son/usv_yaw"  , 5);
  ros::Publisher usv_depth_pub = nh.advertise<std_msgs::Float64>("son/usv_depth", 5);

  ros::Rate loop_rate(50);  

  PID_Init_Angle_ROLL (7.0,   0.015, 0.5);  
  PID_Init_Angle_PITCH(7.0,   0.015, 0.5);
  PID_Init_Angle_YAW  (5.0,   0.000, 0.5);
  PID_Init_Depth      (500.0, 1.000, 0.5);  

  while (ros::ok()) 
  {
    switch (STATE) 
	  {
      case NONE_MODE:
        PID_Angle_ROLL.errNow  = 0.00f;
        PID_Angle_PITCH.errNow = 0.00f;
        PID_Angle_YAW.errNow   = 0.00f;
        PID_Depth.errNow       = 0.00f;
        break;
      case MANUAL_MODE:
        PID_Angle_ROLL.errNow  = 0.00f;
        PID_Angle_PITCH.errNow = 0.00f;
        PID_Angle_YAW.errNow   = 0.00f;
        PID_Depth.errNow       = 0.00f;
        break;
      case STABILIZE_MODE:
        PID_Angle_ROLL.errNow  = usv_roll  - 0.00f;
        PID_Angle_PITCH.errNow = usv_pitch - 0.00f;
        PID_Angle_YAW.errNow   = 0.00f;
        PID_Depth.errNow       = 0.00f;
        break;
      case ALTITUDE_MODE:
        PID_Angle_ROLL.errNow  = usv_roll  - 0.00f;
        PID_Angle_PITCH.errNow = usv_pitch - 0.00f;
        PID_Angle_YAW.errNow   = 0.00f;
        PID_Depth.errNow       = usv_depth - depth_manual_set;
        break;
		  case POSITION_MODE:
        switch (joy_mode) 
		    {
    		  case NORMAL_MODE:
            PID_Angle_ROLL.errNow  = usv_roll  - 0.00f;
            PID_Angle_PITCH.errNow = usv_pitch - 0.00f;
            PID_Angle_YAW.errNow   = find_min_error(yaw_manual_set, usv_yaw);
            PID_Depth.errNow       = usv_depth - depth_manual_set;
        	  break;
    		  case AMERICAN_MODE:
            PID_Angle_ROLL.errNow  = usv_roll  - roll_set;
            PID_Angle_PITCH.errNow = usv_pitch - pitch_set;
            PID_Angle_YAW.errNow   = find_min_error(yaw_manual_set, usv_yaw);
            PID_Depth.errNow       = usv_depth - depth_manual_set;
        	  break;
    		  default:
				    PID_Angle_ROLL.errNow  = usv_roll  - 0.00f;
            PID_Angle_PITCH.errNow = usv_pitch - 0.00f;
            PID_Angle_YAW.errNow   = find_min_error(yaw_manual_set, usv_yaw);
            PID_Depth.errNow       = usv_depth - depth_manual_set;
        	  break;
		    }
        break;
      case DOCK_MODE:
        PID_Angle_ROLL.errNow  = usv_roll  - 0.00f;
        PID_Angle_PITCH.errNow = usv_pitch - 0.00f;
        PID_Angle_YAW.errNow   = find_min_error(yaw_set, usv_yaw);
        PID_Depth.errNow       = usv_depth - depth_manual_set;
        break;
      default:
        PID_Angle_ROLL.errNow  = 0.00f;
        PID_Angle_PITCH.errNow = 0.00f;
        PID_Angle_YAW.errNow   = 0.00f;
        PID_Depth.errNow       = 0.00f;
        break;
	  }

    ERROR_IF_NEAR_ZERO(PID_Angle_ROLL.errNow,  1.000f);
    ERROR_IF_NEAR_ZERO(PID_Angle_PITCH.errNow, 1.000f);
    ERROR_IF_NEAR_ZERO(PID_Angle_YAW.errNow,   1.000f);
    ERROR_IF_NEAR_ZERO(PID_Depth.errNow,       0.010f);

    PID_Calculate(&PID_Angle_ROLL,  200.0f);
    PID_Calculate(&PID_Angle_PITCH, 200.0f);
    PID_Calculate(&PID_Angle_YAW,   100.0f);
    PID_Calculate(&PID_Depth,       200.0f);

    PWMX.data = PID_Angle_ROLL.ctrOut; 
    PWMY.data = PID_Angle_PITCH.ctrOut;
    PWMZ.data = PID_Angle_YAW.ctrOut;
    PWMD.data = PID_Depth.ctrOut;

    if((++cnt_printf) % 5 == 0)
    {
      cnt_printf = 0;
      usv_info_printf();
    }

    pwm_X.publish(PWMX);
		pwm_Y.publish(PWMY);
    pwm_Z.publish(PWMZ);
    pwm_D.publish(PWMD);

    usv_roll_pub.publish(std_usv_roll);
    usv_pitch_pub.publish(std_usv_pitch);
    usv_yaw_pub.publish(std_usv_yaw);
    usv_depth_pub.publish(std_usv_depth);

    ros::spinOnce();
    loop_rate.sleep();
     
  }
  return 0;
}

void usv_info_printf(void)
{
    std::cout << "\033[2J\033[H";
    switch (STATE) 
		{
    		case 0:
          ROS_INFO("MODE: NONE_MODE(%d)", STATE);
        	break;
    		case 1:
          ROS_INFO("MODE: MANUAL_MODE(%d)", STATE);
        	break;
    		case 2:
          ROS_INFO("MODE: STABILIZE_MODE(%d)", STATE);
        	break;
			  case 3:
          ROS_INFO("MODE: ALTITUDE_MODE(%d)", STATE);
        	break;
        case 4:
          ROS_INFO("MODE: POSITION_MODE(%d)", STATE);
        	break;
        case 5:
          ROS_INFO("MODE: DOCK_MODE(%d)", STATE);
        	break;
    		default:
				  ROS_INFO("MODE: UNKOWN_MODE(%d)", STATE);
        	break;
		}
    switch (joy_mode) 
		{
    		case 0:
          ROS_INFO("MODE: NORMAL_MODE(%d)", joy_mode);
        	break;
    		case 1:
          ROS_INFO("MODE: AMERICAN_MODE(%d)", joy_mode);
        	break;
    		default:
				  ROS_INFO("MODE: UNKOWN_JOY_MODE(%d)", joy_mode);
        	break;
		}
    ROS_INFO("  USV attitude raw : [Roll: %6.2f, Pitch: %6.2f, Yaw: %6.2f, Depth: %6.2fm]", (roll_raw / M_PI) * 180.0f, (pitch_raw / M_PI) * 180.0f, angle_mapping(yaw_raw), depth_raw);
    ROS_INFO("  USV attitude     : [Roll: %6.2f, Pitch: %6.2f, Yaw: %6.2f, Depth: %6.2fm]", usv_roll, usv_pitch, usv_yaw, usv_depth);
    ROS_INFO("  USV attitude set : [Roll: %6.2f, Pitch: %6.2f, Yaw: %6.2f, Depth: %6.2fm]", 0.000, 0.000, yaw_set, depth_manual_set);
    ROS_INFO("  PID output       : [Roll: %6.2f, Pitch: %6.2f, Yaw: %6.2f, Depth: %6.2fm]", PWMX.data, PWMY.data, PWMZ.data, PWMD.data);
}

void PID_Init_Angle_ROLL(float P,float I,float D)
{
  PID_Angle_ROLL.kd=D;
  PID_Angle_ROLL.ki=I; 
  PID_Angle_ROLL.kp=P;
}

void PID_Init_Angle_PITCH(float P,float I,float D)
{
  PID_Angle_PITCH.kd=D;
  PID_Angle_PITCH.ki=I;
  PID_Angle_PITCH.kp=P;
}

void PID_Init_Angle_YAW(float P,float I,float D)
{
  PID_Angle_YAW.kd=D;
  PID_Angle_YAW.ki=I;
  PID_Angle_YAW.kp=P;
}

void PID_Init_Depth(float P,float I,float D)
{
  PID_Depth.kd=D;
  PID_Depth.ki=I;
  PID_Depth.kp=P;
}

void PID_Calculate(PID_AbsoluteType* PID, float limit_out)
{
  PID->errP = PID->errNow;
  PID->errD = PID->errNow - PID->errOld;
  PID->errOld = PID->errNow;
  PID->errI = PID->errI + PID->errNow;
  PID->ctrOut = (float)( PID->kp * PID->errP+ PID->ki * PID->errI + PID->kd * (PID->errD * 10));
   
  if(PID->ctrOut > limit_out)
  {
    PID->ctrOut =  limit_out;
  }
  if(PID->ctrOut < -limit_out)
  {
    PID->ctrOut = -limit_out;
  }
}

double angle_mapping(double angle_raw)
{
    angle_raw = fmod(angle_raw + M_PI, 2 * M_PI); 
    if (angle_raw < 0) angle_raw += 2 * M_PI;
    
    // 转换为0-360度
    double angle_deg = angle_raw * 180.0f / M_PI;
    return angle_deg;
}

double find_min_error(double sp, double val)
{
  double err_1 = fabs(sp - val);
  double err_2 = fabs(360.0 -  fabs(sp - val));

  if (err_1 <= err_2)
  {
    return (sp - val);
  }
  else
  {
    if((sp - val) <= 0)
      return (fabs(360.0 -  fabs(sp - val)));
    else
      return (-fabs(360.0 -  fabs(sp - val)));
  }
}

double error_correct(double data, double data_offset)
{
  if(data_offset > -1.0f && data_offset < 1.0f)
  {
    return data;
  }
  else
  {
    if(data + data_offset < 0)
    {
      return data + data_offset + 360.0f;
    }
    else if(data + data_offset > 360.0f)
    {
      return data + data_offset - 360.0f;
    }
    else
    {
      return data + data_offset;
    }
  }
}

void IMU_CALLBACK(const sensor_msgs::Imu::ConstPtr &msg)
{
  px4_imu = *msg;

  double px4_roll, px4_pitch, px4_yaw;

  tf::Quaternion q(px4_imu.orientation.x, px4_imu.orientation.y, px4_imu.orientation.z, px4_imu.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_raw, pitch_raw, yaw_raw);

  //ROS_INFO("PX4 Orientation Raw    : [Roll: %.3f, Pitch: %.3f, Yaw: %.3f]", roll_raw, pitch_raw, yaw_raw);
  // 修正偏移量， 规定船头上仰pitch为正，船右倾roll为正
  px4_roll  = (roll_raw  - roll_offset);
  px4_pitch = -(pitch_raw - pitch_offset);
  px4_yaw   = (yaw_raw   - 0.0000); 

  //ROS_INFO("PX4 Orientation Correct: [Roll: %.3f, Pitch: %.3f, Yaw: %.3f]", px4_roll, px4_pitch, px4_yaw);
  // 归一化处理，原为-π to π，这里缩放成-180° - 180°, 57.2957795 = 180 / pi

  // px4_roll  = px4_roll  * 57.2957795;
  // px4_pitch = px4_pitch * 57.2957795;
  // px4_yaw   = px4_yaw   * 57.2957795;

  px4_roll  = (px4_roll / M_PI) * 180.0f;
  px4_pitch = (px4_pitch / M_PI) * 180.0f;
  px4_yaw   = error_correct(angle_mapping(px4_yaw), yaw_offset); // -π to π -> 0 - 360

  // ROS_INFO("PX4 Orientation Final  : [Roll: %.3f, Pitch: %.3f, Yaw: %.3f]", px4_roll, px4_pitch, px4_yaw);

  usv_roll  = px4_roll;
  usv_pitch = px4_pitch;
  usv_yaw   = px4_yaw;

  std_usv_roll.data  = usv_roll;
  std_usv_pitch.data = usv_pitch;
  std_usv_yaw.data   = usv_yaw;


}

void Joy_Callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  depth_joy = msg->axes[1];
  yaw_joy = msg->axes[2];

	// button[0]
	if((msg->buttons[0] == 1) && (button_buf[0] != msg->buttons[0])) 
	{
		button_buf[0] = msg->buttons[0];
	}
	else if((msg->buttons[0] == 0) && (button_buf[0] != msg->buttons[0]))
	{
		button_buf[0] = msg->buttons[0] ;
	}

  // button[1]
  if((msg->buttons[1] == 1) && (button_buf[1] != msg->buttons[1])) 
  {
    roll_offset = roll_raw;
    pitch_offset = pitch_raw;
    yaw_offset = yaw_set - angle_mapping(yaw_raw);
    depth_offset = depth_raw;

    if (yaw_offset > 180.0f) 
    {
      yaw_offset -= 360.0f;
    } 
    else if (yaw_offset < -180.0f) 
    {
      yaw_offset += 360.0f;
    }

    PID_Angle_ROLL.errI  = 0.0f;
    PID_Angle_PITCH.errI = 0.0f;
    PID_Angle_YAW.errI   = 0.0f;
    PID_Depth.errI       = 0.0f;

    button_buf[1] = msg->buttons[1];
  }
  else if((msg->buttons[1] == 0) && (button_buf[1] != msg->buttons[1]))
	{
		button_buf[1] = msg->buttons[1] ;
	} 
  
  // button[2]
	if((msg->buttons[2] == 1) && (button_buf[2] != msg->buttons[2]))
	{

	}
	else if((msg->buttons[2] == 0) && (button_buf[2] != msg->buttons[2]))
	{
		button_buf[2] = msg->buttons[2] ;
	} 

  // button[3]
	if((msg->buttons[3] == 1) && (button_buf[3] != msg->buttons[3]))
	{

	}
	else if((msg->buttons[3] == 0) && (button_buf[3] != msg->buttons[3]))
	{
		button_buf[3] = msg->buttons[3] ;
	} 
  //  ROS_INFO("PX4 Orientation offset : [Roll: %.3f, Pitch: %.3f, Yaw: %.3f]", roll_offset, pitch_offset, yaw_offset);

  if(msg->axes[0] < -0.005 || msg->axes[0] > 0.005)
  {
    switch (joy_mode) 
		{
      case NORMAL_MODE:
    	  break;
      case AMERICAN_MODE:
        yaw_manual_set = usv_yaw;
        PID_Angle_YAW.errI = 0;
    	  break;
      default:
    	  break;
		}
  }

  if(msg->axes[1] < -0.005 || msg->axes[1] > 0.005)
  {
    depth_manual_set = usv_depth;
    PID_Depth.errI = 0;
  }

  if(msg->axes[2] < -0.005 || msg->axes[2] > 0.005)
  {
    switch (joy_mode) 
		{
      case NORMAL_MODE:
        yaw_manual_set = usv_yaw;
        PID_Angle_YAW.errI = 0;
    	  break;
      case AMERICAN_MODE:
        PID_Angle_PITCH.errI = 0.00f;
        PID_Angle_ROLL.errI = 0.00f;
    	  break;
      default:
    	  break;
		}
  }

  if(msg->axes[3] < -0.005 || msg->axes[3] > 0.005)
  {
    switch (joy_mode) 
		{
      case NORMAL_MODE:
        // yaw_manual_set = usv_yaw;
        // PID_Angle_YAW.errI = 0;
    	  break;
      case AMERICAN_MODE:
        PID_Angle_PITCH.errI = 0.00f;
        PID_Angle_ROLL.errI = 0.00f;
    	  break;
      default:
    	  break;
		}
  }
}

void Depth_Sensor_Callback(const mz_usv::MS5837::ConstPtr& msg)
{
    depth_raw = msg->depth;

    usv_depth = depth_raw - depth_offset;

    std_usv_depth.data = usv_depth;

    // PID_Depth.errNow = depth_now - depth_manual_set;

    // ERROR_IF_NEAR_ZERO(PID_Depth.errNow, 0.01);
}

void Yaw_set_callback(const std_msgs::Float64::ConstPtr& msg)
{
  yaw_set = msg->data;
}

void Usv_mode_callback(const std_msgs::Int8::ConstPtr& msg)
{
  STATE = static_cast<USV_STATE>(msg->data);
}

void Usv_joy_mode_callback(const std_msgs::Int8::ConstPtr& msg)
{
  joy_mode = static_cast<JOY_STATE>(msg->data);
}

void Usv_set_pitch_callback(const std_msgs::Float64::ConstPtr& msg)
{
  pitch_set = msg->data;
}

void Usv_set_roll_callback(const std_msgs::Float64::ConstPtr& msg)
{
  roll_set = msg->data;
}

