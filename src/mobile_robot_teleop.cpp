
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <mobile_robot_teleop/VfomaHud.h>
#include <mobile_robot_teleop/VfomaSetting.h>
#include <string>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Duration.h>

class RobotTeleop
{
private:
  int gearNow = 0;
  int dirctionNow = 0;
  bool initted_ = false;
  ros::NodeHandle nh_;
  sensor_msgs::Joy* lastJoy = new sensor_msgs::Joy;
  
  ros::Publisher hud_pub_, acm_pub_, set_pub_;
  ros::Subscriber joy_sub_;

  mobile_robot_teleop::VfomaHud hud;
  

  struct JoyConfig 
    {
        int gearUpLeft;
        int gearUpMiddle;
        int gearUpRight;
        int gearDownLeft;
        int gearDownMiddle;
        int gearDownRight;

        int steeringWheel;
        int linearVelPedal;
        int breakPad;

        int buttonUp;
        int buttonDown;
        int buttonLeft;
        int buttonRight;
        
        int bottonThumbRight;
        int bottonThumbLeft;

        int bottonBackWheelRight;
        int bottonBackWheelLeft;

    };
  
  JoyConfig getJoyConfig(const sensor_msgs::JoyConstPtr &joy)
    {
    JoyConfig config;
    if(joy->buttons.size() > 20)
    {
        config.gearUpLeft = 12;
        config.gearUpMiddle = 14;
        config.gearUpRight = 16;
        config.gearDownLeft = 13;
        config.gearDownMiddle = 15;
        config.gearDownRight = 17;

        config.steeringWheel = 0;
        config.linearVelPedal = 2;
        config.breakPad = 3;

        config.buttonUp = 3;
        config.buttonDown = 0;
        config.buttonLeft = 1;
        config.buttonRight = 2;
    
        config.bottonThumbRight = 4;
        config.bottonThumbLeft = 5;

        config.bottonBackWheelRight = 6;
        config.bottonBackWheelLeft = 7;


    }else if(joy->buttons.size() > 5) 
    {
        config.gearUpLeft = 12;
        config.gearUpMiddle = 14;
        config.gearUpRight = 16;
        config.gearDownLeft = 13;
        config.gearDownMiddle = 15;
        config.gearDownRight = 17;

        config.steeringWheel = 2;//0;
        config.linearVelPedal = 1;//2;
        config.breakPad = 3;

        config.buttonUp = 3;
        config.buttonDown = 0;
        config.buttonLeft = 1;
        config.buttonRight = 2;
    
        config.bottonThumbRight = 4;
        config.bottonThumbLeft = 5;

        config.bottonBackWheelRight = 6;
        config.bottonBackWheelLeft = 7;

    }else
    {
        RobotTeleop::initted_ = false;
        ROS_ERROR_STREAM("proper joy was not find");
    }

    return config;
    }

  JoyConfig model;

public:
  
  RobotTeleop()
    {
    hud_pub_ = nh_.advertise<mobile_robot_teleop::VfomaHud>("vfoma/joy/hud", 1);
    acm_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("vfoma/joy/acm", 1);
    set_pub_ = nh_.advertise<mobile_robot_teleop::VfomaSetting>("vfoma/joy/set", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RobotTeleop::joyCallback, this);
    } 


  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
  {
  
    if(!initted_) 
    {
        
        initted_ = true;
        *lastJoy = *joy;
        model = getJoyConfig(joy);

    }
    
    
    mobile_robot_teleop::VfomaSetting sett;
    ackermann_msgs::AckermannDrive acm;

    
    sett.names.push_back("clearBeta");
    sett.values.push_back(buttonPressed(model.buttonDown, joy, lastJoy) || buttonPressed(model.buttonLeft, joy, lastJoy));

    sett.names.push_back("addTrailer");
    sett.values.push_back(buttonPressed(model.buttonUp, joy, lastJoy));
    
    sett.names.push_back("debug");
    sett.values.push_back(buttonPressed(model.buttonRight, joy, lastJoy));
    
    sett.names.push_back("sigmaUp");
    sett.values.push_back(buttonPressed(model.bottonBackWheelLeft, joy, lastJoy) || buttonPressed(model.bottonThumbLeft, joy, lastJoy));
    
    sett.names.push_back("sigmaDown");
    sett.values.push_back(buttonPressed(model.bottonBackWheelRight, joy, lastJoy) || buttonPressed(model.bottonThumbRight, joy, lastJoy));
    
    if(joy->buttons.size() > 20)
    { 
        if(joy->buttons[model.gearUpLeft] > 0.5)
        {
            hud.direction = 1;
            gearNow = hud.gear = 1;
            dirctionNow = 1;
        } else if(joy->buttons[model.gearUpMiddle] > 0.5)
        {
            hud.direction = 1;
            gearNow = hud.gear = 2;
            dirctionNow = 1;
        } else if(joy->buttons[model.gearUpRight] > 0.5)
        {
            hud.direction = 1;
            gearNow = hud.gear = 3;
            dirctionNow = 1;
        } else if(joy->buttons[model.gearDownLeft] > 0.5)
        {
            hud.direction = 2;
            gearNow = hud.gear = 1;
            dirctionNow = -1;
        } else if(joy->buttons[model.gearDownMiddle] > 0.5)
        {
            hud.direction = 2;
            gearNow = hud.gear = 2;
            dirctionNow = -1;
        } else if(joy->buttons[model.gearDownRight] > 0.5)
        {
            hud.direction = 2;
            gearNow = hud.gear = 3;
            dirctionNow = -1;
        } else 
        {
            hud.direction = 0;
            gearNow = hud.gear = 0;
            dirctionNow = 0;
        }
    } else if(joy->buttons.size() > 5)
    {
        if(buttonPressed(model.bottonBackWheelLeft, joy, lastJoy))
        {
            ++hud.gear;
            ++gearNow;
            
            if(hud.gear > 3)
            {
                hud.gear = 3;
                gearNow = 3;
            } 
            if(hud.gear > 0)
            {
                hud.direction = 1;
                dirctionNow = 1;
            } else if (hud.gear < 0)
            {
                hud.direction = 2;
                dirctionNow = -1;
            }

        }

        if(buttonPressed(model.bottonBackWheelRight, joy, lastJoy))
        {
            --hud.gear;
            --gearNow;

            if(hud.gear < -3)
            {
                hud.gear = -3;
                gearNow = -3;
            } 
            if(hud.gear > 0)
            {
                hud.direction = 1;
                dirctionNow = 1;
            } else if (hud.gear < 0)
            {
                hud.direction = 2;
                dirctionNow = -1;
            }
        }
    } else
    {
        ROS_ERROR_STREAM("proper joy was not find");
    }
    
    
    acm.steering_angle = joy->axes[model.steeringWheel];
    acm.speed = (joy->axes[model.linearVelPedal] / 3) * gearNow * dirctionNow;
    

    acm.steering_angle_velocity = getDiff(model.steeringWheel, joy, lastJoy);
    acm.acceleration = (getDiff(model.linearVelPedal, joy, lastJoy) /3) * gearNow * dirctionNow ;

    //ROS_INFO_STREAM("acc: " << acm.acceleration << " " );


    hud_pub_.publish(hud);
    acm_pub_.publish(acm);
    set_pub_.publish(sett);

    *lastJoy = *joy;
    ROS_INFO_STREAM("my state: I'm publishing xd " );
  }

  bool buttonPressed(int pressedButton, const sensor_msgs::JoyConstPtr &joy, sensor_msgs::Joy* &lastJoy)
  { 
    if(joy->buttons[pressedButton] > 0.5 && lastJoy->buttons[pressedButton] < 0.5 )
    {
        return true;
    } else {
        return false;
    }   
  }
  
  float getDiff(int axes_, const sensor_msgs::JoyConstPtr &joy, sensor_msgs::Joy* &lastJoy) 
  {
    int64_t  t1 = lastJoy->header.stamp.toNSec();
    int64_t  t2 = joy->header.stamp.toNSec();
    float diff = ((joy->axes[axes_]) - (lastJoy->axes[axes_])) / ((t2 -t1) * (10^-9)); 
    //ROS_INFO_STREAM("diff: " << diff << " " );
    return diff;
  }

};

    


int main(int argc, char *argv[])
{
  using namespace sensor_msgs;
  ros::init(argc, argv, "mobile_robot_teleop");
  
  RobotTeleop robot_teleop;
  
  ros::spin();
  
}
    

    



