
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <mobile_robot_teleop/VfomaHud.h>
#include <mobile_robot_teleop/VfomaSetting.h>
#include <string>
#include <functional>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>


class RobotTeleop
{
private:
  
  bool initted_ = false;

  ros::NodeHandle nh_;
  
  sensor_msgs::Joy lastJoy;
  sensor_msgs::Joy* lastJoyPtr = new sensor_msgs::Joy;

  ros::Publisher hud_pub_, acm_pub_, set_pub_;
  ros::Subscriber joy_sub_;
  
  struct JoyConfig 
    {
        int gearUpLeft;
        int gearUpMidlle;
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
        config.gearUpMidlle = 14;
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
        config.gearUpMidlle = 14;
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
    ROS_INFO_STREAM("beginning of constructor");  
    hud_pub_ = nh_.advertise<mobile_robot_teleop::VfomaHud>("vfoma/joy/hud", 1);
    acm_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("vfoma/joy/acm", 1);
    set_pub_ = nh_.advertise<mobile_robot_teleop::VfomaSetting>("vfoma/joy/set", 1);
    
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RobotTeleop::joyCallback, this);
                                                                 
    ROS_INFO_STREAM("end of Constructor");
  } 


  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
  {
  //ROS_INFO_STREAM("joycallback beginning");
  if(!initted_) 
  {
    //ROS_INFO_STREAM("initiation");
    RobotTeleop::initted_ = true;
    model = getJoyConfig(joy);
    //ROS_INFO_STREAM("model in: " << model.buttonRight << " ");
  }
   // ROS_INFO_STREAM("model out: " << model.buttonRight << " ");
  //ROS_INFO_STREAM("after inittation");
  mobile_robot_teleop::VfomaHud hud;
  mobile_robot_teleop::VfomaSetting sett;
  ackermann_msgs::AckermannDrive acm;
  

  hud.direction = 2;
  hud.gear = 2;
  acm.steering_angle = 2;
  acm.steering_angle_velocity = 2;
  acm.speed = 2;
  
  //if(joy.buttons[model.gearUpLeft] > 0.5 && lastJoy.buttons[model.gearUpLeft])
  //{
    //send msg     
  //}


  //sett.names.push_back("button2");
  //sett.values.push_back(function1());

  //sett.names.push_back("button3");
  //sett.values.push_back(function2());

  hud_pub_.publish(hud);
  acm_pub_.publish(acm);
  set_pub_.publish(sett);

  ROS_INFO_STREAM("joy : " << joy->buttons[model.buttonRight] << " ");
  //ROS_INFO_STREAM("lastJoy: "  << lastJoy.buttons[model.buttonRight] << " ");
  
  
  ROS_INFO_STREAM("lastjoy : " << *lastJoyPtr << " ");
  ROS_INFO_STREAM("joy : " << *joy << " ");
  *lastJoyPtr = *joy;

  //double a = sizeof(sensor_msgs::Joy);
  //char bb[a];
  //strcpy_s(bb, a, joy);
  
  }

};

/*RobotTeleop::RobotTeleop(std::function<getJoyConfig(sensor_msgs::JoyConstPtr)> mapper) : mapper_(mapper): 
{     
  
  
  
}
*/


int main(int argc, char *argv[])
{
  using namespace sensor_msgs;
  ros::init(argc, argv, "mobile_robot_teleop");
  ROS_INFO_STREAM("main loop");
  RobotTeleop robot_teleop;
  ROS_INFO_STREAM("main loop after object");
  ros::spin();
  ROS_INFO_STREAM("after spin");
}
    

    









/*

    
        {
            if(joy.axes.size() <= 3)
            return uCar;
            //change bus type
            

            if( (joy.buttons[0] > 0.5 && lastJoy.buttons[0] < 0.5 && !chooseWayControl) || (joy.buttons[1] > 0.5 && lastJoy.buttons[1] < 0.5 && chooseWayControl) )  //Wheel or Pad
            {
                
            }

            if( (joy.buttons[1] > 0.5 && lastJoy.buttons[1] < 0.5 && !chooseWayControl) || (joy.buttons[0] > 0.5 && lastJoy.buttons[0] < 0.5 && chooseWayControl) )  //Wheel or Pad
            {
               
            }

            if( (joy.buttons[2] > 0.5 && lastJoy.buttons[2] < 0.5 && !chooseWayControl) || (joy.buttons[2] > 0.5 && lastJoy.buttons[2] < 0.5 && chooseWayControl) )  //Wheel or Pad
            {
              
            }

            auto refVel = 0.0;
            auto refSteering = 0.0;
            auto velSig = 0.0;

            //Wheel
            if(!chooseWayControl)
            {
                refVel = ((joy.axes[2] + 1)/2)*carParams.velocityLimit;
                //to let it saturate
                refSteering = joy.axes[0]*carParams.steeringLimit*1.01;
                if(currentVehicleConfiguration.setModeName != "autonomous drive")
                    vehicleConfiguration->actualThrottleValue = (joy.axes[2] + 1.0) * 50.0;

                if(joy.buttons[12] == 1)
                {
                    velSig = 0.5;
                    vehicleConfiguration->direction = 1;
                    vehicleConfiguration->gear = 1;
                }
                else if(joy.buttons[13] == 1)
                {
                    velSig = -0.5;
                    vehicleConfiguration->direction = 2;
                    vehicleConfiguration->gear = 1;
                }
                 else if(joy.buttons[14] == 1)
                {
                    velSig = 2;
                    vehicleConfiguration->direction = 1;
                    vehicleConfiguration->gear = 2;
                }
                else if(joy.buttons[15] == 1)
                {
                    velSig = -2;
                    vehicleConfiguration->direction = 2;
                    vehicleConfiguration->gear = 2;
                }
                else if(joy.buttons[16] == 1)
                {
                    velSig = 4;
                    vehicleConfiguration->direction = 1;
                    vehicleConfiguration->gear = 3;
                }
                else if(joy.buttons[17] == 1)
                {
                    velSig = -4;
                    vehicleConfiguration->direction = 2;
                    vehicleConfiguration->gear = 3;
                }
                else
                {
                    vehicleConfiguration->direction = 0;
                    vehicleConfiguration->gear = 0;
                }
            }

            //Pad
            if(chooseWayControl)
            {
                refVel = (joy.axes[1])*carParams.velocityLimit;
                    if(refVel < 0.0)
                        refVel = 0.0;

                //to let it saturate
                refSteering = joy.axes[2]*carParams.steeringLimit*1.01;
                if(currentVehicleConfiguration.setModeName != "autonomous drive")
                {
                    double velo = joy.axes[1] * 100.0;
                    if(velo < 0.0)
                        velo = 0.0;

                    vehicleConfiguration->actualThrottleValue = velo;
                }

                //Increment gear
                if((joy.buttons[7] == 1) && (lastJoy.buttons[7] == 0))
                {
                    ++actualGear;
                    if(actualGear > 3)
                        actualGear = 3;
                }

                //decrement gear
                if((joy.buttons[6] == 1) && (lastJoy.buttons[6] == 0))
                {
                    --actualGear;
                    if(actualGear < -3)
                        actualGear = -3;
                }

                if(actualGear == -3)
                {
                    velSig = -4;
                    vehicleConfiguration->direction = 2;
                    vehicleConfiguration->gear = 3;
                }
                else if(actualGear == -2)
                {
                    velSig = -2;
                    vehicleConfiguration->direction = 2;
                    vehicleConfiguration->gear = 2;
                }
                else if(actualGear == -1)
                {
                    velSig = -0.5;
                    vehicleConfiguration->direction = 2;
                    vehicleConfiguration->gear = 1;
                }
                else if(actualGear == 0)
                {
                    vehicleConfiguration->direction = 0;
                    vehicleConfiguration->gear = 0;
                }
                else if(actualGear == 1)
                {
                    velSig = 0.5;
                    vehicleConfiguration->direction = 1;
                    vehicleConfiguration->gear = 1;
                }
                else if(actualGear == 2)
                {
                    velSig = 2;
                    vehicleConfiguration->direction = 1;
                    vehicleConfiguration->gear = 2;
                }
                else if(actualGear == 3)
                {
                    velSig = 4;
                    vehicleConfiguration->direction = 1;
                    vehicleConfiguration->gear = 3;
                }
            }

            float maxSigmaParameter = 5.0;

            if( (joy.buttons[7] == 1 && lastJoy.buttons[7] == 0 && !chooseWayControl) || (joy.buttons[5] == 1 && lastJoy.buttons[5] == 0 && chooseWayControl) )
            {
                vehicleConfiguration->sigmaParameter += 0.1;

                if(fabs(vehicleConfiguration->sigmaParameter) <= 0.00001)
                    vehicleConfiguration->sigmaParameter = 0.1;

                if(vehicleConfiguration->sigmaParameter > maxSigmaParameter)
                    vehicleConfiguration->sigmaParameter = maxSigmaParameter;

                vehicleConfigurationOut(vehicleConfiguration);
            }

            if( (joy.buttons[6] == 1 && lastJoy.buttons[6] == 0 && !chooseWayControl) || (joy.buttons[4] == 1 && lastJoy.buttons[4] == 0 && chooseWayControl) )
            {
                vehicleConfiguration->sigmaParameter -= 0.1;

                if(fabs(vehicleConfiguration->sigmaParameter) <= 0.00001)
                    vehicleConfiguration->sigmaParameter = -0.1;

                if(vehicleConfiguration->sigmaParameter < -maxSigmaParameter)
                    vehicleConfiguration->sigmaParameter = -maxSigmaParameter;

                vehicleConfigurationOut(vehicleConfiguration);
            }


            //Compute velocities and beta0c
            ScaledCarInput out;

            out.steeringAngle = sat(refSteering, carParams.steeringLimit);
            out.longitudinalVelocity = velSig*refVel;

            lastJoy = joy;

            return out;
        };



*/