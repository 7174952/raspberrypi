#include <vector>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"

#include "wiringPi.h"

//sonar spec
#define MIN_DETECT_DIST 0.02   //meter
#define MAX_DETECT_DIST 4.5    //meter
#define SOUND_VIEW_RANGE_RAD  15.0*2*3.14159/180 //30 deg
#define SOUND_SPEED_METER_PER_SEC 340.0          //meter/sec

#define SONAR_LEFT 0
#define SONAR_CENTER 1
#define SONAR_RIGHT 2
#define MAX_SONAR_NUM 3 //used sonar num

/*-----Define Sonar pin---------
 * Left:   trig - 16(gpio_4), echo - 18(gpio_5)
 * Center: trig - 13(gpio_2), echo - 15(gpio_3)
 * Right:  trig - 11(gpio_0), echo - 12(gpio_1)
------------------------------*/
                      /*L   C   R*/
const int trig_pin[] = {4, 2, 0};
const int echo_pin[] = {5, 3, 1};
const bool sonar_en[] = {false, true, false};

#define STATE_NOT_SEND             0
#define STATE_KEEP_TRIG_HIGH       1
#define STATE_WAIT_RESP            2
#define STATE_CHECK_RESP_COMPLETE  3
#define STATE_PUBLISH_DISTANCE     4
#define STATE_COMPlETE             5

#define KEEP_ON_10_USEC  10*1000  //10us
#define WAIT_RESP_TIME_OUT 250*1000000 //250ms


struct Sonar
{
    ros::Publisher dist_pub;
    sensor_msgs::Range msg;
    int16_t state;
    uint64_t send_start_time;
    uint64_t send_complete_time;
    uint64_t resp_start_time;

};

std::vector<struct Sonar> sonar_obst;


void ultraInit()
{
    for(int16_t i = 0; i < MAX_SONAR_NUM; i++)
    {
        if(sonar_en[i] == false) continue;

        pinMode(trig_pin[i], OUTPUT);
        pinMode(echo_pin[i], INPUT);
        digitalWrite(trig_pin[i], LOW);
    }
}

int check_sonar()
{
    for(int16_t i = 0; i < MAX_SONAR_NUM; i++)
    {
        if(sonar_en[i] == false) continue;

        while(1)
        {
            switch (sonar_obst[i].state)
            {
            case STATE_NOT_SEND:
                //send out
                digitalWrite(trig_pin[i], HIGH);
                sonar_obst[i].send_start_time = ros::Time::now().toNSec();
                sonar_obst[i].state = STATE_KEEP_TRIG_HIGH;
                // ROS_INFO("Sonar_%d, State 0: send out",i);
                break;
            case STATE_KEEP_TRIG_HIGH:
                if(ros::Time::now().toNSec() - sonar_obst[i].send_start_time > KEEP_ON_10_USEC)
                {
                    digitalWrite(trig_pin[i], LOW);
                    sonar_obst[i].send_complete_time = ros::Time::now().toNSec();
                    sonar_obst[i].state = STATE_WAIT_RESP;
                    // ROS_INFO("Sonar_%d, STATE 1: send complete",i);
                }
                break;
            case STATE_WAIT_RESP:
                if(digitalRead(echo_pin[i]) == LOW)
                {
                    if(ros::Time::now().toNSec() - sonar_obst[i].send_complete_time > WAIT_RESP_TIME_OUT)
                    {
                        //wait response timeout, set range = +Inf
                        sonar_obst[i].msg.range =  INFINITY;
                        sonar_obst[i].state = STATE_PUBLISH_DISTANCE;
                        // ROS_INFO("Sonar_%d, State 2:wait Timeout!",i);
                    }
                }
                else //low => high
                {
                    //start to receive resp data
                    sonar_obst[i].resp_start_time = ros::Time::now().toNSec();
                    sonar_obst[i].state = STATE_CHECK_RESP_COMPLETE;
                    // ROS_INFO("Sonar_%d, STATE 2: wait resp start",i);
                }
                break;
            case STATE_CHECK_RESP_COMPLETE:
                if(digitalRead(echo_pin[i]) == LOW) //high => low
                {
                    //calculate distance
                    sonar_obst[i].msg.range = (SOUND_SPEED_METER_PER_SEC * (ros::Time::now().toNSec() - sonar_obst[i].resp_start_time)/1000000000) / 2;
                    sonar_obst[i].state = STATE_PUBLISH_DISTANCE;
                    // ROS_INFO("Sonar_%d, STATE 3: wait resp complete",i);
                }
                break;
            case STATE_PUBLISH_DISTANCE:
                sonar_obst[i].msg.header.stamp = ros::Time::now();
                sonar_obst[i].dist_pub.publish(sonar_obst[i].msg);
                sonar_obst[i].state = STATE_COMPlETE;
                // ROS_INFO("Sonar_%d, STATE 4: published distance",i);
                ROS_INFO("Sonar_%d, distance=%f", i, sonar_obst[i].msg.range);
                break;
            case STATE_COMPlETE:
                sonar_obst[i].state = STATE_NOT_SEND;
                // ROS_INFO("Sonar_%d, STATE 5: check sonar complete",i);
                break;
            default:
                sonar_obst[i].state = STATE_NOT_SEND;
                break;
            }

            //check for next sonar
            if(sonar_obst[i].state == STATE_NOT_SEND)
            {
                break; //jump from whhile(1)
            }
        }

    }

    return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sonar");
  ros::NodeHandle nh;

  //Init sonar gpio
  if(wiringPiSetup() == -1)
  {
      ROS_INFO("Sonar Pin setup failed!");
      return -1;
  }
  ultraInit();

  //Init sonar struct
  sonar_obst.resize(MAX_SONAR_NUM);
  for(uint16_t i = 0; i < MAX_SONAR_NUM; i++)
  {
      if(sonar_en[i] == false) continue;

      sonar_obst[i].msg.field_of_view = SOUND_VIEW_RANGE_RAD;
      sonar_obst[i].msg.radiation_type = 0; //0 - sonar, 1 - IR
      sonar_obst[i].msg.max_range = MAX_DETECT_DIST;
      sonar_obst[i].msg.min_range = MIN_DETECT_DIST;
      if(i == SONAR_LEFT)
      {
          sonar_obst[i].msg.header.frame_id = "sonar_frame_left";
          sonar_obst[i].dist_pub = nh.advertise<sensor_msgs::Range>("/scan_sonar_left",10);
      }
      if(i == SONAR_CENTER)
      {
          sonar_obst[i].msg.header.frame_id = "sonar_frame_center";
          sonar_obst[i].dist_pub = nh.advertise<sensor_msgs::Range>("/scan_sonar_center",10);
      }
      if(i == SONAR_RIGHT)
      {
          sonar_obst[i].msg.header.frame_id = "sonar_frame_right";
          sonar_obst[i].dist_pub = nh.advertise<sensor_msgs::Range>("/scan_sonar_right",10);
      }
      sonar_obst[i].state = STATE_NOT_SEND;
  }

  ros::Rate loop_rate(4);
  while (ros::ok())
  {
    check_sonar();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
