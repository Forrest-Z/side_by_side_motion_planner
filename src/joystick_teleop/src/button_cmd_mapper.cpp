#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

const static int DEFAULT_BUTTON_ID = 0;
const static std::string DEFAULT_CMD = "echo Hello" ;
const static double DEFAULT_PERIOD = 1;
const static std::string DEFAULT_BUTTON_TOPIC = "/joy";

class ButtonCommandMapper
{
public:
    ButtonCommandMapper();
    void run();

private:
    void execute_command();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::Subscriber joy_sub;    
    int button_id;
    std::string cmd;
    double period;
    ros::Time last_time;
    bool button_is_pressed;
};

ButtonCommandMapper::ButtonCommandMapper():
    button_id(DEFAULT_BUTTON_ID),
    cmd(DEFAULT_CMD),
    period(DEFAULT_PERIOD),
    button_is_pressed(false)
{
    ros::NodeHandle pnh("~");

    std::string button_topic;
    pnh.param("button_topic", button_topic, DEFAULT_BUTTON_TOPIC);
    pnh.param("button_id", button_id, button_id);
    pnh.param("cmd", cmd, cmd);
    pnh.param("period", period, period);

    joy_sub = pnh.subscribe<sensor_msgs::Joy>(button_topic, 1, &ButtonCommandMapper::joyCallback, this); 
    last_time = ros::Time::now();
}

void ButtonCommandMapper::execute_command()
{
    ROS_DEBUG("execute command: %s", cmd.c_str());
    int ret = system(cmd.c_str());
    ROS_DEBUG("return value: %d", ret);
    last_time = ros::Time::now();
}

void ButtonCommandMapper::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(period <= 0)
    {
        if(!button_is_pressed && joy->buttons[button_id] == 1)
        {
            execute_command();
        }
    }
    //else
    //{
    button_is_pressed = joy->buttons[button_id] == 1;
            
    //}
    /*if(joy->buttons[button_id] == 1)
    {
        double delta_time = ros::Time::now().toSec() - last_time.toSec();
        if(delta_time > period)
        {
            execute_command();
        }
    }*/   
}

void ButtonCommandMapper::run()
{
    if(period <= 0)
    {
        ros::spin();
    }
    else
    {
        ros::Duration d = ros::Duration(0.01);
        double delta_time = 0;
        while (ros::ok())
        {
            delta_time = ros::Time::now().toSec() - last_time.toSec();
            if(delta_time > period && button_is_pressed)
            {
                execute_command();
            }
            ros::spinOnce();
            d.sleep();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ButtonCommandMapper");
    ButtonCommandMapper node;
    node.run();
    return 0;
}
