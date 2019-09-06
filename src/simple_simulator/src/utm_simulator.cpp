#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/PointCloud.h"
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

static const int NUM_READINGS_PER_SCAN = 1081;
static const double FREQUENCY = 40.0;
static const double MAX_RANGE_M = 60.0;
static const double MIN_RANGE_M = 0.023;
static const double MAX_ANGLE_RAD = 2.35619449; 
static const double MIN_ANGLE_RAD = -2.35619449; 
static const std::string DEFAULT_FRAME_ID = "laser";
static const std::string DEFAULT_MAP_TOPIC = "/map";
static const std::string DEFAULT_DYNAMIC_OBSTACLE_TOPIC = "/dynamic_obstacles";

static const int OCCUPIED = 100;
static const int FREE = 0;
static const int UNKNOWN = -1;

class UTMSimulatorNode
{
public:
    UTMSimulatorNode();
    void run();


private:
    int value(int index);
    int value(int row, int column);
    void raycast(const geometry_msgs::PointStamped& start, geometry_msgs::PointStamped& end);
    double distance(const geometry_msgs::PointStamped& p1, const geometry_msgs::PointStamped& p2);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void dynamicObstaclesCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    ros::Publisher pub;
    ros::Subscriber map_sub;
    ros::Subscriber dynamic_obstacles_sub;
    sensor_msgs::LaserScan scan;
    tf::TransformListener listener;
    std::string frame_id;
    
    bool map_loaded;
    boost::mutex mutex;
    double resolution;
    double origin_x, origin_y;
    int width, height;
    std::vector<int> static_data;
    std::vector<int> data;
    std::string global_frame_id;
};


UTMSimulatorNode::UTMSimulatorNode():
    map_loaded(false)
{
    ros::NodeHandle pnh("~");

    std::string map_topic, dynamic_obstacles_topic;
    pnh.param<std::string>("frame_id", frame_id, DEFAULT_FRAME_ID);
    pnh.param<std::string>("map_topic", map_topic, DEFAULT_MAP_TOPIC);
    pnh.param<std::string>("dynamic_obstacles_topic", dynamic_obstacles_topic, DEFAULT_DYNAMIC_OBSTACLE_TOPIC);

    pub = pnh.advertise<sensor_msgs::LaserScan>("scan", 20);
    map_sub = pnh.subscribe(map_topic, 1, &UTMSimulatorNode::mapCallback, this);
    dynamic_obstacles_sub = pnh.subscribe(dynamic_obstacles_topic, 1, &UTMSimulatorNode::dynamicObstaclesCallback, this);

    scan.header.frame_id = frame_id;
    scan.angle_min = MIN_ANGLE_RAD;
    scan.angle_max = MAX_ANGLE_RAD;
    scan.angle_increment = (MAX_ANGLE_RAD - MIN_ANGLE_RAD) / (NUM_READINGS_PER_SCAN - 1);
    scan.time_increment = (1.0 / FREQUENCY) / (NUM_READINGS_PER_SCAN);
    scan.range_min = MIN_RANGE_M;
    scan.range_max = MAX_RANGE_M;
    scan.ranges.resize(NUM_READINGS_PER_SCAN);
}

void UTMSimulatorNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
   
    {
      //ROS_ERROR("\n\nIn Map CALLBACK\n\n");
        boost::lock_guard<boost::mutex> lock(mutex);
        resolution = msg->info.resolution;
        origin_x = msg->info.origin.position.x;
        origin_y = msg->info.origin.position.y;
        width = msg->info.width;
        height = msg->info.height;
        global_frame_id = msg->header.frame_id;
        
        int total = width * height;
        static_data.reserve(total);
        static_data.insert(static_data.end(), msg->data.begin(), msg->data.end());
        
        data.reserve(total);
        data.insert(data.end(), static_data.begin(), static_data.end());
        
        map_loaded = true;
    }
}

void UTMSimulatorNode::dynamicObstaclesCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    bool add_dynamic_obstacles = false;
    {
        boost::lock_guard<boost::mutex> lock(mutex);
        add_dynamic_obstacles = map_loaded;
    }

    if(!add_dynamic_obstacles)
        return;
    {
        boost::lock_guard<boost::mutex> lock(mutex);
        data.clear();
        data.insert(data.end(), static_data.begin(), static_data.end());

        for(size_t k = 0; k < msg->points.size(); k++)
        {
            geometry_msgs::Point32 point = msg->points[k];
            int r = (unsigned int)((point.x - origin_x) / resolution);
            int c = (unsigned int)((point.y - origin_y) / resolution);
            int index = r + c * width;
            data[index] = OCCUPIED;
        }
    }
}

int UTMSimulatorNode::value(int index){
  if(index < 0 || index >= (int) data.size()) {
    return UNKNOWN;
  }
  return data[index];
}

int UTMSimulatorNode::value(int i, int j){
  if(j > width - 1 || j < 0 || i > height - 1 || i < 0)
    return UNKNOWN;
  int index = i + j * width;
  return value(index);
}

double UTMSimulatorNode::distance(const geometry_msgs::PointStamped& p1, const geometry_msgs::PointStamped& p2)
{
    double dx = (p1.point.x - p2.point.x);
    double dy = (p1.point.y - p2.point.y);
    return sqrt(dx * dx + dy * dy);
}

void UTMSimulatorNode::raycast(const geometry_msgs::PointStamped& start, geometry_msgs::PointStamped& end)
{

    int r0 = (unsigned int)((start.point.x - origin_x) / resolution);
    int c0 = (unsigned int)((start.point.y - origin_y) / resolution);

    int r1 = (unsigned int)((end.point.x - origin_x) / resolution);
    int c1 = (unsigned int)((end.point.y - origin_y) / resolution);

    int dx = abs(r1 - r0);
    int dy = abs(c1 - c0); 
    int sx = r0 < r1 ? 1 : -1;
    int sy = c0 < c1 ? 1 : -1;
    int err = dx - dy;

    while(true){

        if(value(r0, c0) == OCCUPIED){
            end.point.x = (double) r0 * resolution + origin_x;
            end.point.y = (double) c0 * resolution + origin_y;
            break;
        }

        if(r0 == r1 && c0 == c1){
            end.point.x = (double) r0 * resolution + origin_x;
            end.point.y = (double) c0 * resolution + origin_y;
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy){
          err = err - dy;
          r0 = r0 + sx;
        }

        if(r0 == r1 && c0 == c1){ 
            end.point.x = (double) r0 * resolution + origin_x;
            end.point.y = (double) c0 * resolution + origin_y;
            break;
        }

        if(e2 <  dx){            
          err = err + dx;
          c0 = c0 + sy; 
        }
    }
}

void UTMSimulatorNode::run(){
   
    ros::Rate loop_rate(FREQUENCY);

    ros::Time current_time;

    while (ros::ok())
    {
        bool skip = false;
        
        {
            boost::lock_guard<boost::mutex> lock(mutex);
            if(!map_loaded){
                skip = true;
            }
        }


	//ROS_ERROR("TF: child_frame_id: %s, frame_id_: %s", frame_id.c_str(), global_frame_id.c_str() );
        if(!skip){
            tf::StampedTransform transform;
	    
            try {
                //listener.waitForTransform("base_link", frame_id, ros::Time(0), ros::Duration(10.0) );
                listener.lookupTransform(global_frame_id, frame_id, ros::Time(0), transform);
                current_time =  transform.stamp_;
                //ROS_ERROR("GOT THE TF: child_frame_id: %s, frame_id_: %s, secs: %lf", transform.child_frame_id_.c_str(), transform.frame_id_.c_str(), transform.stamp_.toSec());
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
               skip = true;
            }
        }

        if(!skip){
            scan.header.stamp = current_time;

            //scan.ranges.clear();
            //scan.ranges.resize(NUM_READINGS_PER_SCAN);

            geometry_msgs::PointStamped sensor_in;
            sensor_in.header.stamp = current_time;
            sensor_in.header.frame_id = frame_id;
            sensor_in.point.x = 0;
            sensor_in.point.y = 0;
            sensor_in.point.z = 0;
            geometry_msgs::PointStamped sensor_out;
            listener.transformPoint(global_frame_id, sensor_in, sensor_out);
            //ROS_ERROR("(%lf, %lf) (%lf, %lf)",sensor_in.point.x, sensor_in.point.y, sensor_out.point.x, sensor_out.point.y);
            

            //double a = scan.angle_min;
            double l = scan.range_max;

            #pragma omp parallel for
            for(int i = 0; i < NUM_READINGS_PER_SCAN; i++){
            //while(a <= scan.angle_max){
                double a = scan.angle_min + i * scan.angle_increment;
                geometry_msgs::PointStamped stamped_in;
                stamped_in.header.stamp = current_time;
                stamped_in.header.frame_id = frame_id;
                stamped_in.point.x = l * cos(a);
                stamped_in.point.y = l * sin(a);
                stamped_in.point.z = 0.0;
                geometry_msgs::PointStamped stamped_out;
                listener.transformPoint(global_frame_id, stamped_in, stamped_out);

                raycast(sensor_out, stamped_out);
                double dist = distance(stamped_out, sensor_out);
                
                scan.ranges[i] = dist;
                //scan.ranges.push_back(dist);
                //a += scan.angle_increment;
            }
            pub.publish(scan);
        }
	//else
	//ROS_ERROR("Testing");
	//}

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "utm_simulator");
    UTMSimulatorNode utm_simulator;
    utm_simulator.run();
    return 0;
}
