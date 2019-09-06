#include <fstream>
#include <ros/ros.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "file_to_path");
  ros::NodeHandle n("~");

  std::string filename;
  n.param("filename", filename, std::string(""));
  
  std::string savedFile;
  n.param("saveto", savedFile, std::string("/home/awong1/timedTrajectory.txt"));
  
  if(filename.empty())
  {
    ROS_ERROR("filename parameter is emtpy");
    return -1;
  }



  std::ifstream infile;
  infile.open(filename);
  double time=0;
  std::ofstream outfile;
  outfile.open(savedFile);
  double x,y,z;
  if(outfile.is_open()){
    if (infile.is_open()){
      while(infile>>x>>y>>z){
	outfile<<time<<" "<<x<<" "<<y<<" "<<z<<std::endl;
	//	std::cout<<time<<x<<y<<z<<std::endl;
	time+=0.01;
      }
      infile.close();
      outfile.close();
      ROS_INFO("File successfully written");
      return 0;

    }
    ROS_ERROR("Couldn't open input file");
    return -1;
  }
  ROS_ERROR("Couldn't open output file");
  return -1;
}
