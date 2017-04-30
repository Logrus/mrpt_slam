#include <mrpt_rbpf_slam/RBPFSlamWrapper.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mrpt_rpbf_slam");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~"); // Parameters node handler
  RBPFSlamWrapper slam(&nh, &pnh);
  // slam.get_param();
  // slam.init();

  ros::Rate r(100);
  // if (!slam.rawlogPlay())
  // {  // if not play from rawlog file

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
}
