/*
 * File: RBPFSlamWrapper.h
 * Author: Vladislav Tananaev
 *
 */

#ifndef MRPT_RBPF_SLAM_WRAPPER_H
#define MRPT_RBPF_SLAM_WRAPPER_H

#include <iosfwd>  // forward declarations for iostream
#include <fstream>   // std::ifstream
#include <string>

// add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// add ros msgs
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
// mrpt msgs
#include <mrpt_msgs/ObservationRangeBeacon.h>
// mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/mrpt_log_macros.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/beacon.h>
#include <mrpt_bridge/time.h>

#if MRPT_VERSION >= 0x130
#include <mrpt/obs/CObservationBeaconRanges.h>
//using namespace mrpt::obs;
#else
#include <mrpt/slam/CObservationBeaconRanges.h>
//using namespace mrpt::slam;
#endif

/**
 * @brief The RBPFSlamWrapper class provides the ROS interfaces for Rao-Blackwellized Particle filter SLAM from MRPT libraries.
 *
 */
class RBPFSlamWrapper
{
public:
  /**
  * @brief constructor
  */
  RBPFSlamWrapper(
    ros::NodeHandle *nh,
    ros::NodeHandle *pnh
    );
  /**
 * @brief destructor
 */
  ~RBPFSlamWrapper();
  /**
 * @brief read the parameters from launch file
 */
  void get_param();
  /**
  * @brief initialize publishers subscribers and RBPF slam
  * TODO : REMOVE, do that in ctor
  */
  void init();
  /**
  * @brief play rawlog file
  *
  * @return true if rawlog file exists and played
  */
  bool rawlogPlay();
  /**
  * @brief publish beacon or grid map and robot pose
  *
  */
  void publishMapPose();

  /**
  * @brief check the existance of the file
  *
  * @return true if file exists
  */
  bool is_file_exists(const std::string& name);

  /**
  * @brief callback function for the beacons
  *
  * Given the range only observation wait for odometry,
  * create the pair of action and observation,
  * implement one SLAM update,
  * publish map and pose.
  *
  * @param msg  the beacon message
  */
  void callbackBeacon(const mrpt_msgs::ObservationRangeBeacon& msg);

  /**
  * @brief callback function for the laser scans
  *
  * Given the laser scans  wait for odometry,
  * create the pair of action and observation,
  * implement one SLAM update,
  * publish map and pose.
  *
  * @param msg  the laser scan message
  */
  void laserCallback(const sensor_msgs::LaserScan& msg);

  /**
   * @brief wait for transform between odometry frame and the robot frame
   *
   * @param des position of the robot with respect to odometry frame
   * @param target_frame the odometry tf frame
   * @param source_frame the robot tf frame
   * @param time timestamp of the observation for which we want to retrieve the position of the robot
   * @param timeout timeout for odometry waiting
   * @param polling_sleep_duration timeout for transform wait
   *
   * @return true if there is transform from odometry to the robot
   */
  bool waitForTransform(mrpt::poses::CPose3D& des,
                        const std::string& target_frame,
                        const std::string& source_frame,
                        const ros::Time& time,
                        const ros::Duration& timeout,
                        const ros::Duration& polling_sleep_duration = ros::Duration(0.01));

  /**
  * @brief get the odometry for the observation received last
  *
  * @param odometry odometry for received observation
  * @param msg_header timestamp of the observation
  */
  void odometryForCallback(const CObservationOdometryPtr& odometry, const std_msgs::Header& msg_header);

  /**
  * @brief  update the pose of the sensor with respect to the robot
  *
  *@param frame_id the frame of the sensors
  */
  void updateSensorPose(const std::string frame_id);

  /**
  * @brief  publis tf tree
  *
  */
  void publishTF();
  /**
  * @brief  correct visualization for ro slam (under development)
  *
  */
  void vizBeacons();


  void init3Dwindow();

  void run3Dwindow();

  /**
   * @brief read ini file
   *
   * @param ini_filename the name of the ini file to read
   */
  void read_iniFile(const std::string ini_filename);

  /**
   * @brief initialize the SLAM
   */
  void init_slam();

  /**
  * @brief read pairs of actions and observations from rawlog file
  *
  * @param data vector of pairs of actions and observations
  * @param rawlog_filename the name of rawlog file to read
  */
  void read_rawlog(std::vector<std::pair<CActionCollection, CSensoryFrame>>& data, std::string rawlog_filename);

  /**
  * @brief calculate the actions from odometry model for current observation
  *
  * @param _sf  current observation
  * @param _odometry raw odometry
  */
  void observation(CSensoryFramePtr sf, CObservationOdometryPtr odometry);

private:
  ros::NodeHandle* nh;         ///< Pointer to node handle managed by main app
  ros::NodeHandle* pnh;        ///< Pointer to parameter node handle managed by main app
  double m_rawlog_play_delay;  ///< delay of replay from rawlog file
  bool m_rawlog_play;          ///< true if rawlog file exists

  std::string m_rawlog_filename;  ///< name of rawlog file
  std::string m_ini_filename;     ///< name of ini file
  std::string m_global_frame_id;  ///< /map frame
  std::string m_odom_frame_id;    ///< /odom frame
  std::string m_base_frame_id;    ///< robot frame

  // Sensor source
  std::string m_sensor_source;  ///< 2D laser scans

  std::map<std::string, mrpt::poses::CPose3D> m_laser_poses;   ///< laser scan poses with respect to the map
  std::map<std::string, mrpt::poses::CPose3D> m_beacon_poses;  ///< beacon poses with respect to the map

  // Subscribers
  std::vector<ros::Subscriber> m_sensorSub;  ///< list of sensors topics

  // read rawlog file
  std::vector<std::pair<CActionCollection, CSensoryFrame>> m_data;  ///< vector of pairs of actions and obsrvations from
                                                                  ///rawlog file

  std::vector<mrpt::opengl::CEllipsoidPtr> m_viz_beacons;

  ros::Publisher m_pub_map;
  ros::Publisher m_pub_metadata;
  ros::Publisher m_pub_particles;
  ros::Publisher m_pub_particles_beacons;
  ros::Publisher m_beacon_viz_pub;  ///< publishers for map and pose particles

  tf::TransformListener m_TFListener;         ///<transform listener
  tf::TransformBroadcaster m_TFBroadcaster;  ///<transform broadcaster

  CTicTac m_tictac;  ///<timer for SLAM performance evaluation
  float m_t_exec;    ///<the time which take one SLAM update execution
  
  CMetricMapBuilderRBPF m_mapBuilder;  ///< map builder
  CActionCollectionPtr m_action;        ///< actions
  CSensoryFramePtr m_sf;                ///< observations

  mrpt::poses::CPose2D m_odomLastObservation;                                  ///< last observation of odometry
  bool m_use_motion_model_default_options;                                     ///< used default odom_params
  CActionRobotMovement2D::TMotionModelOptions m_motion_model_default_options;  ///< used if there are is not odom
  CActionRobotMovement2D::TMotionModelOptions m_motion_model_options;          ///< used with odom value motion noise

  CMetricMapBuilderRBPF::TConstructionOptions m_rbpfMappingOptions;  ///< options for SLAM from ini file
  mrpt::system::TTimeStamp m_timeLastUpdate;                        ///< last update of the pose and map

  const CMultiMetricMap* m_metric_map;  ///<receive map after iteration of SLAM to metric map
  CPose3DPDFParticles m_curPDF;    ///<current robot pose

  mrpt::gui::CDisplayWindow3DPtr m_win3D;  ///<MRPT window
  bool m_camera_3dscene_follows_robot;
  bool m_show_progress_in_window;
  int m_show_progress_in_window_delay_ms;
  int m_progress_window_width, m_progress_window_height;
};

#endif /*MRPT_RBPF_SLAM_WRAPPER_H*/
