#include <mrpt_rbpf_slam/RBPFSlamWrapper.h>
#include <iostream>

RBPFSlamWrapper::RBPFSlamWrapper()
{
#if MRPT_VERSION>=0x150
#define gausianModel gaussianModel    // a typo was fixed in 1.5.0
#endif

  use_motion_model_default_options_ = false;
  motion_model_default_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
  motion_model_default_options_.gausianModel.minStdXY = 0.10;
  motion_model_default_options_.gausianModel.minStdPHI = 2.0;

  motion_model_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
  motion_model_options_.gausianModel.a1 = 0.034;
  motion_model_options_.gausianModel.a2 = 0.057;
  motion_model_options_.gausianModel.a3 = 0.014;
  motion_model_options_.gausianModel.a4 = 0.097;
  motion_model_options_.gausianModel.minStdXY = 0.005;
  motion_model_options_.gausianModel.minStdPHI = 0.05;

  PROGRESS_WINDOW_WIDTH = 600;
  PROGRESS_WINDOW_HEIGHT = 500;
  SHOW_PROGRESS_IN_WINDOW = false;
  SHOW_PROGRESS_IN_WINDOW_DELAY_MS = 0;
  CAMERA_3DSCENE_FOLLOWS_ROBOT = false;
  
  rawlog_play_ = false;
  mrpt_bridge::convert(ros::Time(0), timeLastUpdate_);
}


RBPFSlamWrapper::~RBPFSlamWrapper()
{
  try {
    std::string sOutMap = "mrpt_rbpf_";
    mrpt::system::TTimeParts parts;
    mrpt::system::timestampToParts(now(), parts, true);
    sOutMap += format("%04u-%02u-%02u_%02uh%02um%02us",
      (unsigned int)parts.year,
      (unsigned int)parts.month,
      (unsigned int)parts.day,
      (unsigned int)parts.hour,
      (unsigned int)parts.minute,
      (unsigned int)parts.second );
    sOutMap += ".simplemap";

    sOutMap = mrpt::system::fileNameStripInvalidChars( sOutMap );
    ROS_INFO("Saving built map to `%s`", sOutMap.c_str());
    m_mapBuilder.saveCurrentMapToFile(sOutMap);
  } catch (std::exception &e) {
    ROS_ERROR("Exception: %s",e.what());
  }
}



bool RBPFSlamWrapper::is_file_exists(const std::string& name)
{
  std::ifstream f(name.c_str());
  return f.good();
}

void RBPFSlamWrapper::get_param()
{
  ROS_INFO("READ PARAM FROM LAUNCH FILE");
  n_.param<double>("rawlog_play_delay", rawlog_play_delay, 0.1);
  ROS_INFO("rawlog_play_delay: %f", rawlog_play_delay);

  n_.getParam("rawlog_filename", rawlog_filename);
  ROS_INFO("rawlog_filename: %s", rawlog_filename.c_str());

  n_.getParam("ini_filename", ini_filename);
  ROS_INFO("ini_filename: %s", ini_filename.c_str());

  n_.param<std::string>("global_frame_id", global_frame_id, "map");
  ROS_INFO("global_frame_id: %s", global_frame_id.c_str());

  n_.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  ROS_INFO("odom_frame_id: %s", odom_frame_id.c_str());

  n_.param<std::string>("base_frame_id", base_frame_id, "base_link");
  ROS_INFO("base_frame_id: %s", base_frame_id.c_str());

  n_.param<std::string>("sensor_source", sensor_source, "scan");
  ROS_INFO("sensor_source: %s", sensor_source.c_str());
}

void RBPFSlamWrapper::init()
{
  // get parameters from ini file
  if (!is_file_exists(ini_filename))
  {
    ROS_ERROR_STREAM("CAN'T READ INI FILE");
    return;
  }
  read_iniFile(ini_filename);
  // read rawlog file if it  exists
  if (is_file_exists(rawlog_filename))
  {
    ROS_WARN_STREAM("PLAY FROM RAWLOG FILE: " << rawlog_filename.c_str());
    read_rawlog(data, rawlog_filename);
    rawlog_play_ = true;
  }

  /// Create publishers///
  // publish grid map
  pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  pub_metadata_ = n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  // robot pose
  pub_Particles_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);
  // ro particles poses
  pub_Particles_Beacons_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud_beacons", 1, true);
  beacon_viz_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/beacons_viz", 1);

  // read sensor topics
  std::vector<std::string> lstSources;
  mrpt::system::tokenize(sensor_source, " ,\t\n", lstSources);
  ROS_ASSERT_MSG(!lstSources.empty(), "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. "
                                      "\"scan\" or \"beacon\")");

  /// Create subscribers///
  sensorSub_.resize(lstSources.size());
  for (size_t i = 0; i < lstSources.size(); i++)
  {
    if (lstSources[i].find("scan") != std::string::npos)
    {
      sensorSub_[i] = n_.subscribe(lstSources[i], 1, &RBPFSlamWrapper::laserCallback, this);
    }
    else
    {
      sensorSub_[i] = n_.subscribe(lstSources[i], 1, &RBPFSlamWrapper::callbackBeacon, this);
    }
  }

  // init slam
  m_mapBuilder = mrpt::slam::CMetricMapBuilderRBPF(rbpfMappingOptions);
  init_slam();
  init3Dwindow();
}

void RBPFSlamWrapper::odometryForCallback(CObservationOdometryPtr& _odometry, const std_msgs::Header& _msg_header)
{
  mrpt::poses::CPose3D poseOdom;
  if (this->waitForTransform(poseOdom, odom_frame_id, base_frame_id, _msg_header.stamp, ros::Duration(1)))
  {
    _odometry = CObservationOdometry::Create();
    _odometry->sensorLabel = odom_frame_id;
    _odometry->hasEncodersInfo = false;
    _odometry->hasVelocities = false;
    _odometry->odometry.x() = poseOdom.x();
    _odometry->odometry.y() = poseOdom.y();
    _odometry->odometry.phi() = poseOdom.yaw();
  }
}

bool RBPFSlamWrapper::waitForTransform(mrpt::poses::CPose3D& des, const std::string& target_frame,
                                     const std::string& source_frame, const ros::Time& time,
                                     const ros::Duration& timeout, const ros::Duration& polling_sleep_duration)
{
  tf::StampedTransform transform;
  try
  {
    listenerTF_.waitForTransform(target_frame, source_frame, time, polling_sleep_duration);
    listenerTF_.lookupTransform(target_frame, source_frame, time, transform);
  }
  catch (tf::TransformException)
  {
    ROS_INFO("Failed to get transform target_frame (%s) to source_frame (%s)", target_frame.c_str(),
             source_frame.c_str());
    return false;
  }
  mrpt_bridge::convert(transform, des);
  return true;
}

void RBPFSlamWrapper::laserCallback(const sensor_msgs::LaserScan& _msg)
{
#if MRPT_VERSION >= 0x130
  using namespace mrpt::maps;
  using namespace mrpt::obs;
#else
  using namespace mrpt::slam;
#endif
  CObservation2DRangeScanPtr laser = CObservation2DRangeScan::Create();

  if (laser_poses_.find(_msg.header.frame_id) == laser_poses_.end())
  {
    updateSensorPose(_msg.header.frame_id);
  }
  else
  {
    mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];
    mrpt_bridge::convert(_msg, laser_poses_[_msg.header.frame_id], *laser);

    m_sf = CSensoryFrame::Create();
    CObservationOdometryPtr odometry;
    odometryForCallback(odometry, _msg.header);

    CObservationPtr obs = CObservationPtr(laser);
    m_sf->insert(obs);
    observation(sf, odometry);
    timeLastUpdate_ = sf->getObservationByIndex(0)->timestamp;

    tictac.Tic();
    m_mapBuilder.processActionObservation(*m_action, *m_sf);
    t_exec = tictac.Tac();
    ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec);
    publishMapPose();
    run3Dwindow();
    publishTF();
  }
}

void RBPFSlamWrapper::callbackBeacon(const mrpt_msgs::ObservationRangeBeacon& _msg)
{
#if MRPT_VERSION >= 0x130
  using namespace mrpt::maps;
  using namespace mrpt::obs;
#else
  using namespace mrpt::slam;
#endif

  CObservationBeaconRangesPtr beacon = CObservationBeaconRanges::Create();
  if (beacon_poses_.find(_msg.header.frame_id) == beacon_poses_.end())
  {
    updateSensorPose(_msg.header.frame_id);
  }
  else
  {
    mrpt_bridge::convert(_msg, beacon_poses_[_msg.header.frame_id], *beacon);

    m_sf = CSensoryFrame::Create();
    CObservationOdometryPtr odometry;
    odometryForCallback(odometry, _msg.header);

    CObservationPtr obs = CObservationPtr(beacon);
    m_sf->insert(obs);
    observation(m_sf, odometry);
    timeLastUpdate_ = m_sf->getObservationByIndex(0)->timestamp;

    tictac.Tic();
    m_mapBuilder.processActionObservation(*m_action, *m_sf);
    t_exec = tictac.Tac();
    ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec);

    publishMapPose();
    run3Dwindow();
  }
}

void RBPFSlamWrapper::publishMapPose()
{
  // if I received new grid maps from 2D laser scan sensors
  metric_map_ = m_mapBuilder.mapPDF.getCurrentMostLikelyMetricMap();
  m_mapBuilder.mapPDF.getEstimatedPosePDF(curPDF);
  if (metric_map_->m_gridMaps.size())
  {
    // publish map
    nav_msgs::OccupancyGrid _msg;
    mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg);
    pub_map_.publish(_msg);
    pub_metadata_.publish(_msg.info);
  }

  // if I received new beacon (range only) map
  if (metric_map_->m_beaconMap)
  {
    mrpt::opengl::CSetOfObjectsPtr objs;

    objs = mrpt::opengl::CSetOfObjects::Create();
    // Get th map as the set of 3D objects
    metric_map_->m_beaconMap->getAs3DObject(objs);

    geometry_msgs::PoseArray poseArrayBeacons;
    poseArrayBeacons.header.frame_id = global_frame_id;
    poseArrayBeacons.header.stamp = ros::Time::now();

    // Count the number of beacons
    int objs_counter = 0;
    while (objs->getByClass<mrpt::opengl::CEllipsoid>(objs_counter))
    {
      objs_counter++;
    }
    poseArrayBeacons.poses.resize(objs_counter);
    mrpt::opengl::CEllipsoidPtr beacon_particle;

    for (size_t i = 0; i < objs_counter; i++)
    {
      beacon_particle = objs->getByClass<mrpt::opengl::CEllipsoid>(i);
      mrpt_bridge::convert(mrpt::poses::CPose3D(beacon_particle->getPose()), poseArrayBeacons.poses[i]);
      viz_beacons.push_back(beacon_particle);
    }
    pub_Particles_Beacons_.publish(poseArrayBeacons);
    vizBeacons();
    viz_beacons.clear();
  }

  // publish pose
  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = global_frame_id;
  poseArray.header.stamp = ros::Time::now();
  poseArray.poses.resize(curPDF.particlesCount());
  for (size_t i = 0; i < curPDF.particlesCount(); i++)
  {
    mrpt::poses::CPose3D p = curPDF.getParticlePose(i);
    mrpt_bridge::convert(p, poseArray.poses[i]);
  }

  pub_Particles_.publish(poseArray);
}

void RBPFSlamWrapper::vizBeacons()
{
  if (viz_beacons.size() == 0)
  {
    return;
  }
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";

  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(1);
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.12;
  marker.scale.y = 0.12;
  marker.scale.z = 0.12;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;

  for (int i = 0; i < viz_beacons.size(); i++)
  {
    CPose3D meanPose(viz_beacons[i]->getPose());
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.pose.position.x = meanPose.x();
    marker.pose.position.y = meanPose.y();
    marker.pose.position.z = meanPose.z();
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    ma.markers.push_back(marker);
    marker.id++;

    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = std::to_string(i);

    marker.pose.position.x = meanPose.x();
    marker.pose.position.y = meanPose.y();
    marker.pose.position.z = meanPose.z() + 0.12;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    // marker.scale.z = 1;
    ma.markers.push_back(marker);
    marker.id++;
  }

  beacon_viz_pub_.publish(ma);
}

void RBPFSlamWrapper::updateSensorPose(std::string _frame_id)
{
  CPose3D pose;
  tf::StampedTransform transform;
  try
  {
    listenerTF_.lookupTransform(base_frame_id, _frame_id, ros::Time(0), transform);

    tf::Vector3 translation = transform.getOrigin();
    tf::Quaternion quat = transform.getRotation();
    pose.x() = translation.x();
    pose.y() = translation.y();
    pose.z() = translation.z();
    double roll, pitch, yaw;
    tf::Matrix3x3 Rsrc(quat);
    CMatrixDouble33 Rdes;
    for (int c = 0; c < 3; c++)
      for (int r = 0; r < 3; r++)
        Rdes(r, c) = Rsrc.getRow(r)[c];
    pose.setRotationMatrix(Rdes);
    laser_poses_[_frame_id] = pose;
    beacon_poses_[_frame_id] = pose;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

bool RBPFSlamWrapper::rawlogPlay()
{
  if (rawlog_play_ == false)
  {
    return false;
  }
  else
  {
    for (int i = 0; i < data.size(); i++)
    {
      if (ros::ok())
      {
        tictac.Tic();
        m_mapBuilder.processActionObservation(data[i].first, data[i].second);
        t_exec = tictac.Tac();
        ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec);

        ros::Duration(rawlog_play_delay).sleep();

        metric_map_ = m_mapBuilder.mapPDF.getCurrentMostLikelyMetricMap();
        m_mapBuilder.mapPDF.getEstimatedPosePDF(curPDF);
        // if I received new grid maps from 2D laser scan sensors
        if (metric_map_->m_gridMaps.size())
        {
          nav_msgs::OccupancyGrid _msg;

          // if we have new map for current sensor update it
          mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg);
          pub_map_.publish(_msg);
          pub_metadata_.publish(_msg.info);
        }

        // if I received new beacon (range only) map
        if (metric_map_->m_beaconMap)
        {
          mrpt::opengl::CSetOfObjectsPtr objs;
          objs = mrpt::opengl::CSetOfObjects::Create();
          metric_map_->m_beaconMap->getAs3DObject(objs);

          geometry_msgs::PoseArray poseArrayBeacons;
          poseArrayBeacons.header.frame_id = global_frame_id;
          poseArrayBeacons.header.stamp = ros::Time::now();

          int objs_counter = 0;
          while (objs->getByClass<mrpt::opengl::CEllipsoid>(objs_counter))
          {
            objs_counter++;
          }
          poseArrayBeacons.poses.resize(objs_counter);
          mrpt::opengl::CEllipsoidPtr beacon_particle;

          for (size_t i = 0; i < objs_counter; i++)
          {
            beacon_particle = objs->getByClass<mrpt::opengl::CEllipsoid>(i);
            mrpt_bridge::convert(mrpt::poses::CPose3D(beacon_particle->getPose()), poseArrayBeacons.poses[i]);
            viz_beacons.push_back(beacon_particle);
          }
          pub_Particles_Beacons_.publish(poseArrayBeacons);
          vizBeacons();
          viz_beacons.clear();
        }

        // publish pose
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = global_frame_id;
        poseArray.header.stamp = ros::Time::now();
        poseArray.poses.resize(curPDF.particlesCount());
        for (size_t i = 0; i < curPDF.particlesCount(); i++)
        {
          mrpt::poses::CPose3D p = curPDF.getParticlePose(i);
          mrpt_bridge::convert(p, poseArray.poses[i]);
        }

        pub_Particles_.publish(poseArray);
      }
      ros::spinOnce();
      run3Dwindow();
    }
  }
  // if there is mrpt_gui it will wait until push any key in order to close the window
  if (win3D)
    win3D->waitForKey();
  return true;
}

void RBPFSlamWrapper::publishTF()
{
  // Most of this code was copy and pase from ros::amcl
  mrpt::poses::CPose3D robotPose;
  m_mapBuilder.mapPDF.getEstimatedPosePDF(curPDF);

  curPDF.getMean(robotPose);

  tf::Stamped<tf::Pose> odom_to_map;
  tf::Transform tmp_tf;
  ros::Time stamp;
  mrpt_bridge::convert(timeLastUpdate_, stamp);
  mrpt_bridge::convert(robotPose, tmp_tf);

  try
  {
    tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), stamp, base_frame_id);
    listenerTF_.transformPose(odom_frame_id, tmp_tf_stamped, odom_to_map);
  }
  catch (tf::TransformException)
  {
    ROS_INFO("Failed to subtract global_frame (%s) from odom_frame (%s)", global_frame_id.c_str(),
             odom_frame_id.c_str());
    return;
  }

  tf::Transform latest_tf_ =
      tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));

  // We want to send a transform that is good up until a
  // tolerance time so that odom can be used

  ros::Duration transform_tolerance_(0.5);
  ros::Time transform_expiration = (stamp + transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration, global_frame_id, odom_frame_id);
  tf_broadcaster_.sendTransform(tmp_tf_stamped);
}






void RBPFSlamWrapper::read_iniFile(std::string ini_filename)
{
  CConfigFile iniFile(ini_filename);
  rbpfMappingOptions.loadFromConfigFile(iniFile, "MappingApplication");
  rbpfMappingOptions.dumpToConsole();

  // Display variables
  CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool("MappingApplication", "CAMERA_3DSCENE_FOLLOWS_ROBOT", true);
  SHOW_PROGRESS_IN_WINDOW = iniFile.read_bool("MappingApplication", "SHOW_PROGRESS_IN_WINDOW", false);
  SHOW_PROGRESS_IN_WINDOW_DELAY_MS = iniFile.read_int("MappingApplication", "SHOW_PROGRESS_IN_WINDOW_DELAY_MS", 1);

  MRPT_LOAD_CONFIG_VAR(PROGRESS_WINDOW_WIDTH, int, iniFile, "MappingApplication");
  MRPT_LOAD_CONFIG_VAR(PROGRESS_WINDOW_HEIGHT, int, iniFile, "MappingApplication");
}

void RBPFSlamWrapper::read_rawlog(std::vector<std::pair<CActionCollection, CSensoryFrame>> &data, std::string rawlog_filename)
{
  size_t rawlogEntry = 0;
  CFileGZInputStream rawlogFile(rawlog_filename);
  CActionCollectionPtr action;
  CSensoryFramePtr observations;

  for (;;)
  {
    if (os::kbhit())
    {
      char c = os::getch();
      if (c == 27)
        break;
    }

    // Load action/observation pair from the rawlog:
    // --------------------------------------------------
    if (!CRawlog::readActionObservationPair(rawlogFile, action, observations, rawlogEntry))
    {
      break;  // file EOF
    }
    data.push_back(std::make_pair(*action, *observations));
  }
}

void RBPFSlamWrapper::observation(CSensoryFramePtr sf, CObservationOdometryPtr odometry)
{
  m_action = CActionCollection::Create();
  CActionRobotMovement2D odom_move;
  odom_move.timestamp = sf->getObservationByIndex(0)->timestamp;

  if (odometry)
  {
    if (odomLastObservation_.empty())
    {
      odomLastObservation_ = odometry->odometry;
    }

    mrpt::poses::CPose2D incOdoPose = odometry->odometry - odomLastObservation_;
    odomLastObservation_ = odometry->odometry;
    odom_move.computeFromOdometry(incOdoPose, motion_model_options_);
    m_action->insert(odom_move);
  }
  else if (use_motion_model_default_options_)
  {
    odom_move.computeFromOdometry(mrpt::poses::CPose2D(0, 0, 0), motion_model_default_options_);
    m_action->insert(odom_move);
  }
}

void RBPFSlamWrapper::init_slam()
{
#if MRPT_VERSION < 0x150
  m_mapBuilder.options.verbose = true;
#else
  log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  m_mapBuilder.setVerbosityLevel(mrpt_bridge::rosLoggerLvlToMRPTLoggerLvl(ros_logger->getLevel()));
  m_mapBuilder.logging_enable_console_output = false;
  m_mapBuilder.logRegisterCallback(static_cast<output_logger_callback_t>(&mrpt_bridge::mrptToROSLoggerCallback));
#endif

  m_mapBuilder.options.enableMapUpdating = true;
  m_mapBuilder.options.debugForceInsertion = false;

  randomGenerator.randomize();
}

void RBPFSlamWrapper::init3Dwindow()
{
#if MRPT_HAS_WXWIDGETS

  if (SHOW_PROGRESS_IN_WINDOW)
  {
    win3D = mrpt::gui::CDisplayWindow3D::Create("RBPF-SLAM @ MRPT C++ Library", PROGRESS_WINDOW_WIDTH,
                                                PROGRESS_WINDOW_HEIGHT);
    win3D->setCameraZoom(40);
    win3D->setCameraAzimuthDeg(-50);
    win3D->setCameraElevationDeg(70);
  }

#endif
}
void RBPFSlamWrapper::run3Dwindow()
{
  // Save a 3D scene view of the mapping process:
  if (SHOW_PROGRESS_IN_WINDOW && win3D.present())
  {
    // get the current map and pose
    metric_map_ = m_mapBuilder.mapPDF.getCurrentMostLikelyMetricMap();
    m_mapBuilder.mapPDF.getEstimatedPosePDF(curPDF);
    COpenGLScenePtr scene;
    scene = COpenGLScene::Create();

    // The ground:
    mrpt::opengl::CGridPlaneXYPtr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200, 200, -200, 200, 0, 5);
    groundPlane->setColor(0.4, 0.4, 0.4);
    scene->insert(groundPlane);

    // The camera pointing to the current robot pose:
    if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
    {
      mrpt::opengl::CCameraPtr objCam = mrpt::opengl::CCamera::Create();
      CPose3D robotPose;
      curPDF.getMean(robotPose);

      objCam->setPointingAt(robotPose);
      objCam->setAzimuthDegrees(-30);
      objCam->setElevationDegrees(30);
      scene->insert(objCam);
    }
    // Draw the map(s):
    mrpt::opengl::CSetOfObjectsPtr objs = mrpt::opengl::CSetOfObjects::Create();
    metric_map_->getAs3DObject(objs);
    scene->insert(objs);

    // Draw the robot particles:
    size_t M = m_mapBuilder.mapPDF.particlesCount();
    mrpt::opengl::CSetOfLinesPtr objLines = mrpt::opengl::CSetOfLines::Create();
    objLines->setColor(0, 1, 1);
    for (size_t i = 0; i < M; i++)
    {
      std::deque<TPose3D> path;
      m_mapBuilder.mapPDF.getPath(i, path);

      float x0 = 0, y0 = 0, z0 = 0;
      for (size_t k = 0; k < path.size(); k++)
      {
        objLines->appendLine(x0, y0, z0 + 0.001, path[k].x, path[k].y, path[k].z + 0.001);
        x0 = path[k].x;
        y0 = path[k].y;
        z0 = path[k].z;
      }
    }
    scene->insert(objLines);

    // An ellipsoid:
    CPose3D lastMeanPose;
    float minDistBtwPoses = -1;
    std::deque<TPose3D> dummyPath;
    m_mapBuilder.mapPDF.getPath(0, dummyPath);
    for (int k = (int)dummyPath.size() - 1; k >= 0; k--)
    {
      CPose3DPDFParticles poseParts;
      m_mapBuilder.mapPDF.getEstimatedPosePDFAtTime(k, poseParts);
      CPose3D meanPose;
      CMatrixDouble66 COV;
      poseParts.getCovarianceAndMean(COV, meanPose);

      if (meanPose.distanceTo(lastMeanPose) > minDistBtwPoses)
      {
        CMatrixDouble33 COV3 = COV.block(0, 0, 3, 3);

        minDistBtwPoses = 6 * sqrt(COV3(0, 0) + COV3(1, 1));

        opengl::CEllipsoidPtr objEllip = opengl::CEllipsoid::Create();
        objEllip->setLocation(meanPose.x(), meanPose.y(), meanPose.z() + 0.001);
        objEllip->setCovMatrix(COV3, COV3(2, 2) == 0 ? 2 : 3);

        objEllip->setColor(0, 0, 1);
        objEllip->enableDrawSolid3D(false);
        scene->insert(objEllip);

        lastMeanPose = meanPose;
      }
    }

    COpenGLScenePtr &scenePtr = win3D->get3DSceneAndLock();
    scenePtr = scene;
    win3D->unlockAccess3DScene();
    win3D->forceRepaint();
  }
}
