#ifndef __PARTICLEFILTER_HPP__
#define __PARTICLEFILTER_HPP__

#include <vector>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <time.h>
#include <math.h>
#include <sewer_graph/sewer_graph.h>
#include <fstream>
#include <plane_detector/WallInfo.h>
#include <cmath>

//Struct that contains the data concerning one Particle
struct Particle
{
  //Position
  float x;
  float y;
  
  // Yaw angle (with respect to MAP frame)
  float a;

  // Weight (One weight)
  float w;
};

//Class definition
class ParticleFilter
{
public:

  //!Default contructor 
  ParticleFilter(std::string &node_name) : 
  m_randVar(boost::mt19937(time(0)), boost::normal_distribution<>(0, 0.4)), s_g(NULL)
  {
    // Setup random number generator from GSL
    gsl_rng_env_setup();
    m_randomType = gsl_rng_default;
    m_randomValue = gsl_rng_alloc(m_randomType);
    
    // Read node parameters
    ros::NodeHandle lnh("~");
    if(!lnh.getParam("base_frame_id", m_baseFrameId))
      m_baseFrameId = "base_link";  
    if(!lnh.getParam("odom_frame_id", m_odomFrameId))
      m_odomFrameId = "odom";  
    if(!lnh.getParam("global_frame_id", m_globalFrameId))
      m_globalFrameId = "map";  
    
    m_lastPoseCov.header.frame_id = m_globalFrameId;

    // Read amcl parameters
    if(!lnh.getParam("update_rate", m_updateRate))
      m_updateRate = 10.0;
    if(!lnh.getParam("min_particles", m_minParticles))
      m_minParticles = 300;
    if(!lnh.getParam("max_particles", m_maxParticles))
      m_maxParticles = 600;
    if(m_minParticles > m_maxParticles)
      m_maxParticles = m_minParticles;
    if(!lnh.getParam("odom_x_mod", m_odomXMod))
      m_odomXMod = 0.2;
    if(!lnh.getParam("odom_y_mod", m_odomYMod))
      m_odomYMod = 0.2;
    if(!lnh.getParam("odom_a_mod", m_odomAMod))
      m_odomAMod = 0.2;
    if(!lnh.getParam("odom_a_noise", m_odomANoise))
      m_odomANoise = 0.05;
    if(!lnh.getParam("initial_x", m_initX))
      m_initX = 0.0;
    if(!lnh.getParam("initial_y", m_initY))
      m_initY = 0.0;
    if(!lnh.getParam("initial_a", m_initA))
      m_initA = 0.0;  
    if(!lnh.getParam("initial_x_dev", m_initXDev))
      m_initXDev = 0.3;
    if(!lnh.getParam("initial_y_dev", m_initYDev))
      m_initYDev = 0.3;
    if(!lnh.getParam("initial_a_dev", m_initADev))
      m_initADev = 0.2;  
    if(!lnh.getParam("update_min_d", m_dTh))
      m_dTh = 0.5;
    if(!lnh.getParam("update_min_a", m_aTh))
      m_aTh = 0.2;
    if(!lnh.getParam("resample_interval", m_resampleInterval))
      m_resampleInterval = 0;
    // Get the parameters related with the sewer
    if (!lnh.getParam("detect_manhole_topic", m_detectManholeTopic))
      m_detectManholeTopic = "/manhole";
    
    if (!lnh.getParam("angular_weight", angular_weight))
      angular_weight = 5.0;
      
    
    if (!lnh.getParam("sewer_graph_file", m_sewer_graph_file))
      m_sewer_graph_file = "/home/siar/siar_ws/src/siar_packages/localization_siar/sewer_graph/test/sewer_graph_1";
    
    if (!lnh.getParam("min_manhole_detected", m_min_manhole_detected))
      m_min_manhole_detected = 10;
    if (!lnh.getParam("edgeDev", m_edgeDev))
      m_edgeDev = 1.0;
    
    edgeConst1 = 1./(m_edgeDev*sqrt(2*M_PI));
    edgeConst2 = 1./(2*m_edgeDev*m_edgeDev);
    
    if (!lnh.getParam("forkDev", m_forkDev))
      m_forkDev = 3.0;
    if (!lnh.getParam("fork_dist", m_fork_dist))
      m_fork_dist = 3.0;
    
    forkConst1 = 1./(m_forkDev*sqrt(2*M_PI));
    forkConst2 = 1./(2*m_forkDev*m_forkDev);
    
    if (!lnh.getParam("manholeDev", m_manholeDev))
      m_manholeDev = 0.6;
    if (!lnh.getParam("manholeThres", m_manholeThres))
      m_manholeThres = 0.15;
    
    manholeConst1 = 1./(m_manholeDev*sqrt(2*M_PI)); 
    manholeConst2 = 1./(2*m_manholeDev*m_manholeDev);
    
    if (!lnh.getParam("angleDev", m_angleDev))
      m_angleDev = 0.03;
    angleConst1 = 1./(m_angleDev*sqrt(2*M_PI)); 
    angleConst2 = 1./(2*m_angleDev*m_angleDev);
    
    // Save trajectory
    traj_file_open = false;
    if (!lnh.getParam("traj_file", traj_filename))
      traj_filename = "~/traj_.txt";
    try {
      traj_file.open(traj_filename.c_str());
      traj_file_open = true;
    }catch (std::exception &e) {
      std::cerr << "Could not open the traj file: " << traj_filename << std::endl;
    }
    
    // Init the particle filter internal stuff
    m_nUpdates = 0;
    m_init = false;
    m_doUpdate = false;
    m_p.resize(m_maxParticles);
    
    // Launch subscribers
    m_detect_manhole_Sub = m_nh.subscribe(m_detectManholeTopic, 1, &ParticleFilter::manholeDetectedCallback, this);
    m_initialPoseSub = m_nh.subscribe(node_name+"/initial_pose", 2, &ParticleFilter::initialPoseReceived, this);
    wall_info_sub = m_nh.subscribe("/wall_info", 1, &ParticleFilter::wallInfoCallback, this);
    
    // Launch publishers
    m_posesPub = m_nh.advertise<geometry_msgs::PoseArray>(node_name+"/particle_cloud", 1, true);
    m_posecovPub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(node_name+"/estimated_pose", 1, true);
    m_graphPub = m_nh.advertise<visualization_msgs::Marker>(node_name+"/sewer_graph", 0, true);
    m_gpsPub = m_nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 2, true);

    // Launch updater timer
    updateTimer = m_nh.createTimer(ros::Duration(1.0/m_updateRate), &ParticleFilter::checkUpdateThresholdsTimer, this);
    
    // Initialize TF from odom to map as identity
    m_lastGlobalTf.setIdentity();
    
    // Initialize sewer graph
    s_g = new sewer_graph::SewerGraph(m_sewer_graph_file);
    
    // PUblish gps reference position
    ROS_INFO("Publishing gps position: %f, %f", s_g->getReferencePosition().latitude, s_g->getReferencePosition().longitude);
    m_gpsPub.publish(s_g->getReferencePosition());
    
    // Initialize filter if requested
    bool initialize = false;
    if (!lnh.getParam("initialize", initialize)) {
      initialize = false;
    }
    if (initialize)
    {
      tf::Pose pose;
      tf::Vector3 v(m_initX, m_initY, 0.0);
      pose.setOrigin(v);
      pose.setRotation(tf::createQuaternionFromYaw(m_initA));
      ROS_INFO("Setting pose: %.3f %.3f %.3f", pose.getOrigin().x(), pose.getOrigin().y(), getYawFromTf(pose));
      setInitialPose(pose, m_initXDev, m_initYDev, m_initADev);
    }
    last_info.angle = 1e100;
  }

  //!Default destructor
  ~ParticleFilter()
  {
    gsl_rng_free(m_randomValue);
    delete s_g;
  }
    
  //! Check motion and time thresholds for AMCL update
  bool checkUpdateThresholds()
  {
    // Publish current TF from odom to map
    m_tfBr.sendTransform(tf::StampedTransform(m_lastGlobalTf, ros::Time::now(), m_globalFrameId, m_odomFrameId));
    
    // If the filter is not initialized then exit
    if(!m_init)
      return false;
//     std::cout << "Checking for AMCl sewer for update" << std::endl;
          
    // Compute odometric trasnlation and rotation since last update 
    tf::StampedTransform odomTf;
    try
    {
      m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
      m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("AMCL3D error: %s",ex.what());
      return false;
    }
    tf::Transform T = m_lastOdomTf.inverse()*odomTf;
    
    // Check translation threshold
    if(T.getOrigin().length() > m_dTh)
    {
      ROS_INFO("Translation update");
      m_doUpdate = true;
    }
    
    // Check yaw threshold
    double yaw, pitch, roll;
    T.getBasis().getRPY(roll, pitch, yaw);
    if(fabs(yaw) > m_dTh)
    {
      ROS_INFO("Rotation update");
      m_doUpdate = true;
    }
    
    if (m_doUpdate) {
      update(checkManhole());
    }
    
    return false;
  }
  
  visualization_msgs::Marker getPosMarker(const std::string &ref_frame, int id = 3, double scale = 3.0) {
    float mX, mY, mA, devX, devY, devA;
    computeDev(mX, mY, mA, devX, devY, devA);
    visualization_msgs::Marker pos;
    pos.header.frame_id = ref_frame;
    pos.header.stamp = ros::Time::now();
    pos.ns = "sewer_graph";
    pos.action = visualization_msgs::Marker::ADD;
    pos.pose.orientation.w = 1.0;
    pos.id = id;
    pos.type = visualization_msgs::Marker::SPHERE_LIST;
    pos.color.b = 1.0;
    pos.color.a = 1.0;
    pos.scale.x = scale;
    pos.scale.y = scale;
    pos.scale.z = scale;
    
    geometry_msgs::Point p;
    p.x = mX;
    p.y = mY;
    p.z = 0.0;
    
    
    pos.points.push_back(p);
    return pos;
  }
  
  void publishParticles()
  {
    static int seq = 0;

    // If the filter is not initialized then exit
    if(!m_init)
      return;
      
    // Build the msg based on the particles position and orinetation  
    geometry_msgs::PoseArray particlesMsg;
    particlesMsg.header.stamp = ros::Time::now();
    particlesMsg.header.frame_id = m_globalFrameId;
    particlesMsg.header.seq = seq++;
    particlesMsg.poses.resize(m_p.size());
    for(int i=0; i<m_p.size(); i++)
    {
      particlesMsg.poses[i].position.x = m_p[i].x;
      particlesMsg.poses[i].position.y = m_p[i].y;
      particlesMsg.poses[i].position.z = 0.0;
      particlesMsg.poses[i].orientation.x = 0.0;
      particlesMsg.poses[i].orientation.y = 0.0;
      particlesMsg.poses[i].orientation.z = sin(m_p[i].a*0.5);
      particlesMsg.poses[i].orientation.w = cos(m_p[i].a*0.5);
    }
    
    // Publisg particle cloud
    m_posesPub.publish(particlesMsg);
  }
                                       
private:
  
  inline bool isFork() {
    float x, y, z, xDev, yDev, aDev;
    computeDev(x, y, z, xDev, yDev, aDev);
    return isFork(x,y);
  }
  
  inline bool isFork(double x, double y) {
    return (s_g->getDistanceToClosestVertex(x, y, sewer_graph::FORK) < m_fork_dist);
  }

  void checkUpdateThresholdsTimer(const ros::TimerEvent& event)
  {
    checkUpdateThresholds();
    
    // Publish markers
    std::vector<visualization_msgs::Marker> markers = s_g->getMarkers(m_globalFrameId);
    for (unsigned int i = 0; i < markers.size();i++) {
      m_graphPub.publish(markers[i]);
      
    }
    m_graphPub.publish(getPosMarker(m_globalFrameId));
  }

  void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
  {
    // We only accept initial pose estimates in the global frame
    if(msg->header.frame_id != m_globalFrameId)
    {
      ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
      msg->header.frame_id.c_str(),
      m_globalFrameId.c_str());
      return;  
    }
    
    // Transform into the global frame
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    double x = pose.getOrigin().x();
    double y = pose.getOrigin().y();
    int i,j;
    
    // Logging the pose change
    ROS_INFO("Setting pose: %.3f %.3f %.3f", pose.getOrigin().x(), pose.getOrigin().y(), getYawFromTf(pose));
    ROS_INFO("Distance to closest Edge = %f", s_g->getDistanceToClosestEdge(x , y, i, j));
    ROS_INFO("Vertices of the closest Edge = %d, %d", i,j);
    ROS_INFO("Distance to closest Manhole = %f", s_g->getDistanceToClosestManhole(x, y));
    
    // Initialize the filter
    setInitialPose(pose, m_initXDev, m_initYDev, m_initADev);
  }

  //! 3D point-cloud callback
  void manholeDetectedCallback(const std_msgs::BoolConstPtr& msg)
  {    
    manhole_hist.push_back(msg->data);
  }
  
  void wallInfoCallback(const plane_detector::WallInfoConstPtr &msg) {
    last_info = *msg;
    last_relative_time = ros::Time::now();
    ROS_INFO("Catched angle measurement. Angle = %f", msg->angle);
  }
  
  void update(bool detected_manhole) {
    
    if(!predictParticles())
    {
      ROS_ERROR("ParticleFilterSewer::manholeDetectedCallback --> Prediction error!");
      return;
    }
    
    int mode = 0;
    
      
    float x,y,a,xv,yv,av,xycov;
    computeVar(x,y,a,xv,yv,av,xycov);
    
    if (traj_file_open) {
      traj_file << x << "\t" << y << "\t" << ros::Time::now().sec << "." << ros::Time::now().nsec<<  std::endl;
    }
    
    // Perform different particle update based on current point-cloud and available measurements
    if (detected_manhole) {
      ROS_INFO("Performing Update with Manhole");
      updateParticles(1);
    }
    else if (isFork(x,y)) {
      ROS_INFO("Performing update with fork");
      updateParticles(2);
    }
    else {
      if (last_relative_time - ros::Time::now() < ros::Duration(0,200000000L) && fabs(last_info.angle) < 5 && last_info.d_left > 0.0) {
	ROS_INFO("Performing angular update");
	updateParticles(4); // 4 is for  angular + edge
      } else {
	ROS_INFO("Performing regular update");
	updateParticles(0);
      }
    }
    
    // Re-compute global TF according to new weight of samples
    computeGlobalTfAndPose();

    //Do the resampling if needed
    m_nUpdates++;
    if(m_nUpdates > m_resampleInterval)
    {
      m_nUpdates = 0;
      resample();
    }
    
    /*wt = 0; NOTE: this was the implementation of importance resampling
    for(int i=0; i<(int)m_p.size(); i++)
      wt += m_p[i].w*m_p[i].w;
    float nEff = 1/wt;
    if(nEff < ((float)m_p.size())/10.0)
      resample();*/
    //resample();
      
    m_doUpdate = false;
            
    // Publish particles
    publishParticles();
    m_posecovPub.publish(m_lastPoseCov);
  }

  //!This function implements the PF prediction stage. Translation in X, Y and Z 
  //!in meters and yaw angle incremenet in rad
  //! TODO: In a first stage the predict stage will remain equal to the amcl_3d but without the z
  bool predictParticles()
  {
    // Compute odometric trasnlation and rotation since last update 
    tf::StampedTransform odomTf;
    try
    {
      m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
      m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return false;
    }
    
    // Extract current robot roll and pitch
    double yaw, r, p;
    odomTf.getBasis().getRPY(r, p, yaw);
    
    // Perform particle prediction based on odometry
    float delta_x, delta_y;
    double delta_r, delta_p, delta_a;
    tf::Transform T = m_lastOdomTf.inverse()*odomTf;
    delta_x = T.getOrigin().getX();
    delta_y = T.getOrigin().getY();
    T.getBasis().getRPY(delta_r, delta_p, delta_a);
    
    float xDev, yDev, aDev;
    xDev = fabs(delta_x*m_odomXMod);
    yDev = fabs(delta_y*m_odomYMod);
    aDev = fabs(delta_a*m_odomAMod) + fabs(m_odomANoise); 
    
    //Make a prediction for all particles according to the odometry
    for(int i=0; i<(int)m_p.size(); i++)
    {
      float sa = sin(m_p[i].a);
      float ca = cos(m_p[i].a);
      float randX = delta_x + gsl_ran_gaussian(m_randomValue, xDev);
      float randY = delta_y + gsl_ran_gaussian(m_randomValue, yDev);
      m_p[i].x += ca*randX - sa*randY;
      m_p[i].y += sa*randX + ca*randY;
      m_p[i].a += delta_a + gsl_ran_gaussian(m_randomValue, aDev);
    }
    
    m_lastOdomTf = odomTf;
    
    return true;
  }
  
  
  
  // Update Particles taking into account
  // No input is necessary --> we will get the closest Manhole, which will be used for weighting purposes(with some dispersion)
  void updateParticles(int mode)
  {  
    double wt = 0.0;
    
    for(int i=0; i<(int)m_p.size(); i++)
    {
      double tx = m_p[i].x;
      double ty = m_p[i].y;
      double ta = m_p[i].a;
      // Evaluate the weight of the range sensors
      
      switch (mode) {
	case 1:
	  m_p[i].w = computeManholeWeight(tx, ty);
	  break;
	case 2:
	  m_p[i].w = computeForkWeight(tx, ty);
	  break;
	case 3:
	  m_p[i].w = computeAngularWeight(tx, ty, ta);
	  break;
	case 4:
	  m_p[i].w = computeEdgeWeight(tx, ty);
	  m_p[i].w += angular_weight * computeAngularWeight(tx, ty, ta);
	  break;
	default:
	  m_p[i].w = computeEdgeWeight(tx, ty); // Compute weight as a function of the distance to the closest edge
      }
        
      //Increase the summatory of weights
      wt += m_p[i].w;
    }
    
    //Normalize all weights
    wt = 1.0 / wt;
    for(int i=0; i<(int)m_p.size(); i++)
    {
      m_p[i].w *= wt;  
    }  
  }
  
  double computeEdgeWeight(double x, double y) {
    int i,j;
    double dist = s_g->getDistanceToClosestEdge(x,y,i, j);
    
    return edgeConst1*exp(-dist*dist*edgeConst2);
  }
  
  double computeForkWeight(double x, double y) {
    int i,j;
    double dist = s_g->getDistanceToClosestEdge(x,y,i,j);
    return forkConst1*exp(-dist*dist*forkConst2);
    
  }
  
  double computeManholeWeight(double x, double y) {
    double dist = s_g->getDistanceToClosestManhole(x, y);
    return manholeConst1*exp(-dist*dist*manholeConst2) + m_manholeThres;
  }
  
  double computeAngularWeight(double tx, double ty, double ta) {
    double ret = 0.0;
    double man_angle_1 = s_g->getClosestEdgeAngle(tx, ty);
    double rel_angle = ta - man_angle_1;
    
    while (fabs(rel_angle) > M_PI / 2) {
      rel_angle -= M_PI * ((std::signbit(rel_angle))? -1.0:1.0);
    }
    
    ROS_INFO("ComputeAngularWeight: Estimated rel angle = %f\t Rel angle from particle = %f", last_info.angle, rel_angle);
    
    double angular_error = rel_angle + last_info.angle;
    
    ret = angleConst1*exp(-angular_error*angular_error*angleConst2);
      
    return ret;
  }

  //! Set the initial pose of the particle filter
  void setInitialPose(tf::Pose initPose, float xDev, float yDev, float aDev)
  {
    // Resize particle set
    m_p.resize(m_maxParticles);
    
    // Sample the given pose
    tf::Vector3 t = initPose.getOrigin();
    float a = getYawFromTf(initPose);
    float dev = std::max(xDev, yDev);
    float gaussConst1 = 1./(dev*sqrt(2*M_PI));
    float gaussConst2 = 1./(2*dev*dev);
    float dist = 0.0, wt = 0.0;
    m_p[0].x = t.x();
    m_p[0].y = t.y();
    m_p[0].a = a;
    m_p[0].w = gaussConst1;
    wt = m_p[0].w;
    for(int i=1; i<(int)m_p.size(); i++)
    {
      m_p[i].x = t.x() + gsl_ran_gaussian(m_randomValue, xDev);
      m_p[i].y = t.y() + gsl_ran_gaussian(m_randomValue, yDev);
      m_p[i].a = a + gsl_ran_gaussian(m_randomValue, aDev);
      dist = sqrt((m_p[i].x - t.x())*(m_p[i].x - t.x()) + (m_p[i].y - t.y())*(m_p[i].y - t.y()) );
      m_p[i].w = gaussConst1*exp(-dist*dist*gaussConst2);
      wt += m_p[i].w;
    }
    for(int i=0; i<(int)m_p.size(); i++)
      m_p[i].w /= wt;
        
    // Extract TFs for future updates
    bool initialized = false;
    while (!initialized) {
      try
      {
        m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(15.0));
        m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), m_lastOdomTf);
        initialized = true;
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }
      sleep(2);
    }
    computeGlobalTfAndPose();
    m_doUpdate = false;
    m_init = true;
    
    // Publish particles
    publishParticles();
  }
  
  //! Return yaw from a given TF
  float getYawFromTf(tf::Pose& pose)
  {
    double yaw, pitch, roll;
    
    pose.getBasis().getRPY(roll, pitch, yaw);
    
    return (float)yaw;
  }

  //! resample the set of particules using low-variance sampling
  int resample()
  {
    int i, m;
    float r, u, c, factor;
    std::vector<Particle> newP;

    //Initialize data and vectors
    newP.resize(m_p.size());
    factor = 1.0/((float)m_p.size());
    i = 0;
    c = m_p[0].w;
    r = factor * gsl_rng_uniform(m_randomValue);

    //Do resamplig
    for(m=0; m<m_p.size(); m++)
    {
      u = r + factor*(float)m;
      while(u > c)
      {
        i++;
        c += m_p[i].w;
      }
      newP[m] = m_p[i];
      newP[m].w = factor;
    }
    
    //Asign the new particles set
    m_p = newP;

    return 0;
  }
  
  // Computes TF from odom to global frames
  void computeGlobalTfAndPose()
  {        
    // Compute mean value from particles
    float mx, my, ma, xv, yv, av, xycov;
    computeVar(mx, my, ma, xv, yv, av, xycov);
    Particle p;
    p.x = mx;
    p.y = my;
    p.a = ma;
    p.w = 0.0;
    
    // Compute the TF from odom to global
    std::cout << "New TF:\n\t" << p.x << ", " << p.y << std::endl;
    m_lastGlobalTf = tf::Transform(tf::Quaternion(0.0, 0.0, sin(p.a*0.5), cos(p.a*0.5)), tf::Vector3(p.x, p.y, 0.0))*m_lastOdomTf.inverse();
    m_lastPoseCov.header.stamp = ros::Time::now(); 
    m_lastPoseCov.header.seq++;
    geometry_msgs::Quaternion &orientation = m_lastPoseCov.pose.pose.orientation;
    orientation.w = m_lastGlobalTf.getRotation().getW();
    orientation.x = m_lastGlobalTf.getRotation().getX();
    orientation.y = m_lastGlobalTf.getRotation().getY();
    orientation.z = m_lastGlobalTf.getRotation().getZ();
    geometry_msgs::Point &position = m_lastPoseCov.pose.pose.position;
    position.x = mx; position.y = my; position.z = 0.0;
    boost::array<double, 36> &Cov = m_lastPoseCov.pose.covariance;
    Cov[0] = xv; Cov[7] = yv; Cov[35] = av;
    Cov[1] = Cov[6] = xycov;
  }
  
  void computeVar(float &mX, float &mY, float &mA, float &var_x, float &var_y, float &var_a, float &cov_x_y) {
    // Compute mean value from particles
    var_x = mX = 0.0;
    var_y = mY = 0.0;
    var_a = mA = 0.0;
    cov_x_y = 0.0;
    for(int i=0; i<m_p.size(); i++)
    {
      mX += m_p[i].w * m_p[i].x;
      mY += m_p[i].w * m_p[i].y;
      mA += m_p[i].w * m_p[i].a;
    }
    for(int i=0; i<m_p.size(); i++)
    {
      var_x += m_p[i].w * (m_p[i].x-mX) * (m_p[i].x-mX);
      var_y += m_p[i].w * (m_p[i].y-mY) * (m_p[i].y-mY);
      var_a += m_p[i].w * (m_p[i].a-mA) * (m_p[i].a-mA);
      cov_x_y += m_p[i].w * (m_p[i].x-mX) * (m_p[i].y-mY);
      
    }
  }
  
  void computeDev(float &mX, float &mY, float &mA, float &devX, float &devY, float &devA)
  {        
    // Compute mean value from particles
    devX = mX = 0.0;
    devY = mY = 0.0;
    devA = mA = 0.0;
    for(int i=0; i<m_p.size(); i++)
    {
      mX += m_p[i].w * m_p[i].x;
      mY += m_p[i].w * m_p[i].y;
      mA += m_p[i].w * m_p[i].a;
    }
    for(int i=0; i<m_p.size(); i++)
    {
      devX += m_p[i].w * (m_p[i].x-mX) * (m_p[i].x-mX);
      devY += m_p[i].w * (m_p[i].y-mY) * (m_p[i].y-mY);
      devA += m_p[i].w * (m_p[i].a-mA) * (m_p[i].a-mA);
    }
    devX = sqrt(devX);
    devY = sqrt(devY);
    devA = sqrt(devA);
  }
  
  bool checkManhole() {
    int n_trues = 0;
    
    for (unsigned int i = 0;i < manhole_hist.size(); i++) {
      
      if (manhole_hist[i])
        n_trues++;
    }
    
    
    ROS_INFO("Number of detections %d. Number of positive: %d", (int)manhole_hist.size(), n_trues);
    
    manhole_hist.clear();
    
    return n_trues >= m_min_manhole_detected;
  }
  
  //! Indicates if the filter was initialized
  bool m_init;
  
  //! Particles 
  std::vector<Particle> m_p;
  
  //! Particles roll and pich (given by IMU)
  double m_roll, m_pitch;
  
  //! Number of particles in the filter
  int m_maxParticles;
  int m_minParticles;
  
  //! Odometry characterization
  double m_odomXMod, m_odomYMod, m_odomZMod, m_odomAMod, m_odomANoise;
  double m_initX, m_initY, m_initA;
  double m_initXDev, m_initYDev, m_initADev;
  
  //! Resampling control
  int m_nUpdates;
  int m_resampleInterval;
  
  //! Yaw estimation
  plane_detector::WallInfo last_info;
  ros::Time last_relative_time;
  double angular_weight;
  
  //! Thresholds for filter updating
  double m_dTh, m_aTh, m_tTh;
  tf::StampedTransform m_lastOdomTf;
  tf::Transform m_lastGlobalTf;
  geometry_msgs::PoseWithCovarianceStamped m_lastPoseCov;
  bool m_doUpdate;
  double m_updateRate;

  //! Node parameters
  std::string m_detectManholeTopic;
  std::string m_baseFrameId;
  std::string m_odomFrameId;
  std::string m_globalFrameId;
  std::string m_inOdomTfTopic;
  
  //! ROS msgs and data
  ros::NodeHandle m_nh;
  tf::TransformBroadcaster m_tfBr;
  tf::TransformListener m_tfListener;
  ros::Subscriber m_detect_manhole_Sub, m_initialPoseSub, m_odomTfSub, wall_info_sub;
  ros::Publisher m_posesPub, m_graphPub, m_gpsPub, m_posecovPub;
  ros::Timer updateTimer;
  
  // Sewer stuff
  std::string m_sewer_graph_file;
  sewer_graph::SewerGraph *s_g;
  
  //! Random number generator
  const gsl_rng_type *m_randomType;
  gsl_rng *m_randomValue;
  
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > m_randVar;   

  std::vector<bool> manhole_hist;
  int m_min_manhole_detected;
  double m_edgeDev, m_manholeDev, m_manholeThres, m_angleDev;
  double edgeConst1, edgeConst2;
  double manholeConst1, manholeConst2;
  double angleConst1, angleConst2;
  double m_forkDev, forkConst1, forkConst2, m_fork_dist;
  
  //For saving trajetory
  std::ofstream traj_file;
  std::string traj_filename;
  bool traj_file_open;
};

#endif
