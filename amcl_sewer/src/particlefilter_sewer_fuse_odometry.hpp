#ifndef __PARTICLEFILTERFUSED_HPP__
#define __PARTICLEFILTERFUSED_HPP__

#include "particlefilter_sewer.hpp"
#include "std_msgs/Int8.h"
#include <functions/SimpleProfiler.hpp>
#define PF_PROFILE
  

//Class definition
class ParticleFilterFused:public ParticleFilter
{
public:

  //!Default contructor
  ParticleFilterFused(std::string &node_name) : ParticleFilter(node_name)
  {
    ROS_INFO("Initializing fused particle filter");
    // Read node parameters
    if(!lnh.getParam("base_frame_id2", m_baseFrameId2))
      m_baseFrameId2 = "base_link_vo";
    if(!lnh.getParam("odom_frame_id2", m_odomFrameId2))
      m_odomFrameId2 = "odom_vo";

    m_bad_odom_sub = m_nh.subscribe("/rgbd_odom_node/bad_odom", 1, &ParticleFilterFused::badOdomCallback, this);
    bad_odom = 0;

    // // Init the particle filter internal stuff
    // m_nUpdates = 0;
    // m_doUpdate = false;
    // m_doUpdate2 = false;
    // m_p.resize(m_maxParticles);

    // // Initialize TF from odom to map as identity
    // m_lastGlobalTf.setIdentity();

    // In principle, trust wheel
    m_vo = false;
  }

  //!Default destructor
  ~ParticleFilterFused()
  {
    
  }

  //! Check motion and time thresholds for AMCL update
  virtual bool checkUpdateThresholds()
  {
    // Publish current TF from odom to map
    m_tfBr.sendTransform(tf::StampedTransform(m_lastGlobalTf, ros::Time::now(), m_globalFrameId, m_odomFrameId));

    m_vo = false;

    // If the filter is not initialized then exit
    if(!m_init)
      return false;

    // Compute odometric trasnlation and rotation since last update
    tf::StampedTransform odomTf, odomTf2;
    try
    {
      m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
      m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
      m_tfListener.waitForTransform(m_odomFrameId2, m_baseFrameId2, ros::Time(0), ros::Duration(1.0));
      m_tfListener.lookupTransform(m_odomFrameId2, m_baseFrameId2, ros::Time(0), odomTf2);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("AMCL error: %s",ex.what());
      return false;
    }
    tf::Transform T = m_lastOdomTf.inverse()*odomTf;
    tf::Transform T2 = m_lastOdomTf2.inverse()*odomTf2;

    // Check translation threshold
    if(T.getOrigin().length() > m_dTh)
    {
      ROS_INFO("Translation update");
      m_doUpdate = true;
      if ( fabs(T.getOrigin().length() - T2.getOrigin().length()) > m_dTh*0.82 &&
           bad_odom == 0 ) {
        // Detected slipage
        ROS_INFO("Slippage detected --> using VO");
        m_vo = true;
      }
      if (bad_odom > 0)
        bad_odom--;
    } else {

      // Check yaw threshold
      double yaw, yaw2, pitch, roll;
      T.getBasis().getRPY(roll, pitch, yaw);
      T2.getBasis().getRPY(roll, pitch, yaw2);
      if(fabs(yaw) > m_dTh)
      {
        ROS_INFO("Rotation update");
        m_doUpdate = true;
        if ( fabs(yaw - yaw2) > m_aTh*0.82 && bad_odom == 0) {
          ROS_INFO("Slippage detected --> using VO");
          m_vo = true;
        }
        if (bad_odom > 0) 
          bad_odom--;
      }
    }

    if (m_doUpdate) {
#ifdef PF_PROFILE
      // static functions::SimpleProfiler pro("update_pf.profile.txt");
      // auto f_ = std::bind(&ParticleFilterFused::update, this, checkManhole());
      // pro.profileFunction<void>(f_);
      PROFILE_METHOD_1("update_pf.profile.txt", ParticleFilterFused, update, void, checkManhole()); 
#else
      update(checkManhole());
#endif
    } 

    return false;
  }

private:

  virtual void update(bool detected_manhole) {

    if(!predictParticles())
    {
      ROS_ERROR("ParticleFilterSewer::manholeDetectedCallback --> Prediction error!");
      return;
    }

    float x,y,a,xv,yv,av,xycov;
    computeVar(x,y,a,xv,yv,av,xycov);

    if (traj_file_open) {
      traj_file << x << "\t" << y << "\t" << a <<"\t"<< ros::Time::now().sec << "." << ros::Time::now().nsec<<  std::endl;
    }

    // Perform different particle update based on current point-cloud and available measurements
    if (detected_manhole) {
      ROS_INFO("Performing Update with Manhole");
      updateParticles(MANHOLE);
    } else if (isFork(x,y)) {
      ROS_INFO("Performing update with fork");
      updateParticles(FORK);
    } else if (last_relative_time - ros::Time::now() < ros::Duration(0,100000000L) && fabs(last_yaw) < 5) {

      if (gsl_rng_uniform(m_randomValue) < m_angularRate) {
        ROS_INFO("Performing angular update. ");
        updateParticles(ANGULAR);

      } else {
        ROS_INFO("Performing regular update, but detected yaw.");
        updateParticles(REGULAR);
      }
    } else {
      ROS_INFO("Performing regular update");
      updateParticles(REGULAR);
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

    m_doUpdate = false;
    m_doUpdate2 = false;

    // Publish particles
    publishParticles();
    m_posecovPub.publish(m_lastPoseCov);
  }

  //!This function implements the PF prediction stage. Translation in X, Y and Z
  //!in meters and yaw angle incremenet in rad
  virtual bool predictParticles()
  {
    // Compute odometric trasnlation and rotation since last update
    tf::StampedTransform odomTf, odomTf2;
    try
    {
      m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(0.1));
      m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
      m_tfListener.waitForTransform(m_odomFrameId2, m_baseFrameId2, ros::Time(0), ros::Duration(0.1));
      m_tfListener.lookupTransform(m_odomFrameId2, m_baseFrameId2, ros::Time(0), odomTf2);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s 1",ex.what());
      return false;
    }

    tf::StampedTransform last = m_vo?m_lastOdomTf2:m_lastOdomTf;

    // Perform particle prediction based on odometry
    float delta_x, delta_y;
    double delta_r, delta_p, delta_a;
    tf::Transform T = m_vo?m_lastOdomTf2.inverse()*odomTf2:m_lastOdomTf.inverse()*odomTf;
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
    m_lastOdomTf2 = odomTf2;

    return true;
  }

  void badOdomCallback(const std_msgs::Int8ConstPtr &msg) {
    if (msg->data > 0) {
      ROS_INFO("Received bad odom information: %d", msg->data);
      bad_odom = 5;
    }
  }

  

  bool m_doUpdate2;
  int bad_odom;
  double m_angularRate;
  tf::StampedTransform m_lastOdomTf2;

  std::string m_baseFrameId2;
  std::string m_odomFrameId2;

  ros::Subscriber m_bad_odom_sub;

  bool m_vo; // Flag for considering vo (m_odomFrameId2) or wheel
};

#endif
