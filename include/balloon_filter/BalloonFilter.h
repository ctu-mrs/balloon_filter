#ifndef BALLOONFILTER_H
#define BALLOONFILTER_H

/* includes //{ */

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nodelet/nodelet.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Geometry msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
/* #include <geometry_msgs/PoseStamped.h> */

// Nav msgs
#include <nav_msgs/Odometry.h>

// Standard services
#include <std_srvs/Trigger.h>

// MRS stuff
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>
#include <mrs_lib/lkf.h>

// std
#include <string>
#include <mutex>

// local includes
#include <balloon_filter/FilterParamsConfig.h>
#include <balloon_filter/StartEstimation.h>
#include <balloon_filter/AddExclusionZone.h>
#include <object_detect/BallDetections.h>

//}

namespace balloon_filter
{
  // shortcut type to the dynamic reconfigure manager template instance
  using drcfg_t = balloon_filter::FilterParamsConfig;
  using drmgr_t = mrs_lib::DynamicReconfigureMgr<drcfg_t>;
  constexpr int lkf_n_states = 6;
  constexpr int lkf_n_inputs = 0;
  constexpr int lkf_n_measurements = 3;
  using LKF = mrs_lib::LKF<lkf_n_states, lkf_n_inputs, lkf_n_measurements>;

  using detections_t = object_detect::BallDetections;
  using detection_t = object_detect::BallDetection;
  using ros_pose_t = detection_t::_pose_type::_pose_type;
  using ros_cov_t = detection_t::_pose_type::_covariance_type;

  using pos_t = Eigen::Matrix<double, 3, 1>;
  using cov_t = Eigen::Matrix<double, 3, 3>;
  struct pos_cov_t
  {
    pos_t pos;
    cov_t cov;
  };

  struct sphere_t
  {
    Eigen::Vector3d center;
    double radius;
  };

  /* //{ class BalloonFilter */

  class BalloonFilter : public nodelet::Nodelet
  {
    public:
      BalloonFilter() : m_node_name("BalloonFilter") {};
      virtual void onInit();

      bool m_is_initialized;

    private:
      const std::string m_node_name;
      void main_loop([[maybe_unused]] const ros::TimerEvent& evt);

    private:
      std::unique_ptr<mrs_lib::Profiler> m_profiler_ptr;

    private:

      // --------------------------------------------------------------
      // |                ROS-related member variables                |
      // --------------------------------------------------------------

      /* Parameters, loaded from ROS //{ */
      std::string m_world_frame_id;
      std::string m_uav_frame_id;
      double m_filter_coeff;
      double m_gating_distance;
      double m_max_time_since_update;
      double m_min_updates_to_confirm;
      double m_process_noise_std;
      double m_z_bounds_min;
      double m_z_bounds_max;
      double m_max_speed;
      bool m_constrain_vel_to_plane;
      //}

      /* ROS related variables (subscribers, timers etc.) //{ */
      std::unique_ptr<drmgr_t> m_drmgr_ptr;
      tf2_ros::Buffer m_tf_buffer;
      std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
      mrs_lib::SubscribeHandler<detections_t> m_sh_balloons;

      ros::Publisher m_pub_chosen_balloon;
      ros::Publisher m_pub_used_meas;

      // service servers
      ros::ServiceServer m_start_estimation_server;
      ros::ServiceServer m_stop_estimation_server;
      ros::ServiceServer m_reset_estimation_server;
      ros::ServiceServer m_add_exclusion_zone_server;
      ros::ServiceServer m_reset_exclusion_zones_server;

      ros::Timer m_main_loop_timer;
      //}

      bool m_estimating;
      sphere_t m_initial_area;
      bool m_current_estimate_exists;
      LKF::statecov_t m_current_estimate;
      LKF m_lkf;
      ros::Time m_current_estimate_last_update;
      int m_current_estimate_n_updates;
      std::vector<sphere_t> m_exclusion_zones;

    private:

      // --------------------------------------------------------------
      // |                helper implementation methods               |
      // --------------------------------------------------------------

      /* get_transform_to_world() method //{ */
      bool get_transform_to_world(const std::string& frame_name, ros::Time stamp, Eigen::Affine3d& tf_out)
      {
        try
        {
          const ros::Duration timeout(1.0 / 100.0);
          geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(m_world_frame_id, frame_name, stamp, timeout);
          tf_out = tf2::transformToEigen(transform.transform);
        } catch (tf2::TransformException& ex)
        {
          ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", frame_name.c_str(), m_world_frame_id.c_str(), ex.what());
          return false;
        }
        return true;
      }
      //}

      cov_t msg2cov(const ros_cov_t& msg_cov);
      cov_t rotate_covariance(const cov_t& covariance, const cov_t& rotation);
      bool point_in_exclusion_zone(const pos_t& pt, const std::vector<sphere_t>& exclusion_zones);
      bool point_in_sphere(const pos_t& pt, const sphere_t sphere);
      bool point_valid(const pos_t& pt);

      bool update_current_estimate(const std::vector<pos_cov_t>& measurements, const ros::Time& stamp, pos_cov_t& used_meas);
      bool init_current_estimate(const std::vector<pos_cov_t>& measurements, const ros::Time& stamp, pos_cov_t& used_meas);
      void reset_current_estimate();
      nav_msgs::Odometry to_output_message(const LKF::statecov_t& estimate, const std_msgs::Header& header);
      geometry_msgs::PoseWithCovarianceStamped to_output_message(const pos_cov_t& estimate, const std_msgs::Header& header);
      pos_t get_cur_mav_pos();
      bool find_closest_to(const std::vector<pos_cov_t>& measurements, const pos_t& to_position, pos_cov_t& closest_out, bool use_gating = false);
      bool find_closest(const std::vector<pos_cov_t>& measurements, pos_cov_t& closest_out);

      std::vector<pos_cov_t> message_to_positions(const detections_t& balloon_msg);

      bool start_estimation(balloon_filter::StartEstimation::Request& req, balloon_filter::StartEstimation::Response& resp);
      bool stop_estimation(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
      bool reset_estimation(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
      bool add_exclusion_zone(balloon_filter::AddExclusionZone::Request& req, balloon_filter::AddExclusionZone::Response& resp);
      bool reset_exclusion_zones(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

      void load_dynparams(drcfg_t cfg);

  };
  
  //}

}  // namespace balloon_filter

#endif
