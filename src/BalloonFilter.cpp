#include <balloon_filter/BalloonFilter.h>

namespace balloon_filter
{

  /* main_loop() method //{ */
  void BalloonFilter::main_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    load_dynparams(m_drmgr_ptr->config);

    if (!m_estimating)
    {
      ROS_INFO_THROTTLE(1.0, "[%s]: Estimation stopped, skipping.", m_node_name.c_str());
      return;
    }

    if (m_sh_balloons->new_data())
    {
      const auto balloons = *(m_sh_balloons->get_data());

      if (!balloons.detections.empty())
      {
        ROS_INFO("[%s]: Processing %lu new detections vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv", m_node_name.c_str(), balloons.detections.size());
        auto measurements = message_to_positions(balloons);
        if (!measurements.empty())
        {
          pos_cov_t used_meas;
          bool used_meas_valid = false;
          if (m_current_estimate_exists)
            used_meas_valid = update_current_estimate(measurements, balloons.header.stamp, used_meas);
          else
            used_meas_valid = init_current_estimate(measurements, balloons.header.stamp, used_meas);

          /* publish the used measurement for debugging and visualisation purposes //{ */
          if (used_meas_valid)
          {
            std_msgs::Header header;
            header.frame_id = m_world_frame;
            header.stamp = m_current_estimate_last_update;
            m_pub_used_meas.publish(to_output_message(used_meas, header));
          }
          //}
        }
        ros::Duration del = ros::Time::now() - balloons.header.stamp;
        ROS_INFO_STREAM("delay (from image acquisition): " << del.toSec() * 1000.0 << "ms");
        ROS_INFO("[%s]: New data processed          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^", m_node_name.c_str());
      } else
      {
        ROS_INFO_THROTTLE(1.0, "[%s]: Empty detections message received", m_node_name.c_str());
      }
    }
    if ((ros::Time::now() - m_current_estimate_last_update).toSec() >= m_max_time_since_update)
    {
      reset_current_estimate();
    }

    if (m_current_estimate_exists && m_current_estimate_n_updates > m_min_updates_to_confirm)
    {
      std_msgs::Header header;
      header.frame_id = m_world_frame;
      header.stamp = m_current_estimate_last_update;
      m_pub_chosen_balloon.publish(to_output_message({m_current_estimate.x, m_current_estimate.P}, header));
      ROS_INFO_THROTTLE(1.0, "[%s]: Current chosen balloon position: [%.2f, %.2f, %.2f]", m_node_name.c_str(), m_current_estimate.x.x(),
                        m_current_estimate.x.y(), m_current_estimate.x.z());
    }
  }
  //}

  /* update_current_estimate() method //{ */
  bool BalloonFilter::update_current_estimate(const std::vector<pos_cov_t>& measurements, const ros::Time& stamp, pos_cov_t& used_meas)
  {
    pos_cov_t closest_meas;
    bool meas_valid = find_closest_to(measurements, m_current_estimate.x, closest_meas, true);
    if (meas_valid)
    {
      ROS_INFO("[%s]: Updating current estimate using point [%.2f, %.2f, %.2f]", m_node_name.c_str(), closest_meas.pos.x(), closest_meas.pos.y(),
               closest_meas.pos.z());
      const double dt = (stamp - m_current_estimate_last_update).toSec();

      LKF::Q_t Q = m_process_noise_std * LKF::Q_t::Identity(6, 6);
      LKF::A_t A = LKF::A_t::Identity(6, 6);
      A(0, 3) = A(1, 4) = A(2, 5) = dt*m_process_noise_std;
      m_lkf.A = A;

      LKF::x_t x = LKF::x_t::Zero(6, 1);
      x.block<3, 1>(0, 0) = m_current_estimate.x;
      m_current_estimate.x = x;

      LKF::P_t P = m_process_noise_std * LKF::P_t::Ones(6, 6);
      P.block<3, 3>(0, 0) = m_current_estimate.P;
      m_current_estimate.P = P;

      m_current_estimate = m_lkf.predict(m_current_estimate, LKF::u_t(), Q, dt);

      x = m_current_estimate.x.block(0, 0, 3, 1);
      P = m_current_estimate.P.block(0, 0, 3, 3);
      m_current_estimate.x = x;
      m_current_estimate.P = P;

      A = LKF::A_t::Identity(3, 3);
      m_lkf.A = A;
      m_current_estimate = m_lkf.correct(m_current_estimate, closest_meas.pos, closest_meas.cov);

      m_current_estimate_last_update = stamp;
      m_current_estimate_n_updates++;
      used_meas = closest_meas;
    } else
    {
      ROS_INFO("[%s]: No point is close enough to [%.2f, %.2f, %.2f]", m_node_name.c_str(), m_current_estimate.x.x(), m_current_estimate.x.y(),
               m_current_estimate.x.z());
    }
    return meas_valid;
  }
  //}

  /* init_current_estimate() method //{ */
  bool BalloonFilter::init_current_estimate(const std::vector<pos_cov_t>& measurements, const ros::Time& stamp, pos_cov_t& used_meas)
  {
    pos_cov_t closest_meas;
    bool meas_valid = find_closest(measurements, closest_meas);
    if (meas_valid)
    {
      ROS_INFO("[%s]: Initializing estimate using point [%.2f, %.2f, %.2f]", m_node_name.c_str(), closest_meas.pos.x(), closest_meas.pos.y(),
               closest_meas.pos.z());
      m_current_estimate.x = closest_meas.pos;
      m_current_estimate.P = closest_meas.cov;
      m_current_estimate_exists = true;
      m_current_estimate_last_update = stamp;
      m_current_estimate_n_updates = 1;
      used_meas = closest_meas;
    } else
    {
      ROS_INFO("[%s]: No point is valid for estimate initialization", m_node_name.c_str());
    }
    return meas_valid;
  }
  //}

  /* to_output_message() method //{ */
  geometry_msgs::PoseWithCovarianceStamped BalloonFilter::to_output_message(const pos_cov_t& estimate, const std_msgs::Header& header)
  {
    geometry_msgs::PoseWithCovarianceStamped ret;

    ret.header = header;
    ret.pose.pose.position.x = estimate.pos.x();
    ret.pose.pose.position.y = estimate.pos.y();
    ret.pose.pose.position.z = estimate.pos.z();
    ret.pose.pose.orientation.x = 0.0;
    ret.pose.pose.orientation.y = 0.0;
    ret.pose.pose.orientation.z = 0.0;
    ret.pose.pose.orientation.w = 1.0;

    for (int r = 0; r < 6; r++)
    {
      for (int c = 0; c < 6; c++)
      {
        if (r < 3 && c < 3)
          ret.pose.covariance[r * 6 + c] = estimate.cov(r, c);
        else if (r == c)
          ret.pose.covariance[r * 6 + c] = 666;
        else
          ret.pose.covariance[r * 6 + c] = 0.0;
      }
    }

    return ret;
  }
  //}

  /* get_cur_mav_pos() method //{ */
  pos_t BalloonFilter::get_cur_mav_pos()
  {
    Eigen::Affine3d m2w_tf;
    bool tf_ok = get_transform_to_world(m_uav_frame_id, ros::Time::now(), m2w_tf);
    if (!tf_ok)
      return pos_t(0, 0, 0);
    ;
    return m2w_tf.translation();
  }
  //}

  /* find_closest_to() method //{ */
  bool BalloonFilter::find_closest_to(const std::vector<pos_cov_t>& measurements, const pos_t& to_position, pos_cov_t& closest_out, bool use_gating)
  {
    double min_dist = std::numeric_limits<double>::infinity();
    pos_cov_t closest_pt{std::numeric_limits<double>::quiet_NaN() * pos_t::Ones(), std::numeric_limits<double>::quiet_NaN() * cov_t::Ones()};
    for (const auto& pt : measurements)
    {
      const double cur_dist = (to_position - pt.pos).norm();
      if (cur_dist < min_dist)
      {
        min_dist = cur_dist;
        closest_pt = pt;
      }
    }
    if (use_gating)
    {
      if (min_dist < m_gating_distance)
      {
        closest_out = closest_pt;
        return true;
      } else
      {
        return false;
      }
    } else
    {
      closest_out = closest_pt;
      return true;
    }
  }
  //}

  /* find_closest() method //{ */
  bool BalloonFilter::find_closest(const std::vector<pos_cov_t>& measuremets, pos_cov_t& closest_out)
  {
    pos_t cur_pos = get_cur_mav_pos();
    return find_closest_to(measuremets, cur_pos, closest_out, false);
  }
  //}

  /* message_to_positions() method //{ */
  std::vector<pos_cov_t> BalloonFilter::message_to_positions(const detections_t& balloon_msg)
  {
    std::vector<pos_cov_t> ret;

    // Construct a new world to sensor transform
    Eigen::Affine3d s2w_tf;
    bool tf_ok = get_transform_to_world(balloon_msg.header.frame_id, balloon_msg.header.stamp, s2w_tf);
    if (!tf_ok)
      return ret;
    const Eigen::Matrix3d s2w_rot = s2w_tf.rotation();

    ret.reserve(balloon_msg.detections.size());
    for (size_t it = 0; it < balloon_msg.detections.size(); it++)
    {
      const auto& cur_ball = balloon_msg.detections[it];
      const object_detect::color_id_t ball_color_id = object_detect::color_id_t(cur_ball.type);
      if (ball_color_id != m_filtered_color_id)
      {
        ROS_INFO("[%s]: Skipping uninteresting ball with color '%s' (looking only for '%s')", m_node_name.c_str(), object_detect::color_name(ball_color_id).c_str(), object_detect::color_name(m_filtered_color_id).c_str());
        continue;
      }
      const auto msg_pos = cur_ball.pose.pose;
      const auto msg_cov = cur_ball.pose.covariance;
      const pos_t pos = s2w_tf * pos_t(msg_pos.position.x, msg_pos.position.y, msg_pos.position.z);
      if (point_valid(pos))
      {
        const cov_t cov = rotate_covariance(msg2cov(msg_cov), s2w_rot);
        const pos_cov_t pos_cov{pos, cov};
        ret.push_back(pos_cov);
        ROS_INFO("[%s]: Adding valid point [%.2f, %.2f, %.2f] (original: [%.2f %.2f %.2f])", m_node_name.c_str(), pos.x(), pos.y(), pos.z(),
                 msg_pos.position.x, msg_pos.position.y, msg_pos.position.z);
      } else
      {
        ROS_WARN("[%s]: Skipping invalid point [%.2f, %.2f, %.2f] (original: [%.2f %.2f %.2f])", m_node_name.c_str(), pos.x(), pos.y(), pos.z(),
                 msg_pos.position.x, msg_pos.position.y, msg_pos.position.z);
      }
    }

    return ret;
  }
  //}

  /* msg2cov() method //{ */
  cov_t BalloonFilter::msg2cov(const ros_cov_t& msg_cov)
  {
    cov_t cov;
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        cov(r, c) = msg_cov[r * 6 + c];
      }
    }
    return cov;
  }
  //}

  /* rotate_covariance() method //{ */
  cov_t BalloonFilter::rotate_covariance(const cov_t& covariance, const cov_t& rotation)
  {
    return rotation * covariance * rotation.transpose();  // rotate the covariance to point in direction of est. position
  }
  //}

  /* point_in_sphere() method //{ */
  bool BalloonFilter::point_in_sphere(const pos_t& pt, const sphere_t sphere)
  {
    const double dist_from_center = (pt - sphere.center).norm();
    if (dist_from_center < sphere.radius)
      return true;
    return false;
  }
  //}

  /* point_in_exclusion_zone() method //{ */
  bool BalloonFilter::point_in_exclusion_zone(const pos_t& pt, const std::vector<sphere_t>& exclusion_zones)
  {
    for (const auto& zone : exclusion_zones)
    {
      if (point_in_sphere(pt, zone))
        return true;
    }
    return false;
  }
  //}

  /* point_valid() method //{ */
  bool BalloonFilter::point_valid(const pos_t& pt)
  {
    const bool height_valid = pt.z() > m_z_bounds_min && pt.z() < m_z_bounds_max;
    const bool sane_values = !pt.array().isNaN().any() && !pt.array().isInf().any();
    const bool not_excluded = !point_in_exclusion_zone(pt, m_exclusion_zones);
    const bool in_area = point_in_sphere(pt, m_initial_area);
    return height_valid && sane_values && not_excluded && in_area;
  }
  //}

  /* reset_current_estimate() method //{ */
  void BalloonFilter::reset_current_estimate()
  {
    m_current_estimate_exists = false;
    m_current_estimate_last_update = ros::Time::now();
    m_current_estimate_n_updates = 0;
    ROS_INFO("[%s]: Current chosen balloon ==RESET==.", m_node_name.c_str());
  }
  //}

  /* load_dynparams() method //{ */
  void BalloonFilter::load_dynparams(drcfg_t cfg)
  {
    m_z_bounds_min = cfg.z_bounds__min;
    m_z_bounds_max = cfg.z_bounds__max;
    m_gating_distance = cfg.gating_distance;
    m_max_time_since_update = cfg.max_time_since_update;
    m_min_updates_to_confirm = cfg.min_updates_to_confirm;
  }
  //}

  /* onInit() //{ */

  void BalloonFilter::onInit()
  {

    ROS_INFO("[%s]: Initializing", m_node_name.c_str());
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    ros::Time::waitForValid();

    /* load parameters //{ */

    // LOAD DYNAMIC PARAMETERS
    ROS_INFO("[%s]: LOADING DYNAMIC PARAMETERS", m_node_name.c_str());
    m_drmgr_ptr = std::make_unique<drmgr_t>(nh, m_node_name);
    if (!m_drmgr_ptr->loaded_successfully())
    {
      ROS_ERROR("Some dynamic parameter default values were not loaded successfully, ending the node");
      ros::shutdown();
    }

    ROS_INFO("[%s]: LOADING STATIC PARAMETERS", m_node_name.c_str());
    mrs_lib::ParamLoader pl(nh, m_node_name);

    double filter_period = pl.load_param2<double>("filter_period");
    pl.load_param("world_frame", m_world_frame);
    pl.load_param("uav_frame_id", m_uav_frame_id);
    pl.load_param("gating_distance", m_gating_distance);
    pl.load_param("max_time_since_update", m_max_time_since_update);
    pl.load_param("min_updates_to_confirm", m_min_updates_to_confirm);
    pl.load_param("process_noise_std", m_process_noise_std);
    pl.load_param("max_speed", m_max_speed);

    std::string filtered_color_name = pl.load_param2<std::string>("filtered_color");
    std::transform(filtered_color_name.begin(), filtered_color_name.end(), filtered_color_name.begin(), ::tolower);
    m_filtered_color_id = object_detect::color_id(filtered_color_name);

    if (!pl.loaded_successfully())
    {
      ROS_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
      ros::shutdown();
    }

    //}

    /* subscribers //{ */

    m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer, m_node_name);
    mrs_lib::SubscribeMgr smgr(nh);
    constexpr bool time_consistent = true;
    m_sh_balloons =
        smgr.create_handler<detections_t, time_consistent>("balloon_detections", ros::Duration(5.0));

    m_start_estimation_server = nh.advertiseService("start_estimation", &BalloonFilter::start_estimation, this);
    m_stop_estimation_server = nh.advertiseService("stop_estimation", &BalloonFilter::stop_estimation, this);
    m_reset_estimation_server = nh.advertiseService("reset_estimation", &BalloonFilter::reset_estimation, this);
    m_add_exclusion_zone_server = nh.advertiseService("add_exclusion_zone", &BalloonFilter::add_exclusion_zone, this);
    m_reset_exclusion_zones_server = nh.advertiseService("reset_exclusion_zones", &BalloonFilter::reset_exclusion_zones, this);
    //}

    /* publishers //{ */

    m_pub_chosen_balloon = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("balloon_chosen_out", 1);
    m_pub_used_meas = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("balloon_detection_used", 1);

    //}

    /* profiler //{ */

    m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

    //}

    {
      LKF::A_t A = LKF::A_t::Identity(3, 3);
      LKF::B_t B;
      LKF::H_t H = LKF::H_t::Identity(3, lkf_n_measurements);
      m_lkf = LKF(A, B, H);
    }
    reset_current_estimate();
    m_is_initialized = true;
    m_estimating = false;

    /* timers  //{ */

    m_main_loop_timer = nh.createTimer(ros::Duration(filter_period), &BalloonFilter::main_loop, this);

    //}

    ROS_INFO("[%s]: initialized", m_node_name.c_str());
  }

  //}

  /* service callbacks //{ */

  bool BalloonFilter::start_estimation(balloon_filter::StartEstimation::Request& req, balloon_filter::StartEstimation::Response& resp)
  {
    reset_current_estimate();
    m_initial_area = {Eigen::Vector3d(req.inital_point.x, req.inital_point.y, req.inital_point.z), req.radius};
    m_estimating = true;
    std::stringstream strstr;
    strstr << "Starting estimation at point " << m_initial_area.center.transpose() << " with search radius " << m_initial_area.radius << ".";
    resp.message = strstr.str();
    resp.success = true;
    return true;
  }

  bool BalloonFilter::stop_estimation([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
  {
    m_estimating = false;
    resp.message = "Stopped estimation.";
    resp.success = true;
    return true;
  }

  bool BalloonFilter::reset_estimation([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
  {
    reset_current_estimate();
    resp.message = "Estimation was reset.";
    resp.success = true;
    return true;
  }

  bool BalloonFilter::add_exclusion_zone(balloon_filter::AddExclusionZone::Request& req, balloon_filter::AddExclusionZone::Response& resp)
  {
    const Eigen::Vector3d new_zone_pt(req.zone_center.x, req.zone_center.y, req.zone_center.z);
    const double new_zone_radius = req.zone_radius;
    m_exclusion_zones.push_back({new_zone_pt, new_zone_radius});
    std::stringstream strstr;
    strstr << "Adding exclusion zone with center " << new_zone_pt.transpose() << " and radius " << new_zone_radius << "m for a total of "
           << m_exclusion_zones.size() << " zones.";
    resp.message = strstr.str();
    resp.success = true;
    return true;
  }

  bool BalloonFilter::reset_exclusion_zones([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
  {
    m_exclusion_zones.clear();
    resp.message = "Exclusion zones were reset.";
    resp.success = true;
    return true;
  }

  //}

}  // namespace balloon_filter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_filter::BalloonFilter, nodelet::Nodelet)
