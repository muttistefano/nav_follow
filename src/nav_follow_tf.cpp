#include <nav_follow/nav_follow.h>


using namespace boost::assign;
using namespace boost::adaptors;
using namespace pcl::registration;  
using namespace std::chrono_literals;


inline bool exists_file (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}


void nav_follow_class::current_tf_thread()
{
    while(rclcpp::ok())
    {
        try {
            std::scoped_lock<std::mutex> guard_tf(_tf_mutex);
            _tf_master_slave = _tf_buffer->lookupTransform(
            _params.frame_name_base_master, _params.frame_name_base_slave,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            _params.frame_name_base_master.c_str(), _params.frame_name_base_slave.c_str(), ex.what());
        }
        // std::this_thread::sleep_for(50ms);
        _rate_tf->sleep();
    }
}

void nav_follow_class::get_tf_goal()
{
    //TODO spin_once ??
    tf2::Stamped<tf2::Transform> stamped_transform;
    while(!_tf_buffer->canTransform(_params.frame_name_base_master, _params.frame_name_base_slave, tf2::TimePointZero, 2000ms) && rclcpp::ok()) {
      RCLCPP_INFO(
                    get_logger(), "waiting 2000 ms for %s->%s transform to become available",
                    _params.frame_name_base_master.c_str(), _params.frame_name_base_slave.c_str());
      std::this_thread::sleep_for(200ms);
    }
    _tf_goal_msg = _tf_buffer->lookupTransform(
            _params.frame_name_base_master, _params.frame_name_base_slave,
            tf2::TimePointZero);

    tf2::fromMsg(_tf_goal_msg, _tf_goal_transform);

    RCLCPP_INFO(get_logger(), "Initial transformation stored");
}

void nav_follow_class::tf_follow_thread()
{

    tf2::Stamped<tf2::Transform> stamped_transform_now;
    geometry_msgs::msg::Twist vel_cmd;
    double x_err,y_err,w_err;
    double x_cmd,y_cmd,w_cmd;

    while(rclcpp::ok() && this->_tf_controlling)
    {
        {
            std::scoped_lock<std::mutex> guard_tf(_tf_mutex);
            tf2::fromMsg(_tf_master_slave, stamped_transform_now);
        }

        tf2::Transform err =  stamped_transform_now.inverse() * _tf_goal_transform;

        x_err = err.getOrigin().getX();
        y_err = err.getOrigin().getY();

        double r{}, p{}, y{};
        tf2::Matrix3x3 m(err.getRotation());
        m.getRPY(r, p, y);
        w_err = y;


        RCLCPP_INFO_STREAM(get_logger(),"err: " << x_err << " " << y_err << " " << w_err << " " << "\n");
        //TODO rate to Duration ?
        auto dt = rclcpp::Duration(50ms);
        x_cmd = _x_pid->computeCommand(x_err,dt);
        y_cmd = _y_pid->computeCommand(y_err,dt);
        w_cmd = _w_pid->computeCommand(w_err,dt);
        
        RCLCPP_INFO_STREAM(get_logger(),"cmd: " << x_cmd << " " << y_cmd << " " << w_cmd << " " << "\n");
        vel_cmd.linear.x  = x_cmd;
        vel_cmd.linear.y  = y_cmd;
        vel_cmd.angular.z = w_cmd;
        _cmd_vel->publish(vel_cmd);
        _rate_follow->sleep();
    }
}
