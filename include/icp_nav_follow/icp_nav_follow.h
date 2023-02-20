#include "rclcpp/rclcpp.hpp"
#include <math.h> 
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/msg/point32.hpp>
// #include <icp_nav_follow/save_pcl.h>
// #include <icp_nav_follow/move_to_pcl.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/range/adaptor/indexed.hpp>
#include <pcl/io/pcd_io.h>
#include <boost/assign.hpp>
#include <iterator>
#include <eigen3/Eigen/Core>
// #include <visualization_msgs/Marker.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/wait_for_message.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_srvs/srv/trigger.hpp>


constexpr int   LASER_SCAN_FILTER_LENGTH   = 3;
// constexpr double FILT_DIST                 = 3.0;
// constexpr double MIN_DIST                  = 0.3;

class icp_nav_follow_class : public rclcpp::Node
{
    private:

        std::mutex   _lock_pcl_master,_lock_pcl_slave;
        std::mutex   _tf_mutex, _icp_mutex;

        int     _icp_iterations;
        double  _icp_TransformationEpsilon;
        double  _icp_EuclideanFitnessEpsilon;
        double  _icp_RANSACOutlierRejectionThreshold;
        double  _icp_MaxCorrespondenceDistance;

        //PARAMS
        std::string  _laser_scan_master;
        std::string  _laser_scan_slave;
        std::string  _frame_name_laser_slave;
        std::string  _frame_name_laser_master;
        std::string  _frame_name_base_slave;
        std::string  _frame_name_base_master;
        std::string  _cmd_vel_topic;
        // std::string  _slave_frame,_master_frame;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
        rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
        

        //ROS

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr        _sub_master;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr        _sub_slave;

        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr     _vis_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             _cmd_vel;
            
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         _pcl_master_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         _pcl_slave_pub;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _save_icp_goal_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _start_icp_goal_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _stop_icp_goal_srv;

        geometry_msgs::msg::Twist cmd_vel_dock;


        //PCL
        boost::circular_buffer<sensor_msgs::msg::PointCloud>  _pcl_buffer_master;
        boost::circular_buffer<sensor_msgs::msg::PointCloud>  _pcl_buffer_slave;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> _icp;

        sensor_msgs::msg::PointCloud _NewPcl_master;
        sensor_msgs::msg::PointCloud _NewPcl_slave;
        sensor_msgs::msg::PointCloud _TPcl;
        geometry_msgs::msg::Point32  _pnt_filt_slave;
        geometry_msgs::msg::Point32  _pnt_filt_master;

        Eigen::Matrix4d _icp_current = Eigen::Matrix4d::Identity ();
        Eigen::Matrix4d _icp_goal = Eigen::Matrix4d::Identity ();
        Eigen::Matrix4f _guess_tf;
        tf2::Stamped<tf2::Transform> _tf_las_master_to_base;
        tf2::Stamped<tf2::Transform> _tf_las_slave_to_base;
        
        std::string _path = ament_index_cpp::get_package_share_directory("icp_nav_follow");
        int siz_s,siz_m;
        bool _icp_controlling = false;

        //TF
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        geometry_msgs::msg::TransformStamped _tf_master_slave;
        geometry_msgs::msg::TransformStamped _tf_goal_msg;
        tf2::Stamped<tf2::Transform> _tf_goal_transform;
        tf2::Stamped<tf2::Transform> _tf_laser_base_master;
        tf2::Stamped<tf2::Transform> _tf_laser_base_slave;
        

        std::thread _th_tf;
        std::thread _th_follow;
        std::thread _th_pcl;


        //PID
        std::unique_ptr<control_toolbox::PidROS>  _x_pid;
        std::unique_ptr<control_toolbox::PidROS>  _y_pid;
        std::unique_ptr<control_toolbox::PidROS>  _w_pid;
        std::unique_ptr<control_toolbox::PidROS>  _x_pid_icp;
        std::unique_ptr<control_toolbox::PidROS>  _y_pid_icp;
        std::unique_ptr<control_toolbox::PidROS>  _w_pid_icp;
        //RATE
        std::unique_ptr<rclcpp::Rate> _rate_tf;
        std::unique_ptr<rclcpp::Rate> _rate_follow;

        

    public:

        explicit icp_nav_follow_class();

        void LasCallback_master(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
        void LasCallback_slave(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
        void current_tf_thread();
        void get_tf_goal();
        void tf_follow_thread();
        void init_control();
        void current_pcl_thread();
        void get_tf_laser_base_master();
        void get_tf_laser_base_slave();

        void save_icp_goal(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response>      response);
        void start_icp_goal(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response>      response);
        void stop_icp_goal(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response>      response);

};