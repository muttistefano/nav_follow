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


constexpr int   LASER_FIXED_FILTER_LENGTH  = 10;
constexpr int   LASER_SCAN_FILTER_LENGTH   = 3;
constexpr double FILT_DIST                 = 3.0;
constexpr double MIN_DIST                  = 0.3;

class icp_nav_follow_class : public rclcpp::Node
{
    private:

        std::mutex   _Lock1;
        std::string  _laser_scan_name,_frame_name,_cmd_vel_topic;
        std::string  _slave_frame,_master_frame;
        

        double _timeout,_cone_min,_cone_max;
    
        double _xP,_yP,_zP;

        //ROS

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    _sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr      _output_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr      _filt_pub;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr _vis_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         _cmd_vel;


        geometry_msgs::msg::Twist cmd_vel_dock;


        //PCL
        boost::circular_buffer<sensor_msgs::msg::PointCloud>  _pcl_buffer;//

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> _icp;

        sensor_msgs::msg::PointCloud _NewPcl;
        sensor_msgs::msg::PointCloud _PclFilt;
        sensor_msgs::msg::PointCloud _TPcl;
        geometry_msgs::msg::Point32  _pnt_filt;

        sensor_msgs::msg::LaserScan::ConstSharedPtr _LaserFixed;//
        
        Eigen::Matrix4d _icp_out  = Eigen::Matrix4d::Identity ();
        Eigen::Matrix4d _icp_out2 = Eigen::Matrix4d::Identity ();
        
        std::string _path = ament_index_cpp::get_package_share_directory("icp_nav_follow");

        std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        geometry_msgs::msg::TransformStamped _t_master_slave;
        geometry_msgs::msg::TransformStamped _t_goal;
        tf2::Stamped<tf2::Transform> _t_goal_transform;
        double _w_goal = 0;
        std::mutex _tf_mutex;

        std::thread _th_tf;
        std::thread _th_follow;


        control_toolbox::PidROS*  _x_pid;
        control_toolbox::PidROS*  _y_pid;
        control_toolbox::PidROS*  _w_pid;

        rclcpp::Rate* _rate_tf;
        rclcpp::Rate* _rate_follow;

        

    public:

        explicit icp_nav_follow_class();

        void LasCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
        void tf_thread();
        void get_t_goal();
        void follow_thread();
        void init_control();
       

        // bool save_pcl_call(icp_nav_follow::save_pcl::Request  &req, icp_nav_follow::save_pcl::Response &res);
        // bool move_pcl_call(icp_nav_follow::move_to_pcl::Request  &req,icp_nav_follow::move_to_pcl::Response &res);
        bool save_pcl_call();
        bool move_pcl_call();

};