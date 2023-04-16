#include <nav_follow/nav_follow.h>


using namespace boost::assign;
using namespace boost::adaptors;
using namespace pcl::registration;  
using namespace std::chrono_literals;

nav_follow_class::nav_follow_class():
rclcpp_lifecycle::LifecycleNode("nav_follow")
{

    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    _pcl_buffer_master = boost::circular_buffer<sensor_msgs::msg::PointCloud> (LASER_SCAN_FILTER_LENGTH);
    _pcl_buffer_slave  = boost::circular_buffer<sensor_msgs::msg::PointCloud> (LASER_SCAN_FILTER_LENGTH);

    _rate_tf     = std::make_unique<rclcpp::Rate>(20);
    _rate_follow = std::make_unique<rclcpp::Rate>(20);

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
nav_follow_class::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Configuring");
    param_listener_ = std::make_shared<nav_follow::ParamListener>(this->shared_from_this());
    if (!param_listener_)
    {
        RCLCPP_FATAL(this->get_logger(), "Error encountered during init");
    }
    _params = param_listener_->get_params();

    if(!_params.enable_tf && !_params.enable_vel_feedforward && !_params.enable_icp  )
    {
        RCLCPP_FATAL(this->get_logger(), "No controllers enabled");
        RCLCPP_FATAL(this->get_logger(), "Enable at least one: enable_tf,enable_vel_feedforward,enable_icp");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    
    _th_tf     = std::thread(&nav_follow_class::current_tf_thread, this);

    if(_params.enable_tf  )
    {
        this->get_tf_goal();
    }

    if(_params.enable_vel_feedforward  )
    {
        _cmd_vel_feed = this->create_subscription<geometry_msgs::msg::Twist>(
            _params.cmd_vel_topic_master, rclcpp::SensorDataQoS(), std::bind(&nav_follow_class::cmd_vel_feedforward, this, std::placeholders::_1));
    }

    if(_params.enable_icp  )
    {
        RCLCPP_WARN(this->get_logger(), "ICP CONTROL IS HIGHLY UNSTABLE AND EXPERIMENTAL!!! USE IT CAREFULLY");   

        _sub_master_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
            _params.laser_scan_master, rclcpp::SensorDataQoS(), std::bind(&nav_follow_class::LasCallback_master, this, std::placeholders::_1));

        _sub_slave_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
            _params.laser_scan_slave, rclcpp::SensorDataQoS(), std::bind(&nav_follow_class::LasCallback_slave, this, std::placeholders::_1));

        _pcl_master_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("master_pcl", 1);
        _pcl_reg_pub    = this->create_publisher<sensor_msgs::msg::PointCloud2>("reg_pcl", 1);
        _pcl_slave_pub  = this->create_publisher<sensor_msgs::msg::PointCloud2>("slave_pcl" , 1);

        _save_icp_goal_srv  = this->create_service<std_srvs::srv::Trigger>("save_icp_goal" , std::bind(&nav_follow_class::save_icp_goal , this, std::placeholders::_1,std::placeholders::_2));
        _start_icp_goal_srv = this->create_service<std_srvs::srv::Trigger>("start_icp_goal", std::bind(&nav_follow_class::start_icp_goal, this, std::placeholders::_1,std::placeholders::_2));
        _stop_icp_goal_srv  = this->create_service<std_srvs::srv::Trigger>("stop_icp_goal" , std::bind(&nav_follow_class::stop_icp_goal , this, std::placeholders::_1,std::placeholders::_2));

        this->get_tf_laser_base_master();
        this->get_tf_laser_base_slave();

    }


    _vis_pub     = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench", 1);
    _cmd_vel     = this->create_publisher<geometry_msgs::msg::Twist>(_params.cmd_vel_topic, 1);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
nav_follow_class::on_activate(const rclcpp_lifecycle::State &)
{

    if(_params.enable_tf  )
    {
        _x_pid = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"x_pid");
        _y_pid = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"y_pid");
        _w_pid = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"w_pid");

        _x_pid->initPid();
        _y_pid->initPid();
        _w_pid->initPid();
        this->_tf_controlling = true;
        _th_follow = std::thread(&nav_follow_class::tf_follow_thread, this);
    }


    if(_params.enable_icp  )
    {
        _x_pid_icp = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"x_pid_icp");
        _y_pid_icp = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"y_pid_icp");
        _w_pid_icp = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"w_pid_icp");

        _x_pid_icp->initPid();
        _y_pid_icp->initPid();
        _w_pid_icp->initPid();
        this->_icp_controlling = true;
        _th_pcl    = std::thread(&nav_follow_class::current_pcl_thread, this);
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
nav_follow_class::on_deactivate(const rclcpp_lifecycle::State &)
{
    this->_tf_controlling = false;
    this->_icp_controlling = false;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
nav_follow_class::on_shutdown(const rclcpp_lifecycle::State &)
{
    this->_tf_controlling = false;
    this->_icp_controlling = false;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
nav_follow_class::on_cleanup(const rclcpp_lifecycle::State &)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

//TODO : merge cmd vels
//TODO : service for tf save,start and stop
//TODO : Futures to stop threads
//TODO : Destructor in shutdown ???
//TODO : Fill cleanup
//TODO : Control QOS

int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    // rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto nodeState    = std::make_shared<nav_follow_class>();
    // executor.add_node(nodeState);
    executor.add_node(nodeState->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
