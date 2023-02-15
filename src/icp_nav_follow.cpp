#include <icp_nav_follow/icp_nav_follow.h>


using namespace boost::assign;
using namespace boost::adaptors;
using namespace pcl::registration;  
using namespace std::chrono_literals;


inline bool exists_file (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}

void icp_nav_follow_class::LasCallback_master(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    geometry_msgs::msg::Point32 pnt;
    pnt.z = 0;
    std::lock_guard<std::mutex> guard_pcl_master(_lock_pcl_master);
    _NewPcl_master.points.clear();
    for (size_t cnt = 0; cnt < msg->ranges.size (); cnt++)
    {
        double ang = msg->angle_min + msg->angle_increment * static_cast<double>(cnt);
        if ((ang < -1.57) || (ang > 1.57))
        {
            _pnt_filt_master.x =  0.0;
            _pnt_filt_master.y =  0.0;
        }
        else
        {
            _pnt_filt_master.x =  (cos(ang) * msg->ranges[cnt]);
            _pnt_filt_master.y =  (sin(ang) * msg->ranges[cnt]);
        }
        _NewPcl_master.points.push_back(_pnt_filt_master);
    }
   
    _NewPcl_master.header.frame_id = _frame_name;
    _NewPcl_master.header.stamp    = this->get_clock()->now();

    // _pcl_buffer_master.push_back(std::move(_NewPcl_master));
    _pcl_buffer_master.push_back(_NewPcl_master);
    // RCLCPP_ERROR(this->get_logger(),"ASDASDMASTER");

}

void icp_nav_follow_class::LasCallback_slave(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    geometry_msgs::msg::Point32 pnt;
    pnt.z = 0;
    std::lock_guard<std::mutex> guard_pcl_slave(_lock_pcl_slave);
    _NewPcl_slave.points.clear();
    for (size_t cnt = 0; cnt < msg->ranges.size (); cnt++)
    {
        double ang = msg->angle_min + msg->angle_increment * static_cast<double>(cnt);
        if ((ang < -1.57) || (ang > 1.57))
        {
            _pnt_filt_slave.x =  0.0;
            _pnt_filt_slave.y =  0.0;
        }
        else
        {
            _pnt_filt_slave.x =  (cos(ang) * msg->ranges[cnt]);
            _pnt_filt_slave.y =  (sin(ang) * msg->ranges[cnt]);
        }
        _NewPcl_slave.points.push_back(_pnt_filt_slave);
    }
   
    _NewPcl_slave.header.frame_id = _frame_name;
    _NewPcl_slave.header.stamp    = this->get_clock()->now();

    // _pcl_buffer_slave.push_back(std::move(_NewPcl_slave));
    _pcl_buffer_slave.push_back(_NewPcl_slave);
    // RCLCPP_ERROR(this->get_logger(),"ASDASDSLAVE");

}


bool icp_nav_follow_class::move_pcl_call()
{

    std::this_thread::sleep_for(2000ms);
    RCLCPP_ERROR(this->get_logger(),"PCL Started");
    const pcl::PointCloud<pcl::PointXYZ>::Ptr    _cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr    _cloud_in  (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointIndices::Ptr inliers_master(new pcl::PointIndices());
    pcl::PointIndices::Ptr inliers_slave(new pcl::PointIndices());

    pcl::ExtractIndices<pcl::PointXYZ> extract_master;
    pcl::ExtractIndices<pcl::PointXYZ> extract_slave;

    rclcpp::Rate _rt(20);

    _icp.setMaximumIterations (50);
    _icp.setTransformationEpsilon (1e-8);
    // _icp.setEuclideanFitnessEpsilon (1);
    // _icp.setRANSACOutlierRejectionThreshold (1.5);
    // _icp.setInputSource (_cloud_in);
    // _icp.setInputTarget (_cloud_out);
    
    double outx;
    double outy;
    double outz;

    RCLCPP_ERROR_STREAM(this->get_logger(),"_pcl_buffer_slave size : "  << _pcl_buffer_slave.size());
    RCLCPP_ERROR_STREAM(this->get_logger(),"_pcl_buffer_master size : " << _pcl_buffer_master.size());

    while(rclcpp::ok())
    {

        std::future<void> slave_pcl = std::async(std::launch::async, 
                                        [this, &_cloud_in,&inliers_slave,&extract_slave]() {

                                        std::lock_guard<std::mutex> guard_pcl_slave(_lock_pcl_slave);
                                        int siz = static_cast<int>(this->_NewPcl_slave.points.size());
                                        //PCL for _icp
                                        _cloud_in->width    = siz;
                                        _cloud_in->height   = 1;
                                        _cloud_in->is_dense = true;
                                        _cloud_in->points.clear();
                                        std::set<size_t> indexes;
                                        _cloud_in->points.resize (_cloud_in->width * _cloud_in->height);
                                        for (auto it=_pcl_buffer_slave.begin(); it!=_pcl_buffer_slave.end(); ++it)
                                        {
                                            for (size_t i = 0; i < _cloud_in->points.size (); ++i)
                                            {
                                                if ((it->points[i].x == 0.0) || (it->points[i].y == 0.0))
                                                {
                                                    indexes.insert(i);
                                                    continue;
                                                }
                                                _cloud_in->points[i].x = _cloud_in->points[i].x +  it->points[i].x/double(LASER_SCAN_FILTER_LENGTH);
                                                _cloud_in->points[i].y = _cloud_in->points[i].y +  it->points[i].y/double(LASER_SCAN_FILTER_LENGTH);
                                                _cloud_in->points[i].z = 0.0;
                                            }
                                        }
                                        

                                        inliers_slave->indices.clear();
                                    
                                        for (auto it = indexes.begin(); it != indexes.end(); ++it) {
                                            inliers_slave->indices.push_back(*it);
                                        }
                                        extract_slave.setInputCloud(_cloud_in);
                                        extract_slave.setIndices(inliers_slave);
                                        extract_slave.setNegative(true);
                                        extract_slave.filter(*_cloud_in);
                                        }
                                    );


        std::future<void> master_pcl = std::async(std::launch::async, 
                                        [this, &_cloud_out,&inliers_master,&extract_master]() {

                                        std::lock_guard<std::mutex> guard_pcl_master(_lock_pcl_slave);
                                        int siz = static_cast<int>(this->_NewPcl_master.points.size());
                                        inliers_master->indices.clear();
                                        std::set<size_t> indexes;
                                        //PCL for _icp
                                        _cloud_out->width    = siz;
                                        _cloud_out->height   = 1;
                                        _cloud_out->is_dense = true;
                                        _cloud_out->points.clear();
                                        _cloud_out->points.resize (_cloud_out->width * _cloud_out->height);
                                        for (auto it=_pcl_buffer_master.begin(); it!=_pcl_buffer_master.end(); ++it)
                                        {
                                            for (size_t i = 0; i < _cloud_out->points.size (); ++i)
                                            {
                                                if ((it->points[i].x == 0.0) || (it->points[i].y == 0.0))
                                                {
                                                    indexes.insert(i);
                                                    continue;
                                                }
                                                _cloud_out->points[i].x = _cloud_out->points[i].x +  it->points[i].x/double(LASER_SCAN_FILTER_LENGTH);
                                                _cloud_out->points[i].y = _cloud_out->points[i].y +  it->points[i].y/double(LASER_SCAN_FILTER_LENGTH);
                                                _cloud_out->points[i].z = 0.0;
                                            }
                                        }
                                        
                                        for (auto it = indexes.begin(); it != indexes.end(); ++it) {
                                            inliers_master->indices.push_back(*it);
                                        }

                                        extract_master.setInputCloud(_cloud_out);
                                        extract_master.setIndices(inliers_master);
                                        extract_master.setNegative(true);
                                        extract_master.filter(*_cloud_out);
                                        }
                                    );


        slave_pcl.get();
        master_pcl.get();

        // std::cout << "in " << _cloud_in->points.size() << std::flush << "\n"; 
        // std::cout << "out " << _cloud_out->points.size() << std::flush <<"\n"; 

        sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_master = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_slave  = std::make_shared<sensor_msgs::msg::PointCloud2>();

        pcl::toROSMsg(*_cloud_in, *pc2_msg_slave);
        pcl::toROSMsg(*_cloud_out, *pc2_msg_master);
        pc2_msg_slave->header.frame_id = "azrael/laser";
        pc2_msg_master->header.frame_id = "omron/base_link";

        pc2_msg_master->header.stamp = now();
        pc2_msg_slave->header.stamp = now();
        
        _pcl_master_pub->publish(*pc2_msg_master);
        _pcl_slave_pub->publish(*pc2_msg_slave);

        Eigen::Matrix4f guess = tf2::transformToEigen(_t_master_slave).matrix().cast<float>();

        // _icp.setInputSource (_cloud_in);
        // _icp.setInputTarget (_cloud_out);
        _icp.setInputSource (_cloud_out);
        _icp.setInputTarget (_cloud_in);
        // _icp.align (*_cloud_in,guess);
        _icp.align (*_cloud_out,guess);
        
        _icp_out2 = _icp.getFinalTransformation().cast<double>() ;
        geometry_msgs::msg::WrenchStamped wrench_out;
        wrench_out.header.frame_id = _frame_name;
        wrench_out.header.stamp    = this->get_clock()->now();
        wrench_out.wrench.force.x  = _icp_out2(0,3);
        wrench_out.wrench.force.y  = _icp_out2(1,3);
        wrench_out.wrench.force.z  = _icp_out2(2,3);
        Eigen::Affine3d b;
        b.matrix() = _icp_out2;
        auto qua = Eigen::Quaterniond(b.linear());
        
        Eigen::Vector3d eul = qua.toRotationMatrix().eulerAngles(0, 1, 2);
        wrench_out.wrench.torque.x = eul[0];
        wrench_out.wrench.torque.y = eul[1];
        wrench_out.wrench.torque.z = eul[2];

        RCLCPP_INFO_STREAM(this->get_logger(),"ICP " << _icp_out2(0,3) << " " << _icp_out2(1,3) << " " << eul[2]);

        _vis_pub->publish( wrench_out );

        // cmd_vel_dock.twist.linear.x  = std::min(std::max(1.0 * _icp_out2(0,3), -0.05), 0.05);
        // cmd_vel_dock.twist.linear.y  = std::min(std::max(1.0 * _icp_out2(1,3), -0.05), 0.05);
        // cmd_vel_dock.twist.angular.z = std::min(std::max(1.0 *  eul[2]       , -0.05), 0.05);

        // outx = _icp_out2(0,3); 
        // outy = _icp_out2(1,3); 
        // outz = eul[2]; 

        // _cmd_vel->publish(cmd_vel_dock);
        
        _rt.sleep();
    }
    // res.success = true;
    return true;
}

void icp_nav_follow_class::tf_thread()
{
    while(rclcpp::ok())
    {
        try {
            std::lock_guard<std::mutex> guard_tf(_tf_mutex);
            _t_master_slave = _tf_buffer->lookupTransform(
            _master_frame, _slave_frame,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            _master_frame.c_str(), _slave_frame.c_str(), ex.what());
            return;
        }
        // std::this_thread::sleep_for(50ms);
        _rate_tf->sleep();
    }
}

void icp_nav_follow_class::get_t_goal()
{
    tf2::Stamped<tf2::Transform> stamped_transform;
    while(!_tf_buffer->canTransform(_master_frame, _slave_frame, tf2::TimePointZero, 2000ms) && rclcpp::ok()) {
      RCLCPP_INFO(
                    get_logger(), "waiting 2000 ms for %s->%s transform to become available",
                    _master_frame.c_str(), _slave_frame.c_str());
      std::this_thread::sleep_for(200ms);
    }
    _t_goal = _tf_buffer->lookupTransform(
            _master_frame, _slave_frame,
            tf2::TimePointZero);

    tf2::fromMsg(_t_goal, _t_goal_transform);

    RCLCPP_INFO(get_logger(), "Initial transformation stored");
}

void icp_nav_follow_class::follow_thread()
{

    tf2::Stamped<tf2::Transform> stamped_transform_now;
    geometry_msgs::msg::Twist vel_cmd;
    double x_err,y_err,w_err;
    double x_cmd,y_cmd,w_cmd;

    while(rclcpp::ok())
    {
        {
            std::lock_guard<std::mutex> guard_tf(_tf_mutex);
            tf2::fromMsg(_t_master_slave, stamped_transform_now);
        }

        tf2::Transform err =  stamped_transform_now.inverse() * _t_goal_transform;

        x_err = err.getOrigin().getX();
        y_err = err.getOrigin().getY();

        double r{}, p{}, y{};
        tf2::Matrix3x3 m(err.getRotation());
        m.getRPY(r, p, y);
        w_err = y;


        RCLCPP_INFO_STREAM(get_logger(),"err: " << x_err << " " << y_err << " " << w_err << " " << "\n");
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

icp_nav_follow_class::icp_nav_follow_class():
Node("icp_nav_follow")
{

    this->declare_parameter("laser_scan_master", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("laser_scan_slave", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("frame_name"      , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("cmd_vel_topic"   , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("master_frame"    , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("slave_frame"     , rclcpp::ParameterValue(std::string("")));


    _laser_scan_master = this->get_parameter("laser_scan_master").get_parameter_value().get<std::string>();
    _laser_scan_slave = this->get_parameter("laser_scan_slave").get_parameter_value().get<std::string>();
    _frame_name      = this->get_parameter("frame_name").get_parameter_value().get<std::string>();
    _cmd_vel_topic   = this->get_parameter("cmd_vel_topic").get_parameter_value().get<std::string>();
    
    _master_frame    = this->get_parameter("master_frame").get_parameter_value().get<std::string>();
    _slave_frame     = this->get_parameter("slave_frame").get_parameter_value().get<std::string>();


    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    _pcl_buffer_master = boost::circular_buffer<sensor_msgs::msg::PointCloud> (LASER_SCAN_FILTER_LENGTH);
    _pcl_buffer_slave  = boost::circular_buffer<sensor_msgs::msg::PointCloud> (LASER_SCAN_FILTER_LENGTH);

    _sub_master = this->create_subscription<sensor_msgs::msg::LaserScan>(
    _laser_scan_master, rclcpp::SensorDataQoS(), std::bind(&icp_nav_follow_class::LasCallback_master, this, std::placeholders::_1));

    _sub_slave = this->create_subscription<sensor_msgs::msg::LaserScan>(
    _laser_scan_slave, rclcpp::SensorDataQoS(), std::bind(&icp_nav_follow_class::LasCallback_slave, this, std::placeholders::_1));

    _vis_pub     = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench", 1);
    _cmd_vel     = this->create_publisher<geometry_msgs::msg::Twist>(_cmd_vel_topic, 1);

    _pcl_master_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("master_pcl", 1);
    _pcl_slave_pub  = this->create_publisher<sensor_msgs::msg::PointCloud2>("slave_pcl" , 1);

    _rate_tf     = new rclcpp::Rate(20);
    _rate_follow = new rclcpp::Rate(20);

    // this->get_t_goal();
    // _th_tf     = std::thread(&icp_nav_follow_class::tf_thread, this);

}

void icp_nav_follow_class::init_control()
{
    _x_pid = new control_toolbox::PidROS(this->shared_from_this(),"x_pid");
    _y_pid = new control_toolbox::PidROS(this->shared_from_this(),"y_pid");
    _w_pid = new control_toolbox::PidROS(this->shared_from_this(),"w_pid");

    _x_pid->initPid();
    _y_pid->initPid();
    _w_pid->initPid();

    // _x_pid_a->reset();
    // _y_pid->reset();
    // _w_pid->reset();

    // _th_follow = std::thread(&icp_nav_follow_class::follow_thread, this);
    _th_pcl    = std::thread(&icp_nav_follow_class::move_pcl_call, this);
}


int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto nodeState    = std::make_shared<icp_nav_follow_class>();
    nodeState->init_control();
    executor.add_node(nodeState);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
