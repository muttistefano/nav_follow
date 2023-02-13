#include <icp_nav_follow/icp_nav_follow.h>


using namespace boost::assign;
using namespace boost::adaptors;
using namespace pcl::registration;  
using namespace std::chrono_literals;


inline bool exists_file (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}

void icp_nav_follow_class::LasCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    geometry_msgs::msg::Point32 pnt;
    pnt.z = 0;
    _Lock1.lock();
    _NewPcl.points.clear();
    for (size_t cnt = 0; cnt < msg->ranges.size (); cnt++)
    {
        double ang = msg->angle_min + msg->angle_increment * static_cast<double>(cnt);
        if((msg->ranges[cnt] < FILT_DIST) && (msg->ranges[cnt] > MIN_DIST) && (ang > _cone_min) && (ang < _cone_max))
        {
            _pnt_filt.x =  (cos(ang) * msg->ranges[cnt]);
            _pnt_filt.y =  (sin(ang) * msg->ranges[cnt]);
        }
        else
        {
            _pnt_filt.x = 0;
            _pnt_filt.y = 0;
            // continue;
        }   

        _NewPcl.points.push_back(_pnt_filt);
    }
   
    _NewPcl.header.frame_id = _frame_name;
    _NewPcl.header.stamp    = this->get_clock()->now();

    _pcl_buffer.push_back(_NewPcl);

    _Lock1.unlock();
}

bool icp_nav_follow_class::save_pcl_call()
{
    RCLCPP_INFO(this->get_logger(),"pcl save \n");
    const    pcl::PointCloud<pcl::PointXYZ>::Ptr       cloud_save (new pcl::PointCloud<pcl::PointXYZ>);
    int* _Invalid_Data;

    int cnt = 0;
    while(cnt < LASER_FIXED_FILTER_LENGTH)
    {
        sensor_msgs::msg::LaserScan::ConstSharedPtr LaserFixed;// = ros::topic::waitForMessage<sensor_msgs::LaserScan>(_laser_scan_name,_nh);
        rclcpp::wait_for_message<sensor_msgs::msg::LaserScan::ConstSharedPtr>(LaserFixed,this->shared_from_this(),_laser_scan_name);
        
        int sizfx  = static_cast<int>(LaserFixed->ranges.size());
        if(cnt==0)
        {
            cloud_save->width    = sizfx;
            cloud_save->height   = 1;
            cloud_save->is_dense = true;
            cloud_save->points.clear();
            cloud_save->points.resize (cloud_save->width * cloud_save->height);
            _Invalid_Data = new int[sizfx]();
        }
        for (size_t i = 0; i < cloud_save->points.size (); ++i)
        {
            double ang = LaserFixed->angle_min + (LaserFixed->angle_increment * static_cast<double>(i));
            if((LaserFixed->ranges[i] < FILT_DIST) && (LaserFixed->ranges[i] > MIN_DIST) && (ang > _cone_min) && (ang < _cone_max))
            {
                cloud_save->points[i].x = cloud_save->points[i].x + (cos(ang) * LaserFixed->ranges[i]);
                cloud_save->points[i].y = cloud_save->points[i].y + (sin(ang) * LaserFixed->ranges[i]);
                cloud_save->points[i].z = 0.0;
            }
            else
            {
                _Invalid_Data[i] = _Invalid_Data[i] + 1;
            }

        }
        cnt++;
    }

    RCLCPP_INFO_STREAM(this->get_logger()," points in ::  " << cloud_save->points.size());

    for (size_t i = 0; i < cloud_save->points.size (); ++i)
    {
        if(_Invalid_Data[i] < 2)//WTF
        {
            cloud_save->points[i].x = cloud_save->points[i].x / (double(LASER_FIXED_FILTER_LENGTH) - double(_Invalid_Data[i]));
            cloud_save->points[i].y = cloud_save->points[i].y / (double(LASER_FIXED_FILTER_LENGTH) - double(_Invalid_Data[i]));
        }
        else
        {
            cloud_save->points[i].x = 0.0;
            cloud_save->points[i].y = 0.0;
        }
        
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for (int i = 0; i < (*cloud_save).size(); i++)
    {
        pcl::PointXYZ pt(cloud_save->points[i].x, cloud_save->points[i].y, cloud_save->points[i].z);
        if ((pt.x==0.0) && (pt.y==0.0))
        {
            inliers->indices.push_back(i);
        }
    }
    extract.setInputCloud(cloud_save);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_save);

    std::string path_save = _path + "/data/fixed_0.pcd";
    pcl::io::savePCDFileASCII (path_save, *cloud_save);

    _PclFilt.points.clear();
    geometry_msgs::msg::Point32 _pnt_filt;
    for(const auto& pnt: cloud_save->points)
    {
        _pnt_filt.x = pnt.x;
        _pnt_filt.y = pnt.y;
        _pnt_filt.z = pnt.z;
        _PclFilt.points.push_back(_pnt_filt);
    }

    _PclFilt.header.frame_id = _frame_name;
    _PclFilt.header.stamp    = this->get_clock()->now();
    _filt_pub->publish(_PclFilt);

    // res.saved = true;
    return true;
}

bool icp_nav_follow_class::move_pcl_call()
{

    // _sub         = _nh.subscribe<sensor_msgs::LaserScan>(_laser_scan_name, 1, &icp_nav_follow_class::LasCallback,this);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr       _cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr             _cloud_in  (new pcl::PointCloud<pcl::PointXYZ>);

    rclcpp::Rate _rt(10);
    std::string path_load = _path + "/data/fixed_0.pcd";
    if (exists_file(path_load))
    {
        RCLCPP_INFO_STREAM(this->get_logger(),"pcl file found << " << path_load);
        pcl::io::loadPCDFile<pcl::PointXYZ> (path_load, *_cloud_out);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),"pcl file not found");
        return false;
    }
    
    _PclFilt.points.clear();
    geometry_msgs::msg::Point32 _pnt_filt;
    for(const auto& pnt: _cloud_out->points)
    {
        _pnt_filt.x = pnt.x;
        _pnt_filt.y = pnt.y;
        _pnt_filt.z = pnt.z;
        _PclFilt.points.push_back(_pnt_filt);
    }

    _PclFilt.header.frame_id = _frame_name;
    _PclFilt.header.stamp    = this->get_clock()->now();
    _filt_pub->publish(_PclFilt);

    _icp.setMaximumIterations(15);
    
    _icp.setInputSource (_cloud_in);
    _icp.setInputTarget (_cloud_out);
    
    rclcpp::Time start_time = this->get_clock()->now();

    double outx;
    double outy;
    double outz;
    
    while(rclcpp::ok() && (this->get_clock()->now() - start_time).seconds() <= _timeout  )
    {
        _Lock1.lock();
        int siz = static_cast<int>(_NewPcl.points.size());

        _output_pub->publish(_NewPcl);
        
        //PCL for _icp
        _cloud_in->width    = siz;
        _cloud_in->height   = 1;
        _cloud_in->is_dense = true;
        _cloud_in->points.clear();
        _cloud_in->points.resize (_cloud_in->width * _cloud_in->height);
        for (auto it=_pcl_buffer.begin(); it!=_pcl_buffer.end(); ++it)
        {
            for (size_t i = 0; i < _cloud_in->points.size (); ++i)
            {
                _cloud_in->points[i].x = _cloud_in->points[i].x +  it->points[i].x/double(LASER_SCAN_FILTER_LENGTH);
                _cloud_in->points[i].y = _cloud_in->points[i].y +  it->points[i].y/double(LASER_SCAN_FILTER_LENGTH);
                _cloud_in->points[i].z = 0.0;
            }
        }
        

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        for (int i = 0; i < (*_cloud_in).size(); i++)
        {
            pcl::PointXYZ pt(_cloud_in->points[i].x, _cloud_in->points[i].y, _cloud_in->points[i].z);
            if ((pt.x==0.0) && (pt.y==0.0))
            {
                inliers->indices.push_back(i);
            }
        }
        extract.setInputCloud(_cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*_cloud_in);

        _icp.align (*_cloud_in);
        _Lock1.unlock();
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

        _vis_pub->publish( wrench_out );

        cmd_vel_dock.linear.x  = std::min(std::max(1.0 * _icp_out2(0,3), -0.05), 0.05);
        cmd_vel_dock.linear.y  = std::min(std::max(1.0 * _icp_out2(1,3), -0.05), 0.05);
        cmd_vel_dock.angular.z = std::min(std::max(1.0 *  eul[2]       , -0.05), 0.05);

        outx = _icp_out2(0,3); 
        outy = _icp_out2(1,3); 
        outz = eul[2]; 

        _cmd_vel->publish(cmd_vel_dock);
        
        _rt.sleep();
    }
    // _sub.shutdown();
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
    }
}

void icp_nav_follow_class::get_t_goal()
{
    while(!_tf_buffer->canTransform(_master_frame, _slave_frame, tf2::TimePointZero, 2000ms) && rclcpp::ok()) {
      RCLCPP_INFO(
                    get_logger(), "waiting 200 ms for %s->%s transform to become available",
                    _master_frame.c_str(), _slave_frame.c_str());
      std::this_thread::sleep_for(200ms);
    }
    _t_goal = _tf_buffer->lookupTransform(
            _master_frame, _slave_frame,
            tf2::TimePointZero);

    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg = _t_goal.transform.rotation;
    tf2::fromMsg(quat_msg, quat_tf);
    double r{}, p{}, y{};
    tf2::Matrix3x3 m(quat_tf);
    m.getRPY(r, p, y);

    _w_goal = y;

    RCLCPP_INFO(get_logger(), "Initial transformation stored");
}

void icp_nav_follow_class::follow_thread()
{

    geometry_msgs::msg::TransformStamped _t_copy;
    double x_err,y_err,w_err;

    while(rclcpp::ok())
    {
        {
            std::lock_guard<std::mutex> guard_tf(_tf_mutex);
            _t_copy = _t_master_slave;
        }
        x_err = _t_goal.transform.translation.x - _t_copy.transform.translation.x;
        y_err = _t_goal.transform.translation.y - _t_copy.transform.translation.y;
        

        tf2::Quaternion quat_tf;
        geometry_msgs::msg::Quaternion quat_msg = _t_copy.transform.rotation;
        tf2::fromMsg(quat_msg, quat_tf);
        double r{}, p{}, y{};
        tf2::Matrix3x3 m(quat_tf);
        m.getRPY(r, p, y);

        w_err = y - _w_goal;

        RCLCPP_INFO_STREAM(get_logger(),x_err << " " << y_err << " " << w_err << " " << "\n");
        std::this_thread::sleep_for(50ms);
    }
}

icp_nav_follow_class::icp_nav_follow_class():
Node("icp_nav_follow")
{

//   auto node = parent_node.lock();

//   if (!node->has_parameter("goal_blackboard_id")) {
    // node->declare_parameter("goal_blackboard_id", std::string("goal"));

    this->declare_parameter("laser_scan_topic", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("frame_name"      , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("cmd_vel_topic"   , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("master_frame"    , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("slave_frame"     , rclcpp::ParameterValue(std::string("")));

    this->declare_parameter("timeout",  rclcpp::ParameterValue(10.0));
    this->declare_parameter("cone_min", rclcpp::ParameterValue(1.57));
    this->declare_parameter("cone_max", rclcpp::ParameterValue(4.71));
    this->declare_parameter("xpid",     rclcpp::ParameterValue(1.0));
    this->declare_parameter("ypid",     rclcpp::ParameterValue(1.0));
    this->declare_parameter("zpid",     rclcpp::ParameterValue(1.0));


    _laser_scan_name = this->get_parameter("laser_scan_topic").get_parameter_value().get<std::string>();
    _frame_name      = this->get_parameter("frame_name").get_parameter_value().get<std::string>();
    _cmd_vel_topic   = this->get_parameter("cmd_vel_topic").get_parameter_value().get<std::string>();
    
    _master_frame    = this->get_parameter("master_frame").get_parameter_value().get<std::string>();
    _slave_frame     = this->get_parameter("slave_frame").get_parameter_value().get<std::string>();

    _timeout         = this->get_parameter("timeout").get_parameter_value().get<double>();
    _cone_min        = this->get_parameter("cone_min").get_parameter_value().get<double>();
    _cone_max        = this->get_parameter("cone_max").get_parameter_value().get<double>();
    _xP              = this->get_parameter("xpid").get_parameter_value().get<double>();
    _yP              = this->get_parameter("ypid").get_parameter_value().get<double>();
    _zP              = this->get_parameter("zpid").get_parameter_value().get<double>();

    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    _pcl_buffer = boost::circular_buffer<sensor_msgs::msg::PointCloud> (LASER_SCAN_FILTER_LENGTH);




    // _sub         = _nh.subscribe<sensor_msgs::LaserScan>(_laser_scan_name, 1, &icp_nav_follow_class::LasCallback,this);

    _sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      _laser_scan_name, 1, std::bind(&icp_nav_follow_class::LasCallback, this, std::placeholders::_1));

    _output_pub  = this->create_publisher<sensor_msgs::msg::PointCloud>("pcl_input", 1);
    _filt_pub    = this->create_publisher<sensor_msgs::msg::PointCloud>("pcl_reference", 1);
    _vis_pub     = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench", 1);
    _cmd_vel     = this->create_publisher<geometry_msgs::msg::Twist>(_cmd_vel_topic, 1);
    // _output_pub  = _nh.advertise<sensor_msgs::PointCloud>("pcl_input", 1);
    // _filt_pub    = _nh.advertise<sensor_msgs::PointCloud>("pcl_reference", 1,true);
    // _vis_pub     = _nh.advertise<geometry_msgs::WrenchStamped>( "wrench", 1 );
    // _cmd_vel     = _nh.advertise<geometry_msgs::Twist>(_cmd_vel_topic,1);

    this->get_t_goal();
    _th_tf     = std::thread(&icp_nav_follow_class::tf_thread, this);
    _th_follow = std::thread(&icp_nav_follow_class::follow_thread, this);

}


int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto nodeState    = std::make_shared<icp_nav_follow_class>();
    executor.add_node(nodeState);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
