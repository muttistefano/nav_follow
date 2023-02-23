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
    std::scoped_lock<std::mutex> guard_pcl_master(_lock_pcl_master);
    _NewPcl_master.points.clear();
    for (size_t cnt = 0; cnt < msg->ranges.size (); cnt++)
    {
        double ang = msg->angle_min + msg->angle_increment * static_cast<double>(cnt);
        if ((ang < -1.57) || (ang > 1.57) || (msg->ranges[cnt] < 2))
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
   
    _NewPcl_master.header.frame_id = _frame_name_laser_master;
    _NewPcl_master.header.stamp    = this->get_clock()->now();

    _pcl_buffer_master.push_back(_NewPcl_master);
    // RCLCPP_ERROR_STREAM(this->get_logger(),"ASDASDMASTER" << _NewPcl_master.points.size() );

}

void icp_nav_follow_class::LasCallback_slave(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    geometry_msgs::msg::Point32 pnt;
    pnt.z = 0;
    std::scoped_lock<std::mutex> guard_pcl_slave(_lock_pcl_slave);
    _NewPcl_slave.points.clear();
    for (size_t cnt = 0; cnt < msg->ranges.size (); cnt++)
    {
        double ang = msg->angle_min + msg->angle_increment * static_cast<double>(cnt);
        if ((ang < -1.57) || (ang > 1.57) || (msg->ranges[cnt] < 2))
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
   
    _NewPcl_slave.header.frame_id = _frame_name_laser_slave;
    _NewPcl_slave.header.stamp    = this->get_clock()->now();

    _pcl_buffer_slave.push_back(_NewPcl_slave);
    // RCLCPP_ERROR_STREAM(this->get_logger(),"ASDASDSLAVE"  << _NewPcl_slave.points.size());

}

void icp_nav_follow_class::current_pcl_thread()
{

    while(rclcpp::ok())
    {
        bool done_wait = false;
        {
            std::scoped_lock pcl(_lock_pcl_master,_lock_pcl_slave);
            done_wait = (_pcl_buffer_slave.size() == LASER_SCAN_FILTER_LENGTH) && (_pcl_buffer_master.size() == LASER_SCAN_FILTER_LENGTH);
        }
        if (done_wait){break;}
        std::this_thread::sleep_for(200ms);
    }

    RCLCPP_INFO_STREAM(this->get_logger(),"_pcl_buffer_slave size : "  << _pcl_buffer_slave.size());
    RCLCPP_INFO_STREAM(this->get_logger(),"_pcl_buffer_master size : " << _pcl_buffer_master.size());

    
    const pcl::PointCloud<pcl::PointXYZ>::Ptr    _cloud_master = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()  ;
    const pcl::PointCloud<pcl::PointXYZ>::Ptr    _cloud_slave  = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()  ;

    pcl::PointIndices::Ptr inliers_master = std::make_shared<pcl::PointIndices>();
    pcl::PointIndices::Ptr inliers_slave  = std::make_shared<pcl::PointIndices>();

    pcl::ExtractIndices<pcl::PointXYZ> extract_master;
    pcl::ExtractIndices<pcl::PointXYZ> extract_slave;

    rclcpp::Rate _rt(20);

    // {
    //     std::scoped_lock<std::mutex> guard_pcl_slave(_lock_pcl_slave);
    //     siz_s = static_cast<int>(this->_NewPcl_slave.points.size());
    // }

    // _cloud_slave->width    = siz_s;
    // _cloud_slave->height   = 1;
    // _cloud_slave->is_dense = true;
    // _cloud_slave->points.resize (_cloud_slave->width * _cloud_slave->height);

    // {
    //     std::scoped_lock<std::mutex> guard_pcl_master(_lock_pcl_master);
    //     siz_m = static_cast<int>(this->_NewPcl_master.points.size());
    // }

    // _cloud_master->width    = siz_m;
    // _cloud_master->height   = 1;
    // _cloud_master->is_dense = true;
    // _cloud_master->points.resize (_cloud_master->width * _cloud_master->height);
    
    double outx;
    double outy;
    double outz;

    while(rclcpp::ok())
    {

        std::future<void> slave_pcl = std::async(std::launch::async, 
                                        [this, &_cloud_slave,&inliers_slave,&extract_slave]() {

                                        std::lock_guard<std::mutex> guard_pcl_slave(_lock_pcl_slave);
                                        int siz = static_cast<int>(this->_NewPcl_slave.points.size());
                                        //PCL for _icp
                                        _cloud_slave->width    = siz;
                                        _cloud_slave->height   = 1;
                                        _cloud_slave->is_dense = true;
                                        _cloud_slave->points.clear();
                                        std::set<size_t> indexes;
                                        _cloud_slave->points.resize (_cloud_slave->width * _cloud_slave->height);
                                        for (auto it=_pcl_buffer_slave.begin(); it!=_pcl_buffer_slave.end(); ++it)
                                        {
                                            for (size_t i = 0; i < _cloud_slave->points.size (); ++i)
                                            {
                                                if ((it->points[i].x == 0.0) || (it->points[i].y == 0.0))
                                                {
                                                    indexes.insert(i);
                                                    continue;
                                                }
                                                _cloud_slave->points[i].x = _cloud_slave->points[i].x +  it->points[i].x/double(LASER_SCAN_FILTER_LENGTH);
                                                _cloud_slave->points[i].y = _cloud_slave->points[i].y +  it->points[i].y/double(LASER_SCAN_FILTER_LENGTH);
                                                _cloud_slave->points[i].z = 0.0;
                                            }
                                        }
                                        

                                        inliers_slave->indices.clear();
                                    
                                        for (auto it = indexes.begin(); it != indexes.end(); ++it) {
                                            inliers_slave->indices.push_back(*it);
                                        }
                                        extract_slave.setInputCloud(_cloud_slave);
                                        extract_slave.setIndices(inliers_slave);
                                        extract_slave.setNegative(true);
                                        extract_slave.filter(*_cloud_slave);
                                        }
                                    );


        std::future<void> master_pcl = std::async(std::launch::async, 
                                        [this, &_cloud_master,&inliers_master,&extract_master]() {

                                        std::lock_guard<std::mutex> guard_pcl_master(_lock_pcl_master);
                                        int siz = static_cast<int>(this->_NewPcl_master.points.size());
                                        inliers_master->indices.clear();
                                        std::set<size_t> indexes;
                                        //PCL for _icp
                                        _cloud_master->width    = siz;
                                        _cloud_master->height   = 1;
                                        _cloud_master->is_dense = true;
                                        _cloud_master->points.clear();
                                        _cloud_master->points.resize (_cloud_master->width * _cloud_master->height);
                                        for (auto it=_pcl_buffer_master.begin(); it!=_pcl_buffer_master.end(); ++it)
                                        {
                                            for (size_t i = 0; i < _cloud_master->points.size (); ++i)
                                            {
                                                if ((it->points[i].x == 0.0) || (it->points[i].y == 0.0))
                                                {
                                                    indexes.insert(i);
                                                    continue;
                                                }
                                                _cloud_master->points[i].x = _cloud_master->points[i].x +  it->points[i].x/double(LASER_SCAN_FILTER_LENGTH);
                                                _cloud_master->points[i].y = _cloud_master->points[i].y +  it->points[i].y/double(LASER_SCAN_FILTER_LENGTH);
                                                _cloud_master->points[i].z = 0.0;
                                            }
                                        }
                                        
                                        for (auto it = indexes.begin(); it != indexes.end(); ++it) {
                                            inliers_master->indices.push_back(*it);
                                        }

                                        extract_master.setInputCloud(_cloud_master);
                                        extract_master.setIndices(inliers_master);
                                        extract_master.setNegative(true);
                                        extract_master.filter(*_cloud_master);
                                        }
                                    );


        slave_pcl.get();
        master_pcl.get();

        // std::cout << "in " << _cloud_slave->points.size() << std::flush << "\n"; 
        // std::cout << "out " << _cloud_master->points.size() << std::flush <<"\n"; 

        auto pc2_msg_master = std::make_shared<sensor_msgs::msg::PointCloud2>();
        auto pc2_msg_slave  = std::make_shared<sensor_msgs::msg::PointCloud2>();
        auto pc2_reg        = std::make_shared<sensor_msgs::msg::PointCloud2>();

        pcl::toROSMsg(*_cloud_slave, *pc2_msg_slave);
        pcl::toROSMsg(*_cloud_master, *pc2_msg_master);

        pc2_msg_slave->header.frame_id = _frame_name_laser_slave;
        pc2_msg_master->header.frame_id = _frame_name_laser_master;

        pc2_msg_master->header.stamp = now();
        pc2_msg_slave->header.stamp = now();
        
        _pcl_master_pub->publish(*pc2_msg_master);
        _pcl_slave_pub->publish(*pc2_msg_slave);

        {
            std::scoped_lock<std::mutex> guard_tf(_tf_mutex);
            _guess_tf = tf2::transformToEigen(_tf_master_slave).matrix().cast<float>();

        }

        _icp.setMaximumIterations (_icp_iterations);
        _icp.setTransformationEpsilon (_icp_TransformationEpsilon);
        _icp.setEuclideanFitnessEpsilon (_icp_EuclideanFitnessEpsilon);
        _icp.setRANSACOutlierRejectionThreshold (_icp_RANSACOutlierRejectionThreshold);
        _icp.setMaxCorrespondenceDistance(_icp_MaxCorrespondenceDistance);


        //TODO WRONG guess, guess to laser not base footprint
        {
            std::scoped_lock guard_icp(_icp_mutex,_lock_pcl_slave,_lock_pcl_master);
            _icp.setInputSource (_cloud_master);
            _icp.setInputTarget (_cloud_slave);
            // RCLCPP_INFO_STREAM(this->get_logger(),_guess_tf);
            _icp.align (*_cloud_master,_guess_tf.inverse());
            pcl::toROSMsg(*_cloud_master, *pc2_reg);
            pc2_reg->header.frame_id = _frame_name_laser_slave;
            pc2_reg->header.stamp = now();
            _pcl_reg_pub->publish(*pc2_reg);
            // TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ> pd;
            // Eigen::Matrix4f pd_out;
            // pd.estimateRigidTransformation(*_cloud_master,*_cloud_slave,pd_out);
            // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(),1000,"PD: " << pd_out);

            //TODO pub output
            //TODO limita angolo perchè smatta a 3.14
            _icp_current.matrix() = _icp.getFinalTransformation().cast<double>() ;
            // _icp_current.col(2).setZero();
            // _icp_current.row(2).setZero();
            // _icp_current(2,2)   = 1.0;
        }


        geometry_msgs::msg::WrenchStamped wrench_out;
        wrench_out.header.frame_id = _frame_name_laser_slave;
        wrench_out.header.stamp    = this->get_clock()->now();
        wrench_out.wrench.force.x  = _icp_current(0,3);
        wrench_out.wrench.force.y  = _icp_current(1,3);
        wrench_out.wrench.force.z  = _icp_current(2,3);
        auto qua = Eigen::Quaterniond(_icp_current.linear());
        
        Eigen::Vector3d eul = qua.toRotationMatrix().eulerAngles(0, 1, 2);
        wrench_out.wrench.torque.x = eul[0];
        wrench_out.wrench.torque.y = eul[1];
        wrench_out.wrench.torque.z = eul[2];

        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(),1000,"ICP " << _icp_current(0,3) << " " << _icp_current(1,3) << " " << eul[2]);

        _vis_pub->publish( wrench_out );

        // if(_icp_controlling)
        // {
            double x_err,y_err,w_err;
            double x_cmd,y_cmd,w_cmd;
            geometry_msgs::msg::Twist vel_cmd;

            Eigen::Affine3d laser_base_slave  = tf2::transformToEigen(_tf_laser_base_slave);
            Eigen::Affine3d laser_base_master = tf2::transformToEigen(_tf_laser_base_master);

            Eigen::Affine3d curr =  laser_base_slave.inverse() * _icp_current * laser_base_master;
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(),1000,"laser_base_slave: \n" << laser_base_slave.matrix() << "\n");
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(),1000,"laser_base_slave.inverse(): \n" << laser_base_slave.inverse().matrix() << "\n");
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(),1000,"_icp_current: \n" << _icp_current.matrix() << "\n");
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(),1000,"laser_base_master: \n" << laser_base_master.matrix() << "\n");
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(),1000,"curr: \n" << curr.matrix() << "\n");
            Eigen::Affine3d err  = curr * _icp_goal.inverse();
            x_err = err(0,3);
            y_err = err(1,3);

            Eigen::Vector3d ea = err.rotation().eulerAngles(0, 1, 2);
            w_err = ea(2);
            w_err = ( w_err >  3) ? w_err - M_PI : w_err;
            w_err = ( w_err < -3) ? w_err + M_PI : w_err;
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(),1000,"err: " << x_err << " " << y_err << " " << std::setprecision (15) << w_err << " " << "\n");

        if(_icp_controlling)
        {
            auto dt = rclcpp::Duration(50ms);
            x_cmd = _x_pid_icp->computeCommand(x_err,dt);
            y_cmd = _y_pid_icp->computeCommand(y_err,dt);
            w_cmd = _w_pid_icp->computeCommand(w_err,dt);
            
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(),1000,"cmd: " << x_cmd << " " << y_cmd << " " << w_cmd << " " << "\n");
            vel_cmd.linear.x  = x_cmd;
            vel_cmd.linear.y  = y_cmd;
            vel_cmd.angular.z = w_cmd;
            _cmd_vel->publish(vel_cmd);
        }
        
        _rt.sleep();
    }
}

void icp_nav_follow_class::current_tf_thread()
{
    while(rclcpp::ok())
    {
        try {
            std::scoped_lock<std::mutex> guard_tf(_tf_mutex);
            _tf_master_slave = _tf_buffer->lookupTransform(
            _frame_name_base_master, _frame_name_base_slave,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            _frame_name_base_master.c_str(), _frame_name_base_slave.c_str(), ex.what());
        }
        // std::this_thread::sleep_for(50ms);
        _rate_tf->sleep();
    }
}

void icp_nav_follow_class::get_tf_goal()
{
    //TODO spin_once ??
    tf2::Stamped<tf2::Transform> stamped_transform;
    while(!_tf_buffer->canTransform(_frame_name_base_master, _frame_name_base_slave, tf2::TimePointZero, 2000ms) && rclcpp::ok()) {
      RCLCPP_INFO(
                    get_logger(), "waiting 2000 ms for %s->%s transform to become available",
                    _frame_name_base_master.c_str(), _frame_name_base_slave.c_str());
      std::this_thread::sleep_for(200ms);
    }
    _tf_goal_msg = _tf_buffer->lookupTransform(
            _frame_name_base_master, _frame_name_base_slave,
            tf2::TimePointZero);

    tf2::fromMsg(_tf_goal_msg, _tf_goal_transform);

    RCLCPP_INFO(get_logger(), "Initial transformation stored");
}

void icp_nav_follow_class::get_tf_laser_base_master()
{
    //TODO spin_once ??
    tf2::Stamped<tf2::Transform> stamped_transform;
    while(!_tf_buffer->canTransform(_frame_name_laser_master, _frame_name_base_master, tf2::TimePointZero, 2000ms) && rclcpp::ok()) {
      RCLCPP_INFO(
                    get_logger(), "waiting 2000 ms for %s->%s transform to become available",
                    _frame_name_laser_master.c_str(), _frame_name_base_master.c_str());
      std::this_thread::sleep_for(200ms);
    }
    _tf_laser_base_master = _tf_buffer->lookupTransform(
            _frame_name_laser_master, _frame_name_base_master,
            tf2::TimePointZero);

    RCLCPP_INFO(get_logger(), "Initial transformation stored");
}

void icp_nav_follow_class::get_tf_laser_base_slave()
{
    //TODO spin_once ??
    tf2::Stamped<tf2::Transform> stamped_transform;
    while(!_tf_buffer->canTransform(_frame_name_laser_slave, _frame_name_base_slave, tf2::TimePointZero, 2000ms) && rclcpp::ok()) {
      RCLCPP_INFO(
                    get_logger(), "waiting 2000 ms for %s->%s transform to become available",
                    _frame_name_laser_slave.c_str(), _frame_name_base_slave.c_str());
      std::this_thread::sleep_for(200ms);
    }
    _tf_laser_base_slave = _tf_buffer->lookupTransform(
            _frame_name_laser_slave,_frame_name_base_slave ,
            tf2::TimePointZero);


    RCLCPP_INFO(get_logger(), "Initial transformation stored");
}

void icp_nav_follow_class::tf_follow_thread()
{

    tf2::Stamped<tf2::Transform> stamped_transform_now;
    geometry_msgs::msg::Twist vel_cmd;
    double x_err,y_err,w_err;
    double x_cmd,y_cmd,w_cmd;

    while(rclcpp::ok())
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

void icp_nav_follow_class::init_control()
{
    _x_pid = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"x_pid");
    _y_pid = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"y_pid");
    _w_pid = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"w_pid");

    _x_pid->initPid();
    _y_pid->initPid();
    _w_pid->initPid();

    _x_pid_icp = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"x_pid_icp");
    _y_pid_icp = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"y_pid_icp");
    _w_pid_icp = std::make_unique<control_toolbox::PidROS>(this->shared_from_this(),"w_pid_icp");

    _x_pid_icp->initPid();
    _y_pid_icp->initPid();
    _w_pid_icp->initPid();

    this->get_tf_goal();
    this->get_tf_laser_base_master();
    this->get_tf_laser_base_slave();
    // _th_follow = std::thread(&icp_nav_follow_class::tf_follow_thread, this);

    _th_pcl    = std::thread(&icp_nav_follow_class::current_pcl_thread, this);
}

void icp_nav_follow_class::save_icp_goal(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    std::scoped_lock<std::mutex> guard_icp(_icp_mutex);
    Eigen::Affine3d laser_base_slave  = tf2::transformToEigen(_tf_laser_base_slave);
    Eigen::Affine3d laser_base_master = tf2::transformToEigen(_tf_laser_base_master);
    Eigen::Affine3d goal_tmp = laser_base_slave.inverse() * _icp_current * laser_base_master;
    // goal_tmp.col(2).setZero();
    // goal_tmp.row(2).setZero();
    // goal_tmp(2,2)   = 1.0;
    _icp_goal =  goal_tmp;
    response->success = true;
    RCLCPP_WARN_STREAM(this->get_logger(),"icp control goal saved : \n" << _icp_goal.matrix());
}

void icp_nav_follow_class::start_icp_goal(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    _x_pid_icp->reset();
    _y_pid_icp->reset();
    _w_pid_icp->reset();
    _icp_controlling = true;
    response->success = true;
    RCLCPP_WARN(this->get_logger(),"Starting icp control");
}

void icp_nav_follow_class::stop_icp_goal(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    _x_pid_icp->reset();
    _y_pid_icp->reset();
    _w_pid_icp->reset();
    _icp_controlling = false;
    response->success = true;
    RCLCPP_WARN(this->get_logger(),"Stopping icp control");
}

icp_nav_follow_class::icp_nav_follow_class():
Node("icp_nav_follow")
{

    this->declare_parameter("laser_scan_master"                       , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("laser_scan_slave"                        , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("frame_name_laser_slave"                  , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("frame_name_laser_master"                 , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("frame_name_base_slave"                   , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("frame_name_base_master"                  , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("cmd_vel_topic"                           , rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("icp_iterations"                          , rclcpp::ParameterValue(0));
    this->declare_parameter("icp_TransformationEpsilon"               , rclcpp::ParameterValue(0.0));
    this->declare_parameter("icp_EuclideanFitnessEpsilon"             , rclcpp::ParameterValue(0.0));
    this->declare_parameter("icp_RANSACOutlierRejectionThreshold"     , rclcpp::ParameterValue(0.0));
    this->declare_parameter("icp_MaxCorrespondenceDistance"           , rclcpp::ParameterValue(0.0));

    _laser_scan_master       = this->get_parameter("laser_scan_master").get_parameter_value().get<std::string>();
    _laser_scan_slave        = this->get_parameter("laser_scan_slave").get_parameter_value().get<std::string>();
    _frame_name_laser_slave  = this->get_parameter("frame_name_laser_slave").get_parameter_value().get<std::string>();
    _frame_name_laser_master = this->get_parameter("frame_name_laser_master").get_parameter_value().get<std::string>();
    _frame_name_base_slave   = this->get_parameter("frame_name_base_slave").get_parameter_value().get<std::string>();
    _frame_name_base_master  = this->get_parameter("frame_name_base_master").get_parameter_value().get<std::string>();
    _cmd_vel_topic           = this->get_parameter("cmd_vel_topic").get_parameter_value().get<std::string>();
    

    _icp_iterations                      = this->get_parameter("icp_iterations"                     ).get_parameter_value().get<int>();
    _icp_TransformationEpsilon           = this->get_parameter("icp_TransformationEpsilon"          ).get_parameter_value().get<double>();
    _icp_EuclideanFitnessEpsilon         = this->get_parameter("icp_EuclideanFitnessEpsilon"        ).get_parameter_value().get<double>();
    _icp_RANSACOutlierRejectionThreshold = this->get_parameter("icp_RANSACOutlierRejectionThreshold").get_parameter_value().get<double>();
    _icp_MaxCorrespondenceDistance       = this->get_parameter("icp_MaxCorrespondenceDistance"      ).get_parameter_value().get<double>();

    dyn_params_handler_ = this->add_on_set_parameters_callback(std::bind(&icp_nav_follow_class::dynamicParametersCallback, this, std::placeholders::_1));

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
    _pcl_reg_pub    = this->create_publisher<sensor_msgs::msg::PointCloud2>("reg_pcl", 1);
    _pcl_slave_pub  = this->create_publisher<sensor_msgs::msg::PointCloud2>("slave_pcl" , 1);

    _rate_tf     = std::make_unique<rclcpp::Rate>(20);
    _rate_follow = std::make_unique<rclcpp::Rate>(20);

    _th_tf     = std::thread(&icp_nav_follow_class::current_tf_thread, this);

    _save_icp_goal_srv  = this->create_service<std_srvs::srv::Trigger>("save_icp_goal" , std::bind(&icp_nav_follow_class::save_icp_goal , this, std::placeholders::_1,std::placeholders::_2));
    _start_icp_goal_srv = this->create_service<std_srvs::srv::Trigger>("start_icp_goal", std::bind(&icp_nav_follow_class::start_icp_goal, this, std::placeholders::_1,std::placeholders::_2));
    _stop_icp_goal_srv  = this->create_service<std_srvs::srv::Trigger>("stop_icp_goal" , std::bind(&icp_nav_follow_class::stop_icp_goal , this, std::placeholders::_1,std::placeholders::_2));

}

rcl_interfaces::msg::SetParametersResult
icp_nav_follow_class::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    for (const auto parameter : parameters) {

        const auto & type = parameter.get_type();
        const auto & name = parameter.get_name();

        if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
        if (name == "icp_TransformationEpsilon") {
            RCLCPP_INFO_STREAM(this->get_logger(),"UPDATING icp_TransformationEpsilon" << parameter.as_double());
            _icp_TransformationEpsilon = parameter.as_double();
        } else if (name == "icp_EuclideanFitnessEpsilon") {
            RCLCPP_INFO_STREAM(this->get_logger(),"UPDATING icp_EuclideanFitnessEpsilon" << parameter.as_double());
            _icp_EuclideanFitnessEpsilon = parameter.as_double();
        } else if (name == "icp_RANSACOutlierRejectionThreshold") {
            RCLCPP_INFO_STREAM(this->get_logger(),"UPDATING icp_RANSACOutlierRejectionThreshold" << parameter.as_double());
            _icp_RANSACOutlierRejectionThreshold = parameter.as_double();
        } else if (name == "icp_MaxCorrespondenceDistance") {
            RCLCPP_INFO_STREAM(this->get_logger(),"UPDATING icp_MaxCorrespondenceDistance" << parameter.as_double());
            _icp_MaxCorrespondenceDistance = parameter.as_double();
        }
        }
        if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
        if (name == "icp_iterations") {
            RCLCPP_INFO_STREAM(this->get_logger(),"UPDATING icp_iterations" << parameter.as_int());
            _icp_iterations = parameter.as_int();
        } 
        }
    }
    result.successful = true;
    return result;

}

int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    // rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto nodeState    = std::make_shared<icp_nav_follow_class>();
    nodeState->init_control();
    executor.add_node(nodeState);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
