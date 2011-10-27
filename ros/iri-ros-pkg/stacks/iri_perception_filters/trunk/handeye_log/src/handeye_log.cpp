#include "handeye_log.h"

// In the case that XYZRGB Point cloud is received
void 
load_xyzrgb_matlab_file( pcl::PointCloud<pcl::PointXYZRGB> &pcl_xyzrgb, std::ofstream &matlab_file){

  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = pcl_xyzrgb.begin();
  for (int rr = 0; rr < (int)pcl_xyzrgb.height; ++rr) {
    for (int cc = 0; cc < (int)pcl_xyzrgb.width; ++cc, ++pt_iter) {
      pcl::PointXYZRGB &pt = *pt_iter;
      RGBValue color;
      color.float_value= pt.rgb;
      matlab_file << rr << " " << cc << " " << pt.x*1000 << " " << pt.y*1000 << " " << pt.z*1000 << " " << (int)color.Red << " " << (int)color.Green << " " << (int)color.Blue << std::endl;
    }
  }
}

// In the case that XYZI Point cloud is received
void
load_xyzi_matlab_file( pcl::PointCloud<pcl::PointXYZI> &pcl_xyzi, std::ofstream &matlab_file){

  pcl::PointCloud<pcl::PointXYZI>::iterator pt_iter = pcl_xyzi.begin();
  for (int rr = 0; rr < (int)pcl_xyzi.height; ++rr) {
    for (int cc = 0; cc < (int)pcl_xyzi.width; ++cc, ++pt_iter) {
      pcl::PointXYZI &pt = *pt_iter;
      matlab_file << rr << " " << cc << " " << pt.x*1000 << " " << pt.y*1000 << " " << pt.z*1000 << " " << pt.intensity << std::endl;
    }
  }
}

// In the case that XYZ Point cloud is received
void
load_xyz_matlab_file( pcl::PointCloud<pcl::PointXYZ> &pcl_xyz, std::ofstream &matlab_file){

  pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = pcl_xyz.begin();
  for (int rr = 0; rr < (int)pcl_xyz.height; ++rr) {
    for (int cc = 0; cc < (int)pcl_xyz.width; ++cc, ++pt_iter) {
      pcl::PointXYZ &pt = *pt_iter;
      matlab_file << rr << " " << cc << " " << pt.x*1000 << " " << pt.y*1000 << " " << pt.z*1000 << std::endl;
    }
  }
}

// In the case that XYZRGB Point cloud is received
void
load_xyzrgb_img_depth( pcl::PointCloud<pcl::PointXYZRGB> &pcl_xyzrgb, std::ofstream &img_depth){
  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = pcl_xyzrgb.begin();
  for (int rr = 0; rr < (int)pcl_xyzrgb.height; ++rr) {
    for (int cc = 0; cc < (int)pcl_xyzrgb.width; ++cc, ++pt_iter) {
      pcl::PointXYZRGB &pt = *pt_iter;
      img_depth << rr << " " << cc << " " <<  pt.z*1000 << std::endl;
    }
  }
}

// In the case that XYZI Point cloud is received
void
load_xyzi_img_depth( pcl::PointCloud<pcl::PointXYZI> &pcl_xyzi, std::ofstream &img_depth){
  pcl::PointCloud<pcl::PointXYZI>::iterator pt_iter = pcl_xyzi.begin();
  for (int rr = 0; rr < (int)pcl_xyzi.height; ++rr) {
    for (int cc = 0; cc < (int)pcl_xyzi.width; ++cc, ++pt_iter) {
      pcl::PointXYZI &pt = *pt_iter;
      img_depth << rr << " " << cc << " " <<  pt.z*1000 << std::endl;
    }
  }
}

// In the case that XYZ Point cloud is received
void
load_xyz_img_depth( pcl::PointCloud<pcl::PointXYZ> &pcl_xyz, std::ofstream &img_depth){
  pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = pcl_xyz.begin();
  for (int rr = 0; rr < (int)pcl_xyz.height; ++rr) {
    for (int cc = 0; cc < (int)pcl_xyz.width; ++cc, ++pt_iter) {
      pcl::PointXYZ &pt = *pt_iter;
      img_depth << rr << " " << cc << " " <<  pt.z*1000 << std::endl;
    }
  }
}

HandeyeLog::HandeyeLog(): idx(0), nc(0), nr(0), pcl_type_(0), num_captures_(1) {

  this->event_server = CEventServer::instance();
  this->thread_server = CThreadServer::instance();

  this->mpcl2_thread_id_ = "store_matlab_pcl2_thread";
  this->iimg_thread_id_ = "store_intensity_image_thread";
  this->ipng_thread_id_ = "store_intensity_png_thread";
  this->dimg_thread_id_ = "store_depth_image_thread";
  this->tf_robot_thread_id_ = "store_tf_robot_thread";
  this->tf_pattern_thread_id_ = "store_tf_pattern_thread";
  this->tf_robot_inverse_thread_id_ = "store_tf_robot_inverse_thread";

  this->thread_server->create_thread(this->mpcl2_thread_id_);
  this->thread_server->create_thread(this->iimg_thread_id_);
  this->thread_server->create_thread(this->ipng_thread_id_);
  this->thread_server->create_thread(this->dimg_thread_id_);
  this->thread_server->create_thread(this->tf_robot_thread_id_);
  this->thread_server->create_thread(this->tf_pattern_thread_id_);
  this->thread_server->create_thread(this->tf_robot_inverse_thread_id_);

  this->thread_server->attach_thread(this->mpcl2_thread_id_, this->store_mpcl2_thread, this);
  this->thread_server->attach_thread(this->iimg_thread_id_, this->store_iimg_thread, this);
  this->thread_server->attach_thread(this->ipng_thread_id_, this->store_ipng_thread, this);
  this->thread_server->attach_thread(this->dimg_thread_id_, this->store_dimg_thread, this);
  this->thread_server->attach_thread(this->tf_robot_thread_id_, this->store_tf_robot_thread, this);
  this->thread_server->attach_thread(this->tf_pattern_thread_id_, this->store_tf_pattern_thread, this);
  this->thread_server->attach_thread(this->tf_robot_inverse_thread_id_, this->store_tf_robot_inverse_thread, this);

  this->mpcl2_event_on_id_ = "event_mpcl2_ON";
  this->mpcl2_event_off_id_ = "event_mpcl2_OFF";
  this->mpcl2_event_stop_id_ = "event_mpcl2_STOP";
  this->iimg_event_on_id_ = "event_iimg_ON";
  this->iimg_event_off_id_ = "event_iimg_OFF";
  this->iimg_event_stop_id_ = "event_iimg_STOP";
  this->ipng_event_on_id_ = "event_ipng_ON";
  this->ipng_event_off_id_ = "event_ipng_OFF";
  this->ipng_event_stop_id_ = "event_ipng_STOP";
  this->dimg_event_on_id_ = "event_dimg_ON";
  this->dimg_event_off_id_ = "event_dimg_OFF";
  this->dimg_event_stop_id_ = "event_dimg_STOP";
  this->tf_robot_event_on_id_ = "event_tf_robot_ON";
  this->tf_robot_event_off_id_ = "event_tf_robot_OFF";
  this->tf_robot_event_stop_id_ = "event_tf_robot_STOP";
  this->tf_pattern_event_on_id_ = "event_tf_pattern_ON";
  this->tf_pattern_event_off_id_ = "event_tf_pattern_OFF";
  this->tf_pattern_event_stop_id_ = "event_tf_pattern_STOP";
  this->tf_robot_inverse_event_on_id_ = "event_tf_robot_inverse_ON";
  this->tf_robot_inverse_event_off_id_ = "event_tf_robot_inverse_OFF";
  this->tf_robot_inverse_event_stop_id_ = "event_tf_robot_inverse_STOP";

  this->event_server->create_event(this->mpcl2_event_on_id_);
  this->event_server->create_event(this->mpcl2_event_off_id_);
  this->event_server->create_event(this->mpcl2_event_stop_id_);
  this->event_server->create_event(this->iimg_event_on_id_);
  this->event_server->create_event(this->iimg_event_off_id_);
  this->event_server->create_event(this->iimg_event_stop_id_);
  this->event_server->create_event(this->ipng_event_on_id_);
  this->event_server->create_event(this->ipng_event_off_id_);
  this->event_server->create_event(this->ipng_event_stop_id_);
  this->event_server->create_event(this->dimg_event_on_id_);
  this->event_server->create_event(this->dimg_event_off_id_);
  this->event_server->create_event(this->dimg_event_stop_id_);
  this->event_server->create_event(this->tf_robot_event_on_id_);
  this->event_server->create_event(this->tf_robot_event_off_id_);
  this->event_server->create_event(this->tf_robot_event_stop_id_);
  this->event_server->create_event(this->tf_pattern_event_on_id_);
  this->event_server->create_event(this->tf_pattern_event_off_id_);
  this->event_server->create_event(this->tf_pattern_event_stop_id_);
  this->event_server->create_event(this->tf_robot_inverse_event_on_id_);
  this->event_server->create_event(this->tf_robot_inverse_event_off_id_);
  this->event_server->create_event(this->tf_robot_inverse_event_stop_id_);

  this->thread_server->start_thread(this->mpcl2_thread_id_);
  this->thread_server->start_thread(this->iimg_thread_id_);
  this->thread_server->start_thread(this->ipng_thread_id_);
  this->thread_server->start_thread(this->dimg_thread_id_);
  this->thread_server->start_thread(this->tf_robot_thread_id_);
  this->thread_server->start_thread(this->tf_pattern_thread_id_);
  this->thread_server->start_thread(this->tf_robot_inverse_thread_id_);

    if (!this->nh.getParam("/handeye_log/flags/tf_robot", this->flag_tf_robot)){
        this->flag_tf_robot = false;
        this->nh.setParam("/handeye_log/flags/tf_robot", this->flag_tf_robot);
    }

    if (!this->nh.getParam("/handeye_log/flags/tf_pattern", this->flag_tf_pattern)){
        this->flag_tf_pattern = false;
        this->nh.setParam("/handeye_log/flags/tf_pattern", this->flag_tf_pattern);
    }

    if (!this->nh.getParam("/handeye_log/flags/matlab_pcl2", this->flag_mpcl2)){
        this->flag_mpcl2 = true;
        this->nh.setParam("/handeye_log/flags/matlab_pcl2", this->flag_mpcl2);
    }

    if (!this->nh.getParam("/handeye_log/flags/img_intensity", this->flag_iimg)){
        this->flag_iimg = true;
        this->nh.setParam("/handeye_log/flags/img_intensity", this->flag_iimg);
    }

    if (!this->nh.getParam("/handeye_log/flags/png_intensity", this->flag_ipng)){
        this->flag_ipng = true;
        this->nh.setParam("/handeye_log/flags/png_intensity", this->flag_ipng);
    }

    if (!this->nh.getParam("/handeye_log/flags/img_depth", this->flag_dimg)){
        this->flag_dimg = false;
        this->nh.setParam("/handeye_log/flags/img_depth", this->flag_dimg);
    }

    if (!this->nh.getParam("/handeye_log/flags/tf_robot_inverse", this->flag_tf_robot_inverse)){
        this->flag_tf_robot_inverse = false;
        this->nh.setParam("/handeye_log/flags/tf_robot_inverse", this->flag_tf_robot_inverse);
    }

    if (!this->nh.getParam("/handeye_log/num_captures", this->num_captures_)){
        this->num_captures_ = 1;
        this->nh.setParam("/handeye_log/num_captures", this->num_captures_);
    }

    this->cloudBuffer_.set_capacity(this->num_captures_);
    this->iimageBuffer_.set_capacity(this->num_captures_);

    //init class attributes if necessary
    this->tf_robot_ = this->tf_robot_.getIdentity();
    this->tf_pattern_ = this->tf_pattern_.getIdentity();

    //string for port names
    std::string pcl2_sub_name, iimg_sub_name, tf_robot_sub_name, tf_pattern_sub_name;
    pcl2_sub_name       = "input/pcl2/raw";
    iimg_sub_name       = "input/image/intensity";
    tf_robot_sub_name   = "input/pose/robot";
    tf_pattern_sub_name = "input/pose/pattern";

    // [init subscribers]
    this->pcl2_sub = this->nh.subscribe(pcl2_sub_name, 1, &HandeyeLog::pcl2_sub_callback, this);
    this->iimg_sub = this->nh.subscribe(iimg_sub_name, 1, &HandeyeLog::iimg_sub_callback, this);
    this->tf_robot_sub = this->nh.subscribe(tf_robot_sub_name, 1, &HandeyeLog::tf_robot_sub_callback, this);
    this->tf_pattern_sub = this->nh.subscribe(tf_pattern_sub_name, 1, &HandeyeLog::tf_pattern_sub_callback, this);

    // [init publishers]

    // [init services]
    this->srv = this->nh.advertiseService("store_image", &HandeyeLog::srv_callback, this);

    // [init clients]

    // [init action servers]

    // [init action clients]
    ROS_INFO("Starting storeRangeImage Node");
    ROS_INFO("Use [ rosservice call /store_image ] to store current range image");
}

HandeyeLog::~HandeyeLog()
{
  // Assures to stop all threads correctly
  this->event_server->set_event(this->mpcl2_event_stop_id_);
  this->event_server->set_event(this->iimg_event_stop_id_);
  this->event_server->set_event(this->ipng_event_stop_id_);
  this->event_server->set_event(this->dimg_event_stop_id_);
  this->event_server->set_event(this->tf_robot_event_stop_id_);
  this->event_server->set_event(this->tf_pattern_event_stop_id_);
  this->event_server->set_event(this->tf_robot_inverse_event_stop_id_);

  this->thread_server->end_thread(this->mpcl2_thread_id_);
  this->thread_server->end_thread(this->iimg_thread_id_);
  this->thread_server->end_thread(this->ipng_thread_id_);
  this->thread_server->end_thread(this->dimg_thread_id_);
  this->thread_server->end_thread(this->tf_robot_thread_id_);
  this->thread_server->end_thread(this->tf_pattern_thread_id_);
  this->thread_server->end_thread(this->tf_robot_inverse_thread_id_);

  this->event_server->reset_event(this->mpcl2_event_on_id_);
  this->event_server->reset_event(this->iimg_event_on_id_);
  this->event_server->reset_event(this->ipng_event_on_id_);
  this->event_server->reset_event(this->dimg_event_on_id_);
  this->event_server->reset_event(this->tf_robot_event_on_id_);
  this->event_server->reset_event(this->tf_pattern_event_on_id_);
  this->event_server->reset_event(this->tf_robot_inverse_event_on_id_);

  this->thread_server->delete_thread(this->mpcl2_thread_id_);
  this->thread_server->delete_thread(this->iimg_thread_id_);
  this->thread_server->delete_thread(this->ipng_thread_id_);
  this->thread_server->delete_thread(this->dimg_thread_id_);
  this->thread_server->delete_thread(this->tf_robot_thread_id_);
  this->thread_server->delete_thread(this->tf_pattern_thread_id_);
  this->thread_server->delete_thread(this->tf_robot_inverse_thread_id_);

  this->event_server->delete_event(this->mpcl2_event_on_id_);
  this->event_server->delete_event(this->iimg_event_on_id_);
  this->event_server->delete_event(this->ipng_event_on_id_);
  this->event_server->delete_event(this->dimg_event_on_id_);
  this->event_server->delete_event(this->tf_robot_event_on_id_);
  this->event_server->delete_event(this->tf_pattern_event_on_id_);
  this->event_server->delete_event(this->tf_robot_inverse_event_on_id_);

  this->mpcl2_thread_id_ = "";
  this->iimg_thread_id_ = "";
  this->ipng_thread_id_ = "";
  this->dimg_thread_id_ = "";
  this->tf_robot_thread_id_ = "";
  this->tf_pattern_thread_id_ = "";
  this->tf_robot_inverse_thread_id_ = "";

  this->mpcl2_event_on_id_ = "";
  this->iimg_event_on_id_ = "";
  this->ipng_event_on_id_ = "";
  this->dimg_event_on_id_ = "";
  this->tf_robot_event_on_id_ = "";
  this->tf_pattern_event_on_id_ = "";
  this->tf_robot_inverse_event_on_id_ = "";
}

/*  [subscriber callbacks] */
void 
HandeyeLog::pcl2_sub_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{ 
  this->nh.getParam("/handeye_log/num_captures", this->num_captures_);

  // ROS_INFO("HandeyeLog::pcl2_sub_callback: New Message Received"); 
  if (pcl::getFieldIndex(*msg, "rgb")!=-1){
    pcl::fromROSMsg(*msg, this->pcl_xyzrgb_);
    this->pcl_type_ = 1;
  }else if (pcl::getFieldIndex(*msg, "intensity")!=-1){
    pcl::fromROSMsg(*msg, this->pcl_xyzi_);
    this->pcl_type_ = 2;

    //  this->xyzi_cb_.resize(this->num_captures_);
    // posar aquesta variable en Mutex ;)
    // this->xyzi_cb_.push_back(this->pcl_xyzi_);
    this->cloudBuffer_.push_back(this->pcl_xyzi_);
  }else{
    pcl::fromROSMsg(*msg, this->pcl_xyz_);
    this->pcl_type_ = 3;
  }
}

void 
HandeyeLog::iimg_sub_callback(const sensor_msgs::Image::ConstPtr& msg)
{ 
  //TODO: It should work by just copying the ConstPtr to the original msg.
  // I am still wondering the reason why a copy has been coded :/
  this->intens_image_ = msg;

  //  ROS_INFO("HandeyeLog::iimg_sub_callback: New Message Received"); 
  try
    {
      if (sensor_msgs::image_encodings::isColor(msg->encoding))
	this->cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      else {
	// this->cv_ptr_ = cv_bridge::toCvCopy(msg, "8UC1");
	this->cv_ptr_ = cv_bridge::toCvCopy(msg, "mono8");
      }
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  this->iimageBuffer_.push_back(this->cv_ptr_);
}

void 
HandeyeLog::tf_robot_sub_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  btQuaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  btVector3 pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  this->tf_robot_ = btTransform(quat, pos);
}

void
HandeyeLog::tf_pattern_sub_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  btQuaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  btVector3 pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  this->tf_pattern_ = btTransform(quat, pos);
}

bool
HandeyeLog::srv_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::list<std::string> all_active_events;
  
  // ROS_INFO("HandeyeLog::srv_callback: New Service Request Received"); 

  this->nh.getParam("/handeye_log/flags/matlab_pcl2", this->flag_mpcl2);
  this->nh.getParam("/handeye_log/flags/img_depth", this->flag_dimg);
  this->nh.getParam("/handeye_log/flags/img_intensity", this->flag_iimg);
  this->nh.getParam("/handeye_log/flags/png_intensity", this->flag_ipng);
  this->nh.getParam("/handeye_log/flags/tf_robot", this->flag_tf_robot);
  this->nh.getParam("/handeye_log/flags/tf_pattern", this->flag_tf_pattern);
  this->nh.getParam("/handeye_log/flags/inverse", this->flag_tf_robot_inverse);
  
  if(this->flag_mpcl2){
    this->event_server->set_event(this->mpcl2_event_on_id_);
    all_active_events.push_back(this->mpcl2_event_off_id_);
  }
  
  if (this->flag_dimg){
    this->event_server->set_event(this->dimg_event_on_id_);
    all_active_events.push_back(this->dimg_event_off_id_);
  }

  if (this->flag_iimg){
    this->event_server->set_event(this->iimg_event_on_id_);
    all_active_events.push_back(this->iimg_event_off_id_);
  }

  if (this->flag_ipng){
    this->event_server->set_event(this->ipng_event_on_id_);
    all_active_events.push_back(this->ipng_event_off_id_);
  }

  if (this->flag_tf_robot){
    this->event_server->set_event(this->tf_robot_event_on_id_);
    all_active_events.push_back(this->tf_robot_event_off_id_);
  }

  if (this->flag_tf_pattern){
    this->event_server->set_event(this->tf_pattern_event_on_id_);
    all_active_events.push_back(this->tf_pattern_event_off_id_);
  }

  if(this->flag_tf_robot_inverse && !this->flag_tf_robot){
    this->event_server->set_event(this->tf_robot_inverse_event_on_id_);
    all_active_events.push_back(this->tf_robot_inverse_event_off_id_);
  }

  this->event_server->wait_all(all_active_events);
  this->thread_server->start_thread(this->mpcl2_thread_id_);
  this->thread_server->start_thread(this->iimg_thread_id_);
  this->thread_server->start_thread(this->ipng_thread_id_);
  this->thread_server->start_thread(this->dimg_thread_id_);
  this->thread_server->start_thread(this->tf_robot_thread_id_);
  this->thread_server->start_thread(this->tf_pattern_thread_id_);
  this->thread_server->start_thread(this->tf_robot_inverse_thread_id_);
        
  this->idx++;
  return true;
}

void *HandeyeLog::store_mpcl2_thread(void *param)
{
  HandeyeLog *handeyeLog = (HandeyeLog *)param;

  // ROS_INFO("Thread Store MPCL2 started");
  // ROS_INFO("Waiting for Event Store MPCL2");
  std::list<std::string> event_list_store_mpcl2;
  event_list_store_mpcl2.push_back(handeyeLog->mpcl2_event_on_id_);
  event_list_store_mpcl2.push_back(handeyeLog->mpcl2_event_stop_id_);
  int eid = handeyeLog->event_server->wait_first(event_list_store_mpcl2);
  if (eid == 1)
    pthread_exit(NULL);

  // ROS_INFO("Event Store MPCL2 activated");

  // [xyzRGB matlab file name]
  std::stringstream matlab_file_fn;
  matlab_file_fn << "frameXYZRGB_"<< std::setw(2) << std::setfill('0') << handeyeLog->idx << ".dat";

  // [open xyzRGB matlab file]
  handeyeLog->matlab_file_.open(matlab_file_fn.str().c_str(), std::ios::out);

  // [store xyzRGB matlab file]
  if (handeyeLog->pcl_type_ == 1)
    load_xyzrgb_matlab_file(handeyeLog->pcl_xyzrgb_, handeyeLog->matlab_file_);
  else if (handeyeLog->pcl_type_ == 2)
    load_xyzi_matlab_file(handeyeLog->pcl_xyzi_, handeyeLog->matlab_file_);
  else
    load_xyz_matlab_file(handeyeLog->pcl_xyz_, handeyeLog->matlab_file_);

  // [close range image file]
  handeyeLog->matlab_file_.close();
  ROS_INFO("Saved image %s", matlab_file_fn.str().c_str());

  handeyeLog->event_server->set_event(handeyeLog->mpcl2_event_off_id_);

  pthread_exit(NULL);
}

void *HandeyeLog::store_dimg_thread(void *param)
{
  HandeyeLog *handeyeLog = (HandeyeLog *)param;

  // ROS_INFO("Thread Store DIMG started");
  // ROS_INFO("Waiting for Event Store DIMG");
  std::list<std::string> event_list_store_dimg;
  event_list_store_dimg.push_back(handeyeLog->dimg_event_on_id_);
  event_list_store_dimg.push_back(handeyeLog->dimg_event_stop_id_);
  int eid = handeyeLog->event_server->wait_first(event_list_store_dimg);
  if (eid == 1)
    pthread_exit(NULL);

  // ROS_INFO("Event Store DIMG activated");

  // [range image file name]
  for (unsigned int aa = 0; aa < handeyeLog->num_captures_; aa++){
    std::stringstream img_depth_fn;
    // img_depth_fn << "frameTOFDepth_" << std::setw(2) << std::setfill('0') << handeyeLog->idx << "_" << std::setw(2) << std::setfill('0') << aa << ".txt";
    img_depth_fn << "frameTOFDepth" << handeyeLog->idx << "_" << aa << ".txt";

    // [open range image file]
    handeyeLog->img_depth_.open(img_depth_fn.str().c_str(), std::ios::out);

    // [store range image file]
    
    if (handeyeLog->pcl_type_ == 1)
      load_xyzrgb_img_depth(handeyeLog->pcl_xyzrgb_, handeyeLog->img_depth_);
    else if (handeyeLog->pcl_type_ == 2)
      // load_xyzi_img_depth(handeyeLog->pcl_xyzi_, handeyeLog->img_depth_);
      // load_xyzi_img_depth(handeyeLog->xyzi_cb_[aa], handeyeLog->img_depth_);
      load_xyzi_img_depth(handeyeLog->cloudBuffer_[aa], handeyeLog->img_depth_);
    else
      load_xyz_img_depth(handeyeLog->pcl_xyz_, handeyeLog->img_depth_);

    // [close range image file]
    handeyeLog->img_depth_.close();
    ROS_INFO("Saved image %s", img_depth_fn.str().c_str());
  }

  handeyeLog->event_server->set_event(handeyeLog->dimg_event_off_id_);
  
  pthread_exit(NULL);
}

void *HandeyeLog::store_iimg_thread(void *param)
{
  HandeyeLog *handeyeLog = (HandeyeLog *)param;

  // ROS_INFO("Thread Store IIMG started");
  // ROS_INFO("Waiting for Event Store IIMG");
  std::list<std::string> event_list_store_iimg;
  event_list_store_iimg.push_back(handeyeLog->iimg_event_on_id_);
  event_list_store_iimg.push_back(handeyeLog->iimg_event_stop_id_);
  int eid =  handeyeLog->event_server->wait_first(event_list_store_iimg);
  if (eid == 1)
    pthread_exit(NULL);

  // ROS_INFO("Event Store IIMG activated");
  
  // [amplitude image file name]
  for (unsigned int aa = 0; aa < handeyeLog->num_captures_; aa++){
    std::stringstream img_intensity_fn;
    img_intensity_fn << "frameTOFAmplitude" << handeyeLog->idx << "_" << aa << ".txt";
    
    // [open amplitude image file]
    handeyeLog->img_intensity_.open(img_intensity_fn.str().c_str(), std::ios::out);
    
    // [store amplitude image file]
    // cv::Mat_<uchar>::iterator pt_iter = handeyeLog->cv_ptr_->image.begin<uchar>();
    cv::Mat_<uchar>::iterator pt_iter = handeyeLog->iimageBuffer_[aa]->image.begin<uchar>();
    for (int rr = 0; rr < handeyeLog->cv_ptr_->image.rows; ++rr) {
      for (int cc = 0; cc < handeyeLog->cv_ptr_->image.cols; ++cc , ++pt_iter) {
	handeyeLog->img_intensity_ << rr << " " << cc << " " <<  (float)*pt_iter << std::endl; // handeyeLog->cv_ptr_->image.at<float>(rr,cc) << std::endl;
      }
    }

    // [close amplitude image file]
    handeyeLog->img_intensity_.close();
    ROS_INFO("Saved image %s", img_intensity_fn.str().c_str());
  }
  handeyeLog->event_server->set_event(handeyeLog->iimg_event_off_id_);

  pthread_exit(NULL);
}

void *HandeyeLog::store_ipng_thread(void *param)
{
  HandeyeLog *handeyeLog = (HandeyeLog *)param;

  // ROS_INFO("Thread Store IPNG started");
  // ROS_INFO("Waiting for Event Store IPNG");
  std::list<std::string> event_list_store_ipng;
  event_list_store_ipng.push_back(handeyeLog->ipng_event_on_id_);
  event_list_store_ipng.push_back(handeyeLog->ipng_event_stop_id_);
  int eid = handeyeLog->event_server->wait_first(event_list_store_ipng);
  if (eid == 1)
    pthread_exit(NULL);

  // ROS_INFO("Event Store IPNG activated");
  
  // [stores PNG file]
  sensor_msgs::CvBridge g_bridge;
  if (g_bridge.fromImage(*handeyeLog->intens_image_, "bgr8")) {
    IplImage *image = g_bridge.toIpl();
    if (image) {
      std::stringstream filename;
      filename << "frame_" << std::setw(2) << std::setfill('0') << handeyeLog->idx << ".png";
      cvSaveImage(filename.str().c_str(), image);
      ROS_INFO("Saved image %s", filename.str().c_str());
    } else {
      ROS_WARN("Couldn't save image, no data!");
    }
  } else {
    ROS_ERROR("Unable to convert %s image to bgr8", handeyeLog->intens_image_->encoding.c_str());
  }

  handeyeLog->event_server->set_event(handeyeLog->ipng_event_off_id_);

  pthread_exit(NULL);
}


void *HandeyeLog::store_tf_robot_thread(void *param)
{
  HandeyeLog *handeyeLog = (HandeyeLog *)param;

  // ROS_INFO("Thread Store TF_ROBOT started");
  // ROS_INFO("Waiting for Event Store TF_ROBOT");
  std::list<std::string> event_list_store_tf_robot;
  event_list_store_tf_robot.push_back(handeyeLog->tf_robot_event_on_id_);
  event_list_store_tf_robot.push_back(handeyeLog->tf_robot_event_stop_id_);
  int eid = handeyeLog->event_server->wait_first(event_list_store_tf_robot);
  if (eid == 1)
    pthread_exit(NULL);

  // ROS_INFO("Event Store TF_ROBOT activated");
  
    // [tf_robot file name]
    std::stringstream tf_robot_fn;
    tf_robot_fn << "frame_" << std::setw(2) << std::setfill('0') << handeyeLog->idx << ".coords";
  
    // [open tf_robot file]
    handeyeLog->tf_robot_file_.open(tf_robot_fn.str().c_str(), std::ios::out);

    // [store tf_robot file]
    handeyeLog->tf_robot_file_ << handeyeLog->tf_robot_.getBasis()[0][0] << " " << handeyeLog->tf_robot_.getBasis()[0][1] << " " << handeyeLog->tf_robot_.getBasis()[0][2] << " " << handeyeLog->tf_robot_.getOrigin()[0]*1000 << " "
	       << handeyeLog->tf_robot_.getBasis()[1][0] << " " << handeyeLog->tf_robot_.getBasis()[1][1] << " " << handeyeLog->tf_robot_.getBasis()[1][2] << " " << handeyeLog->tf_robot_.getOrigin()[1]*1000 << " "
	       << handeyeLog->tf_robot_.getBasis()[2][0] << " " << handeyeLog->tf_robot_.getBasis()[2][1] << " " << handeyeLog->tf_robot_.getBasis()[2][2] << " " << handeyeLog->tf_robot_.getOrigin()[2]*1000 << std::endl;

    // [close range image file]
    handeyeLog->tf_robot_file_.close();
    ROS_INFO("Saved pose %s", tf_robot_fn.str().c_str());

  handeyeLog->event_server->set_event(handeyeLog->tf_robot_event_off_id_);

  pthread_exit(NULL);
}

void *HandeyeLog::store_tf_pattern_thread(void *param)
{
  HandeyeLog *handeyeLog = (HandeyeLog *)param;

  // ROS_INFO("Thread Store TF_PATTERN started");
  // ROS_INFO("Waiting for Event Store TF_PATTERN");
  std::list<std::string> event_list_store_tf_pattern;
  event_list_store_tf_pattern.push_back(handeyeLog->tf_pattern_event_on_id_);
  event_list_store_tf_pattern.push_back(handeyeLog->tf_pattern_event_stop_id_);
  int eid = handeyeLog->event_server->wait_first(event_list_store_tf_pattern);
  if (eid == 1)
    pthread_exit(NULL);

  // ROS_INFO("Event Store TF_PATTERN activated");

    // [tf_pattern file name]
    std::stringstream tf_pattern_fn;
    tf_pattern_fn << "framePattern_" << std::setw(2) << std::setfill('0') << handeyeLog->idx << ".m";
  
    // [open tf_pattern file]
    handeyeLog->tf_pattern_file_.open(tf_pattern_fn.str().c_str(), std::ios::out);

    // [store tf_pattern file]
    handeyeLog->tf_pattern_file_ << handeyeLog->tf_pattern_.getBasis()[0][0] << " " << handeyeLog->tf_pattern_.getBasis()[0][1] << " " << handeyeLog->tf_pattern_.getBasis()[0][2] << " " << handeyeLog->tf_pattern_.getOrigin()[0]*1000 << " "
		 << handeyeLog->tf_pattern_.getBasis()[1][0] << " " << handeyeLog->tf_pattern_.getBasis()[1][1] << " " << handeyeLog->tf_pattern_.getBasis()[1][2] << " " << handeyeLog->tf_pattern_.getOrigin()[1]*1000 << " "
		 << handeyeLog->tf_pattern_.getBasis()[2][0] << " " << handeyeLog->tf_pattern_.getBasis()[2][1] << " " << handeyeLog->tf_pattern_.getBasis()[2][2] << " " << handeyeLog->tf_pattern_.getOrigin()[2]*1000 << std::endl;

    // [close range image file]
    handeyeLog->tf_pattern_file_.close();
ROS_INFO("Saved pose %s", tf_pattern_fn.str().c_str());

  handeyeLog->event_server->set_event(handeyeLog->tf_pattern_event_off_id_);

  pthread_exit(NULL);
}

void *HandeyeLog::store_tf_robot_inverse_thread(void *param)
{
  HandeyeLog *handeyeLog = (HandeyeLog *)param;

  // ROS_INFO("Thread Store TF_ROBOT_INVERSE started");
  // ROS_INFO("Waiting for Event Store TF_ROBOT_INVERSE");
  std::list<std::string> event_list_store_tf_robot_inverse;
  event_list_store_tf_robot_inverse.push_back(handeyeLog->tf_robot_inverse_event_on_id_);
  event_list_store_tf_robot_inverse.push_back(handeyeLog->tf_robot_inverse_event_stop_id_);
  int eid = handeyeLog->event_server->wait_first(event_list_store_tf_robot_inverse);
  if (eid == 1)
    pthread_exit(NULL);

  // ROS_INFO("Event Store TF_ROBOT_INVERSE activated");

  // [tf_robot file name]
  std::stringstream tf_robot_fn;
  tf_robot_fn << "frame_" << std::setw(2) << std::setfill('0') << handeyeLog->idx << ".coords";
  
  // [open tf_robot file]
  handeyeLog->tf_robot_file_.open(tf_robot_fn.str().c_str(), std::ios::out);

  // [store tf_robot file]
  handeyeLog->tf_robot_file_ << handeyeLog->tf_robot_.inverse().getBasis()[0][0] << " " << handeyeLog->tf_robot_.inverse().getBasis()[0][1] << " " << handeyeLog->tf_robot_.inverse().getBasis()[0][2] << " " << handeyeLog->tf_robot_.inverse().getOrigin()[0]*1000 << " "
		       << handeyeLog->tf_robot_.inverse().getBasis()[1][0] << " " << handeyeLog->tf_robot_.inverse().getBasis()[1][1] << " " << handeyeLog->tf_robot_.inverse().getBasis()[1][2] << " " << handeyeLog->tf_robot_.inverse().getOrigin()[1]*1000 << " "
		       << handeyeLog->tf_robot_.inverse().getBasis()[2][0] << " " << handeyeLog->tf_robot_.inverse().getBasis()[2][1] << " " << handeyeLog->tf_robot_.inverse().getBasis()[2][2] << " " << handeyeLog->tf_robot_.inverse().getOrigin()[2]*1000 << std::endl;

  // [close range image file]
  handeyeLog->tf_robot_file_.close();
  ROS_INFO("Saved (inverse) pose %s", tf_robot_fn.str().c_str());

  handeyeLog->event_server->set_event(handeyeLog->tf_robot_inverse_event_off_id_);

  pthread_exit(NULL);
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "store_rangeImage");
    HandeyeLog handeye_log;
    ros::spin();
    return 0;
}
