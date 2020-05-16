#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

// File managing
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

class LogData
{
private:
  std::ofstream& log_data_file_;  // reference to the input stream
  // public ros node handle
  ros::NodeHandle nh_;
  ros::Subscriber sub_robot_pos_;
  ros::Subscriber sub_power_load_;
  ros::Subscriber sub_odometry_;
  ros::Subscriber sub_imu_data_;
  ros::Subscriber sub_safety_distance_;
  ros::Subscriber sub_diagnostics_;
  ros::Subscriber sub_goal_result_;
  ros::Subscriber sub_goal_;


 
public:
  LogData(std::ofstream& log_file, 
          const ros::NodeHandle &node_handle);
  ~LogData();
  geometry_msgs::Point rob_pos_;
  geometry_msgs::Point goal_pos_;
  float power_load_;
  float dist_to_obstacle_;
  float linear_vel_x_;
  float linear_accel_x_;
  std::string qa_energy_;
  std::string qa_safety_;
  ros::Timer store_info_timer_;


  std::string data_log_filename_;
  
  int goal_counter_;
  int goal_failed_counter_;
  bool goal_rached_;


  
  void robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_rob_pose);
  void power_load_callback(const std_msgs::Float32::ConstPtr& msg_power_load);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg_odom);
  void imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg_imu);
  void safety_distance_callback(const std_msgs::Float32::ConstPtr& msg_safet_distance);
  void diagnostics_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg_diagnostics);
  void goal_result_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg_goal_result);
  void goal_callback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg_goal);

  // Stops simulation by killing a required node
  void stop_simulation();

  // simple euclidean distance between two points
  double point_distance (const geometry_msgs::Point point_1, const geometry_msgs::Point point_2);

  // gets current date
  std::string get_date(bool get_hour);

  // Opens log file
  bool open_log_file();
  bool write_log_header();
  void timerCallback(const ros::TimerEvent& event);
  void store_info();
  double goal_distance ();
  void updateQA(const diagnostic_msgs::DiagnosticStatus diagnostic_status);


};

LogData::LogData(std::ofstream& log_file, const ros::NodeHandle &node_handle) 
: log_data_file_(log_file),
nh_(node_handle),
goal_counter_(0),
goal_failed_counter_(0),
dist_to_obstacle_(0),
linear_accel_x_(0),
power_load_(0),
goal_rached_(false)
{

  int ret_val;

  std::string data_log_folder;
  double prediction_length;
  double store_log_freq;

  log_data_file_ = std::ofstream();

  nh_.param("data_log_folder", data_log_folder, ros::package::getPath("metacontrol_experiments") + std::string("/data/"));
  nh_.param("store_data_freq", store_log_freq, 1.0);

  sub_robot_pos_ = nh_.subscribe("/amcl_pose", 1, &LogData::robot_pose_callback, this);
  sub_power_load_ = nh_.subscribe("/power_load", 1, &LogData::power_load_callback, this);
  sub_odometry_ = nh_.subscribe("/odom", 1, &LogData::odom_callback, this);
  sub_imu_data_ = nh_.subscribe("/imu/data", 1, &LogData::imu_data_callback, this);
  sub_safety_distance_ = nh_.subscribe("/d_obstacle", 1, &LogData::safety_distance_callback, this);
  sub_diagnostics_ = nh_.subscribe("/diagnostics", 1, &LogData::diagnostics_callback, this);
  sub_goal_result_ = nh_.subscribe("/move_base/result", 1, &LogData::goal_result_callback, this);
  sub_goal_ = nh_.subscribe("/move_base/goal", 1, &LogData::goal_callback, this);

  data_log_filename_ = data_log_folder + "log_Metacontrol_sim_";
  data_log_filename_ = data_log_filename_ + get_date(false);
  data_log_filename_ = data_log_filename_ + ".csv";

  log_data_file_.exceptions (std::ifstream::failbit | std::ifstream::badbit);

  if( !write_log_header() )
  {
     ROS_ERROR("[STOP SIM] Error in log file");
  }
  else
  {
    ROS_INFO("[STOP SIM] Logging to %s", data_log_filename_.c_str());
    store_info_timer_ = nh_.createTimer(ros::Duration(1.0/store_log_freq), &LogData::timerCallback, this, false, false);
    ROS_INFO("[STOP SIM] Node Initialization Completed");

  }
}


LogData::~LogData()
{
   log_data_file_.flush();
}

void LogData::robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_rob_pose)
{
    rob_pos_ = msg_rob_pose->pose.pose.position;
    // ROS_INFO("[STOP SIM :: robot_pose_callback] - Pos x %f - y %f", rob_pos_.x, rob_pos_.y);

}
void LogData::power_load_callback(const std_msgs::Float32::ConstPtr& msg_power_load)
{
  power_load_ = msg_power_load->data;
}
 
void LogData::odom_callback(const nav_msgs::Odometry::ConstPtr& msg_odom)
{
  linear_vel_x_ = msg_odom->twist.twist.linear.x;
}
void LogData::imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg_imu)
{
  linear_accel_x_ = msg_imu->linear_acceleration.x;
}

void LogData::LogData::safety_distance_callback(const std_msgs::Float32::ConstPtr& msg_safet_distance)
{
  dist_to_obstacle_ = msg_safet_distance->data;
}

void LogData::diagnostics_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg_diagnostics)
{

  for (size_t i = msg_diagnostics->status.size() - 1; i != (size_t)-1; --i)
  {
    diagnostic_msgs::DiagnosticStatus tmp_diagnostic = msg_diagnostics->status[i];
    
    if (tmp_diagnostic.message.compare(0, 2, "QA") == 0 )
    {
     // ROS_INFO("[LogData :: diagnostics_callback] - Message: %s", tmp_diagnostic.message.c_str());
      updateQA(tmp_diagnostic);
    }
  }
  
  // qa_energy_ = msg_diagnostics->status
}
void LogData::updateQA(const diagnostic_msgs::DiagnosticStatus diagnostic_status)
{
  // ROS_INFO("[LogData :: updateQA] - Key: %s", diagnostic_status.values[0].key.c_str());
  if (diagnostic_status.values[0].key.compare(0, 2, "en") == 0)
  {
    qa_energy_ = diagnostic_status.values[0].value;
  }
  if (diagnostic_status.values[0].key.compare(0, 2, "sa") == 0)
  {
    qa_safety_ = diagnostic_status.values[0].value;
  }
}
  
void LogData::goal_result_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg_goal_result)
{
  if(msg_goal_result->status.status == 3)
  {
    goal_rached_ = true;
  }
  else
  {
    goal_failed_counter_ ++;
  }
  
}
void LogData::goal_callback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg_goal)
{
  goal_pos_ = msg_goal->goal.target_pose.pose.position;
  ROS_INFO("[LogData :: goal_callback] - Goal Pos x %f - y %f", goal_pos_.x, goal_pos_.y);
  goal_counter_ ++;
}


void LogData::stop_simulation()
{
  ROS_INFO("[STOP SIM :: stop_simulation] Killing nodes");
  std::system("rosnode kill /fake_localization");
  std::system("rosnode kill /record_bag_node");
  sleep(1);
}

double LogData::goal_distance ()
{
  return point_distance(rob_pos_, goal_pos_);
}

double LogData::point_distance (const geometry_msgs::Point point_1, const geometry_msgs::Point point_2)
{
    double distance;
    double distance_x;
    double distance_y;

    distance_x = point_1.x - point_2.x;
    distance_y = point_1.y - point_2.y;

    distance = std::sqrt(distance_x*distance_x + distance_y*distance_y);

    return distance;
}

std::string LogData::get_date(bool get_hour)
{
  std::time_t now;
  char the_date[12];

  the_date[0] = '\0';

  now = std::time(NULL);

  if (now != -1)
  {
    if(get_hour)
    {
      std::strftime(the_date, 12, "%H:%M:%S", std::localtime(&now));
    }
    else
    {
      std::strftime(the_date, 12, "%d_%m-%H_%M", std::localtime(&now));
    }
  }

  return std::string(the_date);
}

bool LogData::open_log_file()
{
    bool is_open;
    try
    {
        log_data_file_.open(data_log_filename_.c_str(), std::ios::app);
        is_open = true;
    }
    catch (std::ofstream::failure &writeErr)
    {
        ROS_ERROR("Exception occured when writing to a file - %s", writeErr.what());
        is_open = false;
    }
    return is_open;
}

bool LogData::write_log_header()
{
  if (open_log_file())
  {
    //ROS_INFO("[STOP SIM::write_log_header] Create Log data file Header");
    log_data_file_ << "Time Stamp, ";
    log_data_file_ << "Robot_pose_x, ";
    log_data_file_ << "Robot_pose_y, ";
    log_data_file_ << "D_to_goal, ";
    log_data_file_ << "bat_power_load, ";
    log_data_file_ << "d_to_obstacle, ";
    log_data_file_ << "linear_vel_x, ";
    log_data_file_ << "linear_accel_x, ";
    log_data_file_ << "QA_energy, ";
    log_data_file_ << "QA_safety, ";
    log_data_file_ << "n_reconfigurations, ";
    log_data_file_ << "failed_goals, ";
    log_data_file_ << "\n";
    log_data_file_.close();
    return true;
  }
  return false;
}


void LogData::store_info()
{
    if (open_log_file())
    {
      std::string tmp_string;
      char buffer[100];
      // Time Stamp
      tmp_string = get_date(true) + std::string(", ");
      log_data_file_ << tmp_string.c_str();

      
      // Robot_pose_x
      // Robot_pose_y

      tmp_string.clear();
      sprintf(buffer, "%4f, %.4f, ", rob_pos_.x, rob_pos_.y);
      tmp_string = buffer;
      log_data_file_ << tmp_string.c_str();


      // D_to_goal
      tmp_string.clear();
      sprintf(buffer, "%.3f, ", point_distance (goal_pos_, rob_pos_));
      tmp_string = buffer;
      log_data_file_ << tmp_string.c_str();

      // bat power load and distance
      tmp_string.clear();
      sprintf(buffer, "%.3f, %.3f, ", power_load_, dist_to_obstacle_);
      tmp_string = buffer;
      log_data_file_ << tmp_string.c_str();

      // vel and accel
      tmp_string.clear();
      sprintf(buffer, "%.3f, %.3f, ", linear_vel_x_, linear_accel_x_);
      tmp_string = buffer;
      log_data_file_ << tmp_string.c_str();

      // qa energy and safety
      tmp_string.clear();
      tmp_string = qa_energy_ + std::string(", ") + qa_safety_ + std::string(", ");
      log_data_file_ << tmp_string.c_str();
      
      // n reconfigurations failed goals
      tmp_string.clear();
      sprintf(buffer, "%d, %d ", goal_counter_, goal_failed_counter_);
      tmp_string = buffer;
      log_data_file_ << tmp_string.c_str();
      log_data_file_ << "\n";
      log_data_file_.close();
    }
    else
    {
      ROS_WARN("[LogData::store_info] log data file not open");
    }
    log_data_file_.flush();

}

void LogData::timerCallback(const ros::TimerEvent& event)
{
    store_info();

}


int main(int argc, char **argv){

  bool arrived_to_goal;
  arrived_to_goal = false;

  ros::init(argc, argv, "log_data_stop_sim_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ROS_INFO("[STOP SIM] - Start node");

  std::ofstream log_file;
  LogData log_data = LogData(log_file, nh);
  
  ROS_INFO("Initialized an async multi-thread node.");
  ros::AsyncSpinner async_spinner(4);  // Use 4 threads
      async_spinner.start();

  ros::Rate loop_rate(10);

  while (log_data.goal_pos_.x == 0)
  {
    // wait for goal
    ROS_INFO("[STOP SIM] - Wait for goal msg");
    loop_rate.sleep();
  }
  log_data.store_info_timer_.start();
  double first_distance = log_data.goal_distance ();
  ROS_INFO("[STOP SIM] - Initial goal distance: %.2f", first_distance);
  ROS_INFO("[STOP SIM] - Wait for robot to reach the goal");
  
  while (ros::ok())
  {
  
    // double distance;
    // distance = log_data.goal_distance ();
     

    // if (0.7 > ((first_distance - distance) / first_distance ) > 0.6)
    // {
    //   ROS_INFO("[STOP SIM] - 2 / 3 of the route completed");
    // }
    
    if(log_data.goal_rached_)
    {
        log_data.store_info_timer_.stop();
        log_data.stop_simulation();
        log_data.store_info();
        ros::shutdown();
    }
    loop_rate.sleep();
  }
  return  0;
}
