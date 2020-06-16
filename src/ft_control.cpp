#include <ft_control/ft_control.hpp>

void EliteFtControl::EliteFtControlInit(ros::NodeHandle &nh)
{
  nh_ = nh;

  elite_home = nh_.serviceClient<elite_msgs::Move>("/elite_robot/go_home");;
  elite_movej = nh_.serviceClient<elite_msgs::Move>("/elite_robot/arm_controller/elite_driver/move_joint");;
  elite_movel = nh_.serviceClient<elite_msgs::Move>("/elite_robot/arm_controller/elite_driver/move_line");;
  elite_stop = nh_.serviceClient<std_srvs::Empty>("/elite_robot/stop_move");
  ft_sensor_client = nh_.serviceClient<robotiq_ft_sensor::sensor_accessor>("/robotiq_ft_sensor_acc");
 
  elite_j_sub = nh_.subscribe("/elite_robot/arm_controller/joint_states", 10, &EliteFtControl::joint_state_CB, this);
  elite_p_sub = nh_.subscribe("/elite_robot/arm_controller/tool_point", 10,  &EliteFtControl::pose_state_CB, this);
  elite_state_sub = nh_.subscribe("/elite_robot/arm_controller/robot_states", 10,  &EliteFtControl::robot_state_CB, this);
  elite_ft_sub = nh_.subscribe("/robotiq_ft_sensor", 10,  &EliteFtControl::ft_sensor_CB, this);

  contact_search_server = nh_.advertiseService("ft_contact_search", &EliteFtControl::ftContactSearch, this);
  plannar_search_server = nh_.advertiseService("ft_plannar_search", &EliteFtControl::ftPlannarSearch, this);
}

void EliteFtControl::joint_state_CB(const sensor_msgs::JointState::ConstPtr &msg)
{ 
  // ROS_INFO("subcribing joint states");
  current_joint_positions = {(float) msg->position[0], (float) msg->position[1], (float) msg->position[2], 
                             (float) msg->position[3], (float) msg->position[4], (float) msg->position[5]};
};

void EliteFtControl::pose_state_CB(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  // ROS_INFO("subcribing pose states");
  current_pose = {(float) msg->twist.linear.x, (float) msg->twist.linear.y, (float) msg->twist.linear.z,
                  (float) msg->twist.angular.x, (float) msg->twist.angular.y, (float) msg->twist.angular.z};
};

void EliteFtControl::robot_state_CB(const elite_msgs::RobotMsg::ConstPtr &msg)
{
  current_robot_state = msg->state;
  // ROS_INFO("current robot state %d", current_robot_state);
};

void EliteFtControl::ft_sensor_CB(const robotiq_ft_sensor::ft_sensor::ConstPtr &msg)
{
  current_ft = {msg->Fx, msg->Fy, msg->Fz, msg->Mx, msg->My, msg->Mz};
};

std::vector<float> EliteFtControl::getCurrentPose()
{
  return current_pose;
};

std::vector<float> EliteFtControl::getCurrentJoints()
{
  return current_joint_positions;
};

int EliteFtControl::getCurrentState()
{
  return current_robot_state;
}

std::vector<float> EliteFtControl::getCurrentFT()
{
  return current_ft;
};

void EliteFtControl::eliteMovej(std::vector<float> joint_target, float vel)
{
  elite_msgs::Move movej_srv;
  movej_srv.request.pose = joint_target;
  movej_srv.request.mvvelo = vel;
  movej_srv.request.mvtime = 0;
  movej_srv.request.mvradii = 0;
  
  elite_movej.call(movej_srv);
};

void EliteFtControl::eliteMovel(std::vector<float> target_pose, float vel)
{
  elite_msgs::Move movel_srv;
  movel_srv.request.pose = target_pose;
  movel_srv.request.mvvelo = vel;
  movel_srv.request.mvtime = 0;
  movel_srv.request.mvradii = 0;
  
  elite_movel.call(movel_srv);
};

void EliteFtControl::eliteHome()
{
  elite_msgs::Move home_srv;
  home_srv.request.pose = {0};
  home_srv.request.mvvelo = 5;
  home_srv.request.mvtime = 0;
  home_srv.request.mvradii = 0;
  home_srv.request.mvacc = 0;
  
  elite_home.call(home_srv);
};

void EliteFtControl::eliteStop()
{
  std_srvs::Empty empty_srv;
  elite_stop.call(empty_srv);
};

void EliteFtControl::moveToStandby(void)
{
  eliteMovej(standby_joints, 5);
};

void EliteFtControl::resetFtSensor()
{
  robotiq_ft_sensor::sensor_accessor reset_srv;
  reset_srv.request.command = "";
  reset_srv.request.command_id = 8;
  ft_sensor_client.call(reset_srv);
};

bool EliteFtControl::waitRobotMoving(void)
{
  double elapsed;
  auto start_t = ros::Time::now();
  
  while (getCurrentState() != 3)
  {
    elapsed = (ros::Time::now() - start_t).toSec();
    ROS_INFO("wait robot start moving, elapsed: %lf s", elapsed);
    if (elapsed > 100) 
    {
      ROS_INFO("waiting timeout");
      return false;
    };
    ros::Duration(0.2).sleep();
  }

  while (getCurrentState() == 3)
  {
    elapsed = (ros::Time::now() - start_t).toSec();
    ROS_INFO("robot is moving, elapsed: %lf s", elapsed);
    if (getCurrentState() != 3)
    {
      ROS_INFO("robot stopped");
      return true;
    };
    if (elapsed > 1000)
    {
      ROS_INFO("movement timeout reached");
      return false;
    };
    ros::Duration(0.1).sleep();
  }
};

bool EliteFtControl::ftContactSearch(ft_control::ContactSearchRequest &req, ft_control::ContactSearchResponse &res)
{
  resetFtSensor();
  ros::Duration(0.2).sleep();
  std::vector<float> target_pose;
  target_pose = getCurrentPose();
  target_pose[2] -= req.depth;
  eliteMovel(target_pose, 5);
  while (abs(current_pose[2] - target_pose[2]) > 0.1)
  {
    if (current_ft[2] > req.force)
    {
      ROS_INFO("contact searching");
      ros::Duration(0.1).sleep();
    }
    else
    {
      eliteStop();
      res.success = true;
      return true;
    }
  }
  res.success = false;
  return true;
};

Eigen::Matrix4d createMatrixFromPose(float x, float y, float z, float rx, float ry, float rz)
{
  Eigen::Matrix4d T;
  Eigen::Quaterniond q;
  Eigen::AngleAxisd roll((float)rx, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch((float)ry, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw((float)rz, Eigen::Vector3d::UnitZ());
  q = yaw * pitch * roll;
  T << q.toRotationMatrix(), Eigen::Vector3d(x, y, z), 0, 0, 0, 1;
  return T;
}

Eigen::Matrix4d createMatrixFromPose(std::vector<float> pose)
{
  Eigen::Matrix4d T;
  Eigen::Quaterniond q;
  Eigen::AngleAxisd roll((float)pose[3], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch((float)pose[4], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw((float)pose[5], Eigen::Vector3d::UnitZ());
  q = yaw * pitch * roll;
  T << q.toRotationMatrix(), Eigen::Vector3d(pose[0], pose[1], pose[2]), 0, 0, 0, 1;
  return T;
}

std::vector<std::vector<float>> createPlannarPath(int cycle, int start_dir, float search_range)
{
  float x = 0, y = 0;
  std::vector<std::vector<float>> waypoints;
  waypoints.push_back({x, y});
  float directions[4][2] = {{0,1}, {1,0}, {0,-1}, {-1,0}};
  float step = search_range / cycle;
  for (int i=0; i < cycle; i++)
  {
    for (int d=0; d < 4; d++)
    {
      auto direction = directions[(d + start_dir) % 4];
      x += direction[0] * step * (floor(d / 2) + (i * 2) + 1);
      y += direction[1] * step * (floor(d / 2) + (i * 2) + 1);
      waypoints.push_back({x, y});
    }
  }
  x += directions[start_dir][0] * search_range * 2;
  y += directions[start_dir][1] * search_range * 2;
  waypoints.push_back({x, y});

  return waypoints;
};

bool EliteFtControl::ftPlannarSearch(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  auto wpts = createPlannarPath(5, 1, 20);
  auto target_pose = getCurrentPose();

  for(auto wpt : wpts)
  {
    target_pose[0] += wpt[0];
    target_pose[1] += wpt[1];
    eliteMovel(target_pose, 5);
    waitRobotMoving();
  }
  
  return true;
};
