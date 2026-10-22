#include <xarm_driver/RobotDriverxArm.h>

#include <xarm/wrapper/xarm_api.h>

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace
{

constexpr std::size_t max_xarm_dof = 7;
constexpr std::size_t min_xarm_dof = 5;
constexpr fp32 velocity_command_timeout = 0.05f;

} // namespace

namespace xarm_driver
{

RobotDriverxArm::RobotDriverxArm(const std::string & ip, uint16_t port)
: ip_(ip), port_(port), arm_(std::make_unique<XArmAPI>(ip_, true, true))
{
  if(ip_.empty())
  {
    throw std::invalid_argument("[RobotDriverxArm] Robot ip must not be empty");
  }

  const int connect_return_code = arm_->connect(ip_);

  checkReturnCode("connect", connect_return_code);

  if(!arm_->is_connected())
  {
    throw std::runtime_error(fmt::format("[RobotDriverxArm] connect returned success, but the robot at {} "
                                         "is not connected",
                                         ip_));
  }

  if(arm_->axis < min_xarm_dof || arm_->axis > static_cast<int>(max_xarm_dof))
  {
    throw std::runtime_error(fmt::format("[RobotDriverxArm] Invalid axis count reported by the robot: {}", arm_->axis));
  }

  dof_ = static_cast<std::size_t>(arm_->axis);

  actual_q_.resize(dof_);
  actual_qd_.resize(dof_);
  joint_torques_.resize(dof_);

  sync();

  if(arm_->error_code != 0)
  {
    const int clean_error_return_code = arm_->clean_error();

    checkReturnCode("clean_error", clean_error_return_code);
  }

  if(arm_->warn_code != 0)
  {
    const int clean_warn_return_code = arm_->clean_warn();

    checkReturnCode("clean_warn", clean_warn_return_code);
  }

  checkReturnCode("motion_enable", arm_->motion_enable(true));

  checkReturnCode("set_mode(1)", arm_->set_mode(1));

  checkReturnCode("set_state(0)", arm_->set_state(0));

  std::array<fp32, max_xarm_dof> hold_command{};

  for(std::size_t i = 0; i < dof_; ++i)
  {
    hold_command[i] = static_cast<fp32>(actual_q_[i]);
  }

  checkReturnCode("initial set_servo_angle_j", arm_->set_servo_angle_j(hold_command.data()));
}

RobotDriverxArm::~RobotDriverxArm()
{
  if(!arm_)
  {
    return;
  }

  if(arm_->is_connected())
  {
    arm_->set_state(4);
    arm_->disconnect();
  }
}

void RobotDriverxArm::sync()
{
  if(!arm_ || !arm_->is_connected())
  {
    throw std::runtime_error("[RobotDriverxArm] Cannot read joint state because the robot is not "
                             "connected");
  }

  std::array<fp32, max_xarm_dof> position{};
  std::array<fp32, max_xarm_dof> velocity{};
  std::array<fp32, max_xarm_dof> effort{};

  const int return_code = arm_->get_joint_states(position.data(), velocity.data(), effort.data(), 3);

  checkReturnCode("get_joint_states", return_code);

  for(std::size_t i = 0; i < dof_; ++i)
  {
    actual_q_[i] = static_cast<double>(position[i]);
    actual_qd_[i] = static_cast<double>(velocity[i]);
    joint_torques_[i] = static_cast<double>(effort[i]);
  }
}

std::vector<double> RobotDriverxArm::getActualQ()
{
  return actual_q_;
}

std::vector<double> RobotDriverxArm::getActualQd()
{
  return actual_qd_;
}

std::vector<double> RobotDriverxArm::getJointTorques()
{
  return joint_torques_;
}

void RobotDriverxArm::servoJ(const std::vector<double> & q)
{
  validateCommand("servoJ", q);

  std::array<fp32, max_xarm_dof> command{};

  std::transform(q.begin(), q.end(), command.begin(), [](double value) { return static_cast<fp32>(value); });

  const int code = arm_->set_servo_angle_j(command.data());

  if(code != 0)
  {
    throw std::runtime_error(
        "[RobotDriverxArm] "
        "set_servo_angle_j failed with code "
        + std::to_string(code)
        + "\nMore details at https://github.com/xArm-Developer/xArm-CPLUS-SDK/blob/master/doc/xarm_api_code.md");
  }
}

void RobotDriverxArm::speedJ(const std::vector<double> & velocity)
{
  validateCommand("speedJ", velocity);

  std::array<fp32, max_xarm_dof> command{};

  std::transform(velocity.begin(), velocity.end(), command.begin(),
                 [](double value) { return static_cast<fp32>(value); });

  const int code = arm_->vc_set_joint_velocity(command.data(), true, 0.05f);

  if(code != 0)
  {
    throw std::runtime_error(
        "[RobotDriverxArm] "
        "vc_set_joint_velocity failed with code "
        + std::to_string(code)
        + "\nMore details at https://github.com/xArm-Developer/xArm-CPLUS-SDK/blob/master/doc/xarm_api_code.md");
  }
}

void RobotDriverxArm::tauJ(const std::vector<double> &)
{
  throw std::runtime_error("[RobotDriverxArm] tauJ is not supported. Read more at "
                           "https://github.com/xArm-Developer/xArm-CPLUS-SDK/issues/18");
}

void RobotDriverxArm::validateCommand(const char * operation, const std::vector<double> & command) const
{
  if(command.size() != dof_)
  {
    throw std::invalid_argument(
        fmt::format("[RobotDriverxArm] {} expected {} joint values, received {}", operation, dof_, command.size()));
  }

  for(std::size_t i = 0; i < command.size(); ++i)
  {
    if(!std::isfinite(command[i]))
    {
      throw std::invalid_argument(
          fmt::format("[RobotDriverxArm] {} received a non-finite value at joint {}", operation, i));
    }
  }
}

} // namespace xarm_driver

// Lets robot_interface refuse this plugin (instead of crashing) once the
// RobotDriver interface changes and the plugin needs a rebuild.
ROBOT_DRIVER_EXPORT_ABI_VERSION()

extern "C"
{

  void ROBOT_DRIVER_PLUGIN(std::vector<std::string> & classes)
  {
    classes.push_back("RobotDriverxArm");
  }

  robot_interface::RobotDriver * create(const std::string &, const std::string & ip, const uint16_t & port)
  {
    return new xarm_driver::RobotDriverxArm(ip, port);
  }

  void destroy(robot_interface::RobotDriver * driver)
  {
    delete driver;
  }

} // extern "C"
