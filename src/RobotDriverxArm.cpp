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

  for(std::size_t i = 0; i < dof_; ++i)
  {
    command[i] = static_cast<fp32>(q[i]);
  }

  checkReturnCode("set_servo_angle_j", arm_->set_servo_angle_j(command.data()));
}

void RobotDriverxArm::speedJ(const std::vector<double> &)
{
  throw std::runtime_error("[RobotDriverxArm] speedJ is not supported by the current "
                           "joint-position driver");
}

void RobotDriverxArm::tauJ(const std::vector<double> &)
{
  throw std::runtime_error("[RobotDriverxArm] tauJ is not supported because xarm torque-command "
                           "support has not been verified");
}

void RobotDriverxArm::checkReturnCode(const char * operation, int return_code) const
{
  if(return_code == 0)
  {
    return;
  }

  int error_warning[2] = {-1, -1};

  if(arm_ && arm_->is_connected())
  {
    arm_->get_err_warn_code(error_warning);
  }

  throw std::runtime_error(fmt::format("[RobotDriverxArm] {} failed: return_code={}, error_code={}, "
                                       "warn_code={}, mode={}, state={}, connected={}",
                                       operation, return_code, error_warning[0], error_warning[1],
                                       arm_ ? arm_->mode : -1, arm_ ? arm_->state : -1, arm_ && arm_->is_connected()));
}

void RobotDriverxArm::validateCommand(const char * operation, const std::vector<double> & command) const
{
  if(command.size() != dof_)
  {
    throw std::invalid_argument(
        fmt::format("[RobotDriverxArm] {} expected {} joint values, received {}", operation, dof_, command.size()));
  }

  const auto invalid_value =
      std::find_if(command.begin(), command.end(), [](double value) { return !std::isfinite(value); });

  if(invalid_value != command.end())
  {
    const auto index = static_cast<std::size_t>(std::distance(command.begin(), invalid_value));

    throw std::invalid_argument(
        fmt::format("[RobotDriverxArm] {} received a non-finite value at joint {}", operation, index));
  }
}

} // namespace xarm_driver

extern "C"
{

  void MC_RTC_ROBOT_DRIVER(std::vector<std::string> & classes)
  {
    classes.push_back("RobotDriverxArm");
  }

  mc_robot_interface::RobotDriver * create(const std::string &, const std::string & ip, const uint16_t & port)
  {
    return new xarm_driver::RobotDriverxArm(ip, port);
  }

  void destroy(mc_robot_interface::RobotDriver * driver)
  {
    delete driver;
  }

} // extern "C"
