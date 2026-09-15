#pragma once

#include <robot_interface/RobotDriverTemplate.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class XArmAPI;

namespace xarm_driver
{

class RobotDriverxArm : public mc_robot_interface::RobotDriver
{
public:
  RobotDriverxArm(const std::string & ip, uint16_t port);

  ~RobotDriverxArm() override;

  void sync() override;

  void setDataRead() override {}

  std::vector<double> getActualQ() override;

  std::vector<double> getActualQd() override;

  std::vector<double> getJointTorques() override;

  void servoJ(const std::vector<double> & q) override;

  void speedJ(const std::vector<double> & velocity) override;

  void tauJ(const std::vector<double> & torque) override;

private:
  void checkReturnCode(const char * operation, int return_code) const;

  void validateCommand(const char * operation, const std::vector<double> & command) const;

private:
  std::string ip_;
  uint16_t port_;
  std::size_t dof_ = 0;

  std::unique_ptr<XArmAPI> arm_;

  std::vector<double> actual_q_;
  std::vector<double> actual_qd_;
  std::vector<double> joint_torques_;
};

} // namespace xarm_driver

#include <robot_interface/driver/api.h>

extern "C"
{
  MC_ROBOT_DRIVER_DLLAPI void MC_RTC_ROBOT_DRIVER(std::vector<std::string> & classes);

  MC_ROBOT_DRIVER_DLLAPI mc_robot_interface::RobotDriver * create(const std::string & name,
                                                                  const std::string & ip,
                                                                  const uint16_t & port);

  MC_ROBOT_DRIVER_DLLAPI void destroy(mc_robot_interface::RobotDriver * driver);
}
