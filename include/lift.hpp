#pragma once
#include "StateMachine.hpp"
#include "TaskWrapper.hpp"
#include "lemlib/pid.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include <memory>

namespace lib {

enum class LiftState { Stored, Recieve, Score, Manual, Custom};

// pid constant struct
struct PIDConstants {
  double kP;
  double kI;
  double kD;
};

class Lift : public StateMachine<LiftState, LiftState::Stored>,
             public ryan::TaskWrapper {

private:
  const float DOWN_ANGLE = 0;
  const float MID_ANGLE = 48;
  const float MIDD_ANGLE = 100;
  const float UP_ANGLE = 225;

  const float gearRatio;

  const PIDConstants constants;

  double target;
  int vol;

  lemlib::PID pid = lemlib::PID(constants.kP, constants.kI, constants.kD);

public:
  std::shared_ptr<pros::MotorGroup> motors;
  Lift(pros::MotorGroup *motors, double gearRatio, PIDConstants constants)
      : motors(motors), gearRatio(gearRatio), constants(constants){
    motors->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  void loop() override;

  float getAngle();
  void setAngle(float angle);
  void itterateState(bool delta);
  void setTarget(float target);
  void setVoltage(int voltage);
};

} // namespace lib
// namespace lib