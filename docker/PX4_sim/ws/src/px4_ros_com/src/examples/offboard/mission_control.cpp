/****************************************************************************
 *
 * Copyright 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Autonomous mission using the PX4 ROS 2 Interface Library
 * @file mission_control.cpp
 *
 * Full mission sequence:
 *   arm -> takeoff -> hover -> fly to position -> hover -> land
 *
 * MissionMode   - ModeBase subclass, flies to a configurable NED target and
 *                 hovers there for a set duration before signalling completion.
 * MissionExecutor - ModeExecutorBase subclass, sequences the built-in
 *                 takeoff/land modes with MissionMode to run the full mission.
 */

#include <Eigen/Core>
#include <cmath>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;  // NOLINT

// ---------------------------------------------------------------------------
// Mission parameters — adjust these for your flight
// ---------------------------------------------------------------------------
static constexpr float kHoverAltitudeNed = -5.f;   // NED -5 m = 5 m altitude
static constexpr float kTargetNorthM     = 10.f;   // fly 10 m north
static constexpr float kTargetEastM      =  0.f;
static constexpr float kHoverDurationS   =  5.f;   // hold each waypoint for 5 s
static constexpr float kPositionTolM     =  0.5f;  // position acceptance radius [m]
static constexpr float kVelocityTolMs    =  0.3f;  // velocity acceptance threshold [m/s]

static const std::string kModeName = "Mission Mode";

// ===========================================================================
// MissionMode
// Flies to a target NED position and yaw, then hovers for kHoverDurationS
// before calling completed() to hand control back to the executor.
// ===========================================================================
class MissionMode : public px4_ros2::ModeBase
{
public:
  explicit MissionMode(rclcpp::Node & node)
  : ModeBase(node, Settings{kModeName})
  {
    _goto_setpoint  = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
    _local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  }

  // Called by MissionExecutor before each scheduleMode() call.
  void setTarget(const Eigen::Vector3f & position_ned, float yaw_rad)
  {
    _target_position = position_ned;
    _target_yaw      = yaw_rad;
  }

  // -------------------------------------------------------------------------
  void onActivate() override
  {
    _hover_elapsed_s  = 0.f;
    _position_reached = false;
    RCLCPP_INFO(
      node().get_logger(),
      "MissionMode activated — target NED [%.1f, %.1f, %.1f] yaw %.1f deg",
      _target_position.x(), _target_position.y(), _target_position.z(),
      _target_yaw * 180.f / static_cast<float>(M_PI));
  }

  // -------------------------------------------------------------------------
  void updateSetpoint(float dt_s) override
  {
    // Keep sending the setpoint every tick (required by PX4)
    _goto_setpoint->update(_target_position, _target_yaw);

    if (!_position_reached && positionReached()) {
      _position_reached = true;
      RCLCPP_INFO(node().get_logger(), "Position reached, hovering for %.0f s", kHoverDurationS);
    }

    if (_position_reached) {
      _hover_elapsed_s += dt_s;
      if (_hover_elapsed_s >= kHoverDurationS) {
        completed(px4_ros2::Result::Success);
      }
    }
  }

private:
  bool positionReached() const
  {
    const Eigen::Vector3f error = _target_position - _local_position->positionNed();
    return error.norm() < kPositionTolM &&
           _local_position->velocityNed().norm() < kVelocityTolMs;
  }

  Eigen::Vector3f _target_position{0.f, 0.f, kHoverAltitudeNed};
  float           _target_yaw{static_cast<float>(M_PI)};
  float           _hover_elapsed_s{0.f};
  bool            _position_reached{false};

  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition>       _local_position;
};

// ===========================================================================
// MissionExecutor
// Owns a MissionMode instance and drives the full mission sequence:
//   takeoff -> hover at origin -> fly to position -> hover -> land
// ===========================================================================
class MissionExecutor : public px4_ros2::ModeExecutorBase
{
public:
  explicit MissionExecutor(MissionMode & owned_mode)
  : ModeExecutorBase(Settings{}.activate(Settings::Activation::ActivateImmediately), owned_mode),
    _node(owned_mode.node()),
    _mission_mode(owned_mode)
  {}

  enum class State {
    Reset,
    WaitReadyToArm,
    Arming,
    TakingOff,
    HoverAtTakeoff,   // hover at origin at cruise altitude
    FlyToPosition,    // fly north then hover
    Landing,
    WaitUntilDisarmed,
  };

  // Called automatically by NodeWithModeExecutor once PX4 is ready
  void onActivate() override
  {
    runState(State::WaitReadyToArm, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason /*reason*/) override {}

  // -------------------------------------------------------------------------
  void runState(State state, px4_ros2::Result prev_result)
  {
    if (prev_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(
        _node.get_logger(), "State %i: previous state failed: %s",
        static_cast<int>(state), resultToString(prev_result));
      return;
    }

    RCLCPP_INFO(_node.get_logger(), "--- State: %i ---", static_cast<int>(state));

    switch (state) {
      // ------------------------------------------------------------------
      case State::Reset:
        break;

      // ------------------------------------------------------------------
      case State::WaitReadyToArm:
        waitReadyToArm([this](px4_ros2::Result r) { runState(State::Arming, r); });
        break;

      // ------------------------------------------------------------------
      // Arm with retry: on first attempt PX4 may not yet have received our
      // custom mode's arming-check reply, so retry after 500 ms until it does.
      case State::Arming:
        arm([this](px4_ros2::Result r) {
          if (r == px4_ros2::Result::Success) {
            runState(State::TakingOff, r);
          } else {
            RCLCPP_WARN(_node.get_logger(), "Arm rejected, retrying in 500 ms...");
            _retry_timer = _node.create_wall_timer(
              std::chrono::milliseconds(500),
              [this]() {
                _retry_timer->cancel();
                runState(State::Arming, px4_ros2::Result::Success);
              });
          }
        });
        break;

      // ------------------------------------------------------------------
      // Built-in takeoff: climbs to MIS_TAKEOFF_ALT (vehicle is already armed).
      case State::TakingOff:
        takeoff([this](px4_ros2::Result r) { runState(State::HoverAtTakeoff, r); });
        break;

      // ------------------------------------------------------------------
      // Hover at the origin (above the takeoff point) for kHoverDurationS.
      // Yaw = pi rad (facing south), matching the original example.
      case State::HoverAtTakeoff:
        _mission_mode.setTarget(
          {0.f, 0.f, kHoverAltitudeNed},
          static_cast<float>(M_PI));
        scheduleMode(
          ownedMode().id(),
          [this](px4_ros2::Result r) { runState(State::FlyToPosition, r); });
        break;

      // ------------------------------------------------------------------
      // Fly to a position 10 m north, hover there, then land.
      // Yaw = 0 rad (facing north) during the transit.
      case State::FlyToPosition:
        _mission_mode.setTarget(
          {kTargetNorthM, kTargetEastM, kHoverAltitudeNed},
          0.f);
        scheduleMode(
          ownedMode().id(),
          [this](px4_ros2::Result r) { runState(State::Landing, r); });
        break;

      // ------------------------------------------------------------------
      // Built-in land: descends and disarms at the current position.
      case State::Landing:
        land([this](px4_ros2::Result r) { runState(State::WaitUntilDisarmed, r); });
        break;

      // ------------------------------------------------------------------
      case State::WaitUntilDisarmed:
        waitUntilDisarmed([this](px4_ros2::Result r) {
          RCLCPP_INFO(_node.get_logger(), "Mission complete (%s)", resultToString(r));
        });
        break;
    }
  }

private:
  rclcpp::Node & _node;
  MissionMode  & _mission_mode;
  rclcpp::TimerBase::SharedPtr _retry_timer;
};

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<px4_ros2::NodeWithModeExecutor<MissionExecutor, MissionMode>>(
      "mission_control", true));
  rclcpp::shutdown();
  return 0;
}
