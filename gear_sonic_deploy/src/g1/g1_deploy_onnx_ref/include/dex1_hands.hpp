/**
 * @file dex1_hands.hpp
 * @brief Driver for the Unitree Dex1-1 robotic grippers (left + right).
 *
 * Dex1Hands manages two 1-DOF Unitree Dex1-1 parallel grippers via the Unitree
 * DDS channel API (`unitree_go::msg::dds_::MotorCmds_` and `MotorStates_`),
 * matching the official `dex1_1_service` specification:
 *  - Right gripper (motor_id = 0): cmd `rt/dex1/right/cmd`, state `rt/dex1/right/state`
 *  - Left gripper  (motor_id = 1): cmd `rt/dex1/left/cmd`,  state `rt/dex1/left/state`
 *
 * ## Joint Angle Convention (`dex1_1_service`)
 *
 *  - `Q_CLOSED = 0.0` rad (calibrated with gripper fully closed)
 *  - `Q_OPEN   = 5.40` rad (maximum open stroke ~9 cm at 0.6 rad/cm)
 *
 * ## Close-Ratio Limiting
 *
 * A runtime-adjustable `max_close_ratio_` (range [0.2, 1.0]) limits how
 * far the gripper can close. 1.0 allows full closure (`0.0` rad); 0.2 keeps it
 * at most 20% closed (`4.32` rad). Controlled via `--max-close-ratio` CLI arg
 * and X/C keys.
 *
 * ## DDS Topics
 *
 *   Direction | Left gripper          | Right gripper
 *   ----------|-----------------------|------------------------
 *   Command   | rt/dex1/left/cmd      | rt/dex1/right/cmd
 *   State     | rt/dex1/left/state    | rt/dex1/right/state
 */

#ifndef DEX1_HANDS_HPP
#define DEX1_HANDS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "utils.hpp"

static constexpr int DEX1_MOTOR_MAX = 1;  ///< Number of motors per Dex1-1 gripper.

/**
 * @class Dex1Hands
 * @brief Manages two Unitree Dex1-1 grippers (left + right) over DDS channels.
 *
 * Does not run its own thread – the owning class calls `writeOnce()` at the
 * desired cadence from the command-writer thread.
 */
class Dex1Hands
{
public:
    static constexpr double Q_CLOSED = 0.0;        ///< Fully closed position (rad).
    static constexpr double Q_OPEN = 5.40;         ///< Fully open position (rad).
    static constexpr double DEFAULT_KP = 5.0;      ///< Default position gain for M4010 gripper motor.
    static constexpr double DEFAULT_KD = 0.05;     ///< Default velocity gain for M4010 gripper motor.
    static constexpr double MAX_DELTA_Q = 0.18;    ///< Maximum per-tick position delta (rad).

    Dex1Hands() = default;

    // Initializes channels for both grippers. If networkInterface is empty, skips ChannelFactory init.
    void initialize(const std::string &networkInterface)
    {
        if (!networkInterface.empty())
        {
            unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface.c_str());
        }

        // Left gripper namespaces (dex1_1_service: motor_id = 1)
        const std::string leftCmdTopic = "rt/dex1/left/cmd";
        const std::string leftStateTopic = "rt/dex1/left/state";
        // Right gripper namespaces (dex1_1_service: motor_id = 0)
        const std::string rightCmdTopic = "rt/dex1/right/cmd";
        const std::string rightStateTopic = "rt/dex1/right/state";

        // Initialize left gripper cmd with default open value
        unitree_go::msg::dds_::MotorCmds_ left_cmd;
        sizeCommand(left_cmd);
        left_.cmd_buffer.SetData(left_cmd);

        // Initialize right gripper cmd with default open value
        unitree_go::msg::dds_::MotorCmds_ right_cmd;
        sizeCommand(right_cmd);
        right_.cmd_buffer.SetData(right_cmd);

        // Left gripper DDS pub/sub
        left_.publisher.reset(
            new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>(leftCmdTopic));
        left_.subscriber.reset(
            new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>(leftStateTopic));
        left_.publisher->InitChannel();
        left_.subscriber->InitChannel(
            [this](const void *message) { this->onState(true, message); }, 1);

        // Right gripper DDS pub/sub
        right_.publisher.reset(
            new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>(rightCmdTopic));
        right_.subscriber.reset(
            new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>(rightStateTopic));
        right_.publisher->InitChannel();
        right_.subscriber->InitChannel(
            [this](const void *message) { this->onState(false, message); }, 1);

        std::cout << "[Dex1Hands] Initialized DDS channels: "
                  << leftCmdTopic << ", " << rightCmdTopic << std::endl;
    }

    // Set max close ratio at runtime (bounded to [0.2, 1.0])
    void SetMaxCloseRatio(double ratio)
    {
        max_close_ratio_ = std::max(0.2, std::min(1.0, ratio));
    }

    // Get current max close ratio
    double GetMaxCloseRatio() const { return max_close_ratio_; }

    // Perform one publish tick; call this at your own cadence from the main class/thread.
    void writeOnce()
    {
        publishHand(left_);
        publishHand(right_);
    }

    // Returns a snapshot of the latest state for the requested gripper.
    std::shared_ptr<const unitree_go::msg::dds_::MotorStates_> getState(bool is_left) const
    {
        const HandCtx &ctx = is_left ? left_ : right_;
        return ctx.state_buffer.GetDataWithTime().data;
    }

    // Returns a snapshot of the current command buffer for the requested gripper.
    std::shared_ptr<const unitree_go::msg::dds_::MotorCmds_> getCommand(bool is_left) const
    {
        const HandCtx &ctx = is_left ? left_ : right_;
        return ctx.cmd_buffer.GetDataWithTime().data;
    }

    TimestampedData<unitree_go::msg::dds_::MotorStates_> getStateWithTime(bool is_left) const
    {
        const HandCtx &ctx = is_left ? left_ : right_;
        return ctx.state_buffer.GetDataWithTime();
    }

    TimestampedData<unitree_go::msg::dds_::MotorCmds_> getCommandWithTime(bool is_left) const
    {
        const HandCtx &ctx = is_left ? left_ : right_;
        return ctx.cmd_buffer.GetDataWithTime();
    }

    bool hasState(bool is_left) const { return getStateWithTime(is_left).HasData(); }
    bool hasCommand(bool is_left) const { return getCommandWithTime(is_left).HasData(); }

    // Sets the raw MotorCmds_ command buffer for the requested gripper.
    void setCommand(bool is_left, const unitree_go::msg::dds_::MotorCmds_ &cmd)
    {
        HandCtx &ctx = is_left ? left_ : right_;
        if (static_cast<int>(cmd.cmds().size()) != DEX1_MOTOR_MAX)
        {
            std::cerr << "[Dex1Hands] setCommand: invalid cmds size "
                      << cmd.cmds().size() << ", expected " << DEX1_MOTOR_MAX << std::endl;
            return;
        }
        ctx.cmd_buffer.SetData(cmd);
    }

    // Set gripper command directly in radians [Q_CLOSED=0.0, Q_OPEN=5.40].
    void setGripperCommand(bool is_left,
                           double q,
                           std::optional<double> dq = std::nullopt,
                           std::optional<double> kp = std::nullopt,
                           std::optional<double> kd = std::nullopt,
                           std::optional<double> tau = std::nullopt)
    {
        HandCtx &ctx = is_left ? left_ : right_;
        const auto currentPtr = ctx.cmd_buffer.GetDataWithTime().data;
        unitree_go::msg::dds_::MotorCmds_ cmd =
            currentPtr ? *currentPtr : unitree_go::msg::dds_::MotorCmds_();
        if (!currentPtr || static_cast<int>(cmd.cmds().size()) != DEX1_MOTOR_MAX)
        {
            sizeCommand(cmd);
        }
        auto &m = cmd.cmds()[0];
        m.mode(1);
        m.q(static_cast<float>(std::clamp(q, Q_CLOSED, Q_OPEN)));
        if (dq) { m.dq(static_cast<float>(*dq)); }
        if (kp) { m.kp(static_cast<float>(*kp)); }
        if (kd) { m.kd(static_cast<float>(*kd)); }
        if (tau) { m.tau(static_cast<float>(*tau)); }
        ctx.cmd_buffer.SetData(std::move(cmd));
    }

    /**
     * @brief Compatibility setter accepting the 7-element hand joint buffer used
     *        by the VR teleoperation pipeline (`G1GripperInverseKinematicsSolver`).
     *
     * Maps the 7-DOF Dex3 representation to the 1-DOF Dex1-1 gripper target:
     *  - In VR teleop (`pico_manager_thread_server.py`), open hand is all zeros
     *    (`q = {0, 0, 0, 0, 0, 0, 0}`) and closed hand sets index/middle joints
     *    (`q[3] = ±1.0`, `q[4] = ±1.5`).
     *  - `close_ratio = clamp(|q[4]| / 1.5, 0.0, 1.0)` (or if `q[1..6]` are all 0
     *    and `q[0] > 0`, interprets `q[0]` in `[0, 1]` as closure ratio or `> 1` as rad).
     *  - Target angle `q_target = Q_OPEN * (1.0 - close_ratio)`.
     */
    void setAllJointsCommand(bool is_left, const std::array<double, 7> &q)
    {
        bool has_dex3_pattern = false;
        for (size_t i = 1; i < 7; ++i)
        {
            if (std::abs(q[i]) > 1e-4)
            {
                has_dex3_pattern = true;
                break;
            }
        }

        double desired_q = Q_OPEN;
        if (has_dex3_pattern)
        {
            const double close_ratio = std::clamp(std::abs(q[4]) / 1.5, 0.0, 1.0);
            desired_q = Q_OPEN - close_ratio * (Q_OPEN - Q_CLOSED);
        }
        else if (std::abs(q[0]) > 1e-4)
        {
            if (q[0] >= 0.0 && q[0] <= 1.0)
            {
                desired_q = Q_OPEN - q[0] * (Q_OPEN - Q_CLOSED);
            }
            else
            {
                desired_q = std::clamp(q[0], Q_CLOSED, Q_OPEN);
            }
        }

        setGripperCommand(is_left, desired_q);
    }

    // Quick helper: STOP - relax gripper motor
    void stop(bool is_left)
    {
        HandCtx &ctx = is_left ? left_ : right_;
        unitree_go::msg::dds_::MotorCmds_ cmd;
        sizeCommand(cmd);
        auto &m = cmd.cmds()[0];
        m.mode(0);
        m.tau(0.0f);
        m.dq(0.0f);
        m.kp(0.0f);
        m.kd(0.0f);
        m.q(0.0f);
        ctx.cmd_buffer.SetData(std::move(cmd));
    }

    // Quick helper: HOLD - hold current gripper position with gains
    void hold(bool is_left, double kp = DEFAULT_KP, double kd = DEFAULT_KD)
    {
        double q_current = Q_OPEN;
        {
            const HandCtx &ctx_ro = is_left ? left_ : right_;
            const auto data = ctx_ro.state_buffer.GetDataWithTime().data;
            if (!data || data->states().empty()) { return; }
            q_current = data->states()[0].q();
        }
        setGripperCommand(is_left, q_current, 0.0, kp, kd, 0.0);
    }

    // Quick helper: CLOSE - move to mid-range pose during INIT (matching Dex3Hands::close)
    void close(bool is_left, double kp = DEFAULT_KP, double kd = DEFAULT_KD)
    {
        const double mid = (Q_OPEN + Q_CLOSED) / 2.0;
        setGripperCommand(is_left, mid, 0.0, kp, kd, 0.0);
    }

    // Quick helper: OPEN - move to fully open pose (Q_OPEN = 5.40 rad)
    void open(bool is_left, double kp = DEFAULT_KP, double kd = DEFAULT_KD)
    {
        setGripperCommand(is_left, Q_OPEN, 0.0, kp, kd, 0.0);
    }

private:
    struct HandCtx
    {
        unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::MotorCmds_> publisher;
        unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::MotorStates_> subscriber;

        DataBuffer<unitree_go::msg::dds_::MotorStates_> state_buffer;
        DataBuffer<unitree_go::msg::dds_::MotorCmds_> cmd_buffer;
    };

    static void sizeCommand(unitree_go::msg::dds_::MotorCmds_ &cmd)
    {
        cmd.cmds().resize(DEX1_MOTOR_MAX);
        cmd.cmds()[0].mode(1);
        cmd.cmds()[0].q(static_cast<float>(Q_OPEN));
        cmd.cmds()[0].dq(0.0f);
        cmd.cmds()[0].kp(static_cast<float>(DEFAULT_KP));
        cmd.cmds()[0].kd(static_cast<float>(DEFAULT_KD));
        cmd.cmds()[0].tau(0.0f);
    }

    void onState(bool is_left, const void *message)
    {
        HandCtx &ctx = is_left ? left_ : right_;
        const auto *incoming = static_cast<const unitree_go::msg::dds_::MotorStates_ *>(message);
        ctx.state_buffer.SetData(*incoming);
    }

    // Clip desired_q to prevent closing beyond max_close_ratio_
    // In Dex1-1, Q_OPEN (5.40) is fully open and Q_CLOSED (0.0) is fully closed.
    // max_close_ratio_ = 1.0 -> min allowed q is 0.0 (full closure)
    // max_close_ratio_ = 0.2 -> min allowed q is 5.40 * 0.8 = 4.32 (20% closed)
    static double clipToMaxOpen(double desired_q, double max_close_ratio)
    {
        const double min_allowed_q = Q_OPEN - max_close_ratio * (Q_OPEN - Q_CLOSED);
        return std::clamp(desired_q, min_allowed_q, Q_OPEN);
    }

    void publishHand(HandCtx &ctx)
    {
        const auto cmdPtr = ctx.cmd_buffer.GetDataWithTime().data;
        const auto statePtr = ctx.state_buffer.GetDataWithTime().data;

        if (ctx.publisher && cmdPtr && static_cast<int>(cmdPtr->cmds().size()) == DEX1_MOTOR_MAX)
        {
            unitree_go::msg::dds_::MotorCmds_ smoothedCmd = *cmdPtr;

            double desired_q = cmdPtr->cmds()[0].q();
            desired_q = clipToMaxOpen(desired_q, max_close_ratio_);
            smoothedCmd.cmds()[0].q(static_cast<float>(desired_q));

            if (statePtr && static_cast<int>(statePtr->states().size()) >= DEX1_MOTOR_MAX)
            {
                const double current_q = statePtr->states()[0].q();
                const double delta = desired_q - current_q;
                const double clamped_delta = std::max(-MAX_DELTA_Q, std::min(MAX_DELTA_Q, delta));
                smoothedCmd.cmds()[0].q(static_cast<float>(current_q + clamped_delta));
            }

            ctx.publisher->Write(smoothedCmd);
        }
    }

    HandCtx left_;
    HandCtx right_;
    double max_close_ratio_ = 1.0;
};

#endif // DEX1_HANDS_HPP
