/** VectorNav ROS2 Interface
 *
 * Copyright 2021 Dereck Wonnacott <dereck@gmail.com>
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#ifndef VECTORNAV__VECTORNAV_HPP_
#define VECTORNAV__VECTORNAV_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vectornav_msgs/msg/attitude_group.hpp>
#include <vectornav_msgs/msg/common_group.hpp>
#include <vectornav_msgs/msg/gps_group.hpp>
#include <vectornav_msgs/msg/imu_group.hpp>
#include <vectornav_msgs/msg/ins_group.hpp>
#include <vectornav_msgs/msg/time_group.hpp>
#include "vectornav_msgs/action/mag_cal.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include <vectornav_msgs/msg/imu.hpp>

// VectorNav libvncxx
#include "vn/compositedata.h"
#include "vn/sensors.h"

namespace vectornav {
  class Vectornav : public rclcpp::Node
  {
  public:
    using MagCal = vectornav_msgs::action::MagCal;
    using MagCalGH = rclcpp_action::ServerGoalHandle<MagCal>;

    explicit Vectornav(const rclcpp::NodeOptions & options);
    ~Vectornav();

  private:
    bool optimize_serial_communication(const std::string & portName);
    void reconnect_timer();
    bool configure_sensor();
    bool connect(const std::string port, const int baud);
    rclcpp::Time getTimeStamp(vn::sensors::CompositeData & data);
    bool isConfiguredLayout(vn::protocol::uart::Packet & packet);
    static void ErrorPacketReceivedHandler(
      void * nodeptr, vn::protocol::uart::Packet & errorPacket, size_t packetStartRunningIndex);
    static void AsyncPacketReceivedHandler(
      void * nodeptr, vn::protocol::uart::Packet & asyncPacket, size_t packetStartRunningIndex);
    void handle_cal_accept(const std::shared_ptr<MagCalGH> goal_handle);
    rclcpp_action::GoalResponse handle_cal_goal(
      const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MagCal::Goal> goal);
    rclcpp_action::CancelResponse handle_cal_cancel(const std::shared_ptr<MagCalGH> goal_handle);
    void execute_cal(const std::shared_ptr<MagCalGH> goal_handle);
    /**
    * Callback to take twist message and pass it to VN as velocity aiding 
    *
    * \param msg Shared pointer to ROS2 geometry_msgs/Twist message containing velocity information
    */
    void vel_aiding_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

    //
    // Parsing functions
    //
    static void parseCommonGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields,
  const rclcpp::Time & timestamp);
    static void parseTimeGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields,
  const rclcpp::Time & timestamp);
    static void parseImuGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields,
  const rclcpp::Time & timestamp);
    static void parseGpsGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields,
  const rclcpp::Time & timestamp);
    static void parseAttitudeGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields,
  const rclcpp::Time & timestamp);
    static void parseInsGroup(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields,
  const rclcpp::Time & timestamp);
    static void parseGps2Group(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields,
  const rclcpp::Time & timestamp);

    static void parseCommonAndIMU(
      Vectornav * node, vn::sensors::CompositeData & compositeData, uint16_t groupFields,
  const rclcpp::Time & timestamp);
    //
    // Helper Functions
    //
    static inline geometry_msgs::msg::Vector3 toMsg(const vn::math::vec3f & rhs);
    static inline geometry_msgs::msg::Quaternion toMsg(const vn::math::vec4f & rhs);
    static inline geometry_msgs::msg::Point toMsg(const vn::math::vec3d & rhs);
    static inline vectornav_msgs::msg::TimeUTC toMsg(const vn::protocol::uart::TimeUtc & rhs);
    static inline vectornav_msgs::msg::DOP toMsg(const vn::protocol::uart::GnssDop & rhs);
    static inline vectornav_msgs::msg::VpeStatus toMsg(const vn::protocol::uart::VpeStatus & rhs);
    static inline std::array<float, 9> toMsg(const vn::math::mat3f & rhs);
    static inline vectornav_msgs::msg::TimeStatus toMsg(const uint8_t rhs);
    static inline vectornav_msgs::msg::InsStatus toMsg(const vn::protocol::uart::InsStatus & rhs);

    /// Count the number of set bits in a number
    template <typename T>
    static uint countSetBits(T n)
      {
        T count = 0;
        while (n != 0) {
          n = n & (n - 1);
          count++;
        }
        return count;
      }

    //
    // Member Variables
    //

    /// VectorNav Sensor Handle
    std::shared_ptr<vn::sensors::VnSensor> vs_;

    /// Reconnection Timer
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    /// Publishers
    rclcpp::Publisher<vectornav_msgs::msg::CommonGroup>::SharedPtr pub_common_;
    rclcpp::Publisher<vectornav_msgs::msg::TimeGroup>::SharedPtr pub_time_;
    rclcpp::Publisher<vectornav_msgs::msg::ImuGroup>::SharedPtr pub_imu_;
    rclcpp::Publisher<vectornav_msgs::msg::GpsGroup>::SharedPtr pub_gps_;
    rclcpp::Publisher<vectornav_msgs::msg::AttitudeGroup>::SharedPtr pub_attitude_;
    rclcpp::Publisher<vectornav_msgs::msg::InsGroup>::SharedPtr pub_ins_;
    rclcpp::Publisher<vectornav_msgs::msg::GpsGroup>::SharedPtr pub_gps2_;
    rclcpp::Publisher<vectornav_msgs::msg::Imu>::SharedPtr pub_IMU_;

    /// ROS header time stamp adjustments
    double averageTimeDifference_{0};
    bool adjustROSTimeStamp_{false};
    rclcpp::Time adjustedTimeStamp(vn::sensors::CompositeData & data);

    /// Time stamps from a 1 Hz pulse on SYNC_IN whose edge falls on every whole second of the
    /// system clock (pps_timestamp). See ppsTimeStamp() in vectornav.cc.
    std::optional<rclcpp::Time> ppsTimeStamp(
      vn::sensors::CompositeData & data, const rclcpp::Time & fallback);
    void ppsReset();
    bool ppsEnabled_{false};
    double ppsMaxError_{0.05};  ///< s: how far an edge may be from the whole second
    double ppsEdgeDelay_{0.0};  ///< s: the edge's known delay after the whole second
    double ppsHoldover_{60.0};  ///< s: stamps carry on from the last pulse this long
    static constexpr int kPpsLockPulses = 3;
    /// The loop that follows the pulse once locked: settles within ~3 s (error poles 0.7 +- 0.1i).
    /// On Robin 12, with the sensor's late edges left out (kPpsLateNs), the stamps are within
    /// 2 us of the pulse half the time and 35 us 95 % of the time, also while chrony slews.
    static constexpr double kPpsPhaseGain = 0.5;
    static constexpr double kPpsRateGain = 0.1;
    static constexpr int64_t kPpsLateNs = 15000;  ///< an edge this much later than due is suspect
    bool ppsLateSkipped_{false};                  ///< the last edge was left out as late
    size_t ppsLateEdges_{0};
    bool ppsHaveCount_{false};
    uint32_t ppsLastCount_{0};    ///< SyncInCnt of the last edge seen
    uint64_t ppsLastStartup_{0};  ///< TimeStartup of the last packet: a restarted IMU goes back
    bool ppsHavePrev_{false};
    uint64_t ppsPrevEdge_{0};   ///< last accepted edge, IMU clock (ns since startup)
    int64_t ppsPrevSecond_{0};  ///< and the whole second it fell on
    int ppsGoodInRow_{0};
    bool ppsLocked_{false};
    uint64_t ppsRefEdge_{0};     ///< the edge stamps are counted from, IMU clock
    int64_t ppsRefNs_{0};        ///< and its system time (ns), following the pulse
    double ppsScale_{1.0};       ///< system-clock seconds per IMU-clock second
    uint64_t ppsAnchorEdge_{0};  ///< first edge of the pulses the rate is measured over to lock
    int64_t ppsAnchorSecond_{0};
    double ppsLastOffset_{0.0};  ///< s: the last pulse's second minus where the stamps put it
    size_t ppsStrayEdges_{0};
    /// Binary packet layouts (groups byte + group fields) the sensor was configured to send
    struct BinaryLayout
    {
      uint8_t groups;
      std::vector<uint16_t> fields;
    };
    std::vector<BinaryLayout> binaryLayouts_;
    std::mutex binaryLayoutsMutex_;
    size_t droppedPackets_{0};
    size_t consecutiveDropped_{0};
  
    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_aiding_;

    /// Action servers for calibration
    rclcpp_action::Server<vectornav_msgs::action::MagCal>::SharedPtr server_mag_cal_;
    std::thread action_thread_;

  };
}
#endif // VECTORNAV__VECTORNAV_HPP_
