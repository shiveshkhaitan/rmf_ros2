/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATION_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATION_HPP

#include <rclcpp/node.hpp>

#include <rmf_traffic/schedule/NegotiationRoom.hpp>

#include <rmf_traffic_msgs/msg/negotiation_ack.hpp>
#include <rmf_traffic_msgs/msg/negotiation_repeat.hpp>
#include <rmf_traffic_msgs/msg/negotiation_notice.hpp>
#include <rmf_traffic_msgs/msg/negotiation_refusal.hpp>
#include <rmf_traffic_msgs/msg/negotiation_forfeit.hpp>
#include <rmf_traffic_msgs/msg/negotiation_proposal.hpp>
#include <rmf_traffic_msgs/msg/negotiation_rejection.hpp>
#include <rmf_traffic_msgs/msg/negotiation_conclusion.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic_msgs::msg::NegotiationForfeit convert(
  const rmf_traffic::schedule::NegotiationRoom::Forfeit& from);

//==============================================================================
rmf_traffic_msgs::msg::NegotiationKey convert(
  const rmf_traffic::schedule::NegotiationRoom::Key& from);

//==============================================================================
rmf_traffic_msgs::msg::NegotiationRejection convert(
  const rmf_traffic::schedule::NegotiationRoom::Rejection& from);

//==============================================================================
rmf_traffic_msgs::msg::NegotiationProposal convert(
  const rmf_traffic::schedule::NegotiationRoom::Proposal& from);

//==============================================================================
std::vector<rmf_traffic_msgs::msg::NegotiationKey> convert(
  const std::vector<rmf_traffic::schedule::NegotiationRoom::Key>& from);

//==============================================================================
rmf_traffic::schedule::NegotiationRoom::Conclusion convert(
  const rmf_traffic_msgs::msg::NegotiationConclusion& from);

//==============================================================================
rmf_traffic::schedule::NegotiationRoom::Key convert(
  const rmf_traffic_msgs::msg::NegotiationKey& from);

//==============================================================================
rmf_traffic::schedule::Negotiation::VersionedKeySequence convert(
  const std::vector<rmf_traffic_msgs::msg::NegotiationKey>& from);

//==============================================================================
std::vector<rmf_traffic_msgs::msg::NegotiationKey> convert(
  const rmf_traffic::schedule::Negotiation::VersionedKeySequence& from);

//==============================================================================
rmf_traffic_msgs::msg::NegotiationRefusal convert(
  const rmf_traffic::schedule::NegotiationRoom::Refusal& from);

//==============================================================================
rmf_traffic_msgs::msg::NegotiationAck convert(
  const rmf_traffic::schedule::NegotiationRoom::Ack& from);

//==============================================================================
rmf_traffic::schedule::NegotiationRoom::Repeat convert(
  const rmf_traffic_msgs::msg::NegotiationRepeat& from);

//==============================================================================
rmf_traffic::schedule::NegotiationRoom::Notice convert(
  const rmf_traffic_msgs::msg::NegotiationNotice& from);

//==============================================================================
rmf_traffic::schedule::NegotiationRoom::Proposal convert(
  const rmf_traffic_msgs::msg::NegotiationProposal& from);

//==============================================================================
rmf_traffic::schedule::NegotiationRoom::Rejection convert(
  const rmf_traffic_msgs::msg::NegotiationRejection& from);

//==============================================================================
rmf_traffic::schedule::NegotiationRoom::Forfeit convert(
  const rmf_traffic_msgs::msg::NegotiationForfeit& from);

namespace schedule {

//==============================================================================
/// A ROS2 interface for negotiating solutions to schedule conflicts
class Negotiation
{
public:
  /// Constructor
  ///
  /// \param[in] worker
  ///   If a worker is provided, the Negotiation will be performed
  ///   asynchronously. If it is not provided, then the Negotiators must be
  ///   single-threaded, and their respond() functions must block until
  ///   finished.
  Negotiation(
    rclcpp::Node& node,
    std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer,
    std::shared_ptr<rmf_traffic::schedule::NegotiationRoom::Worker> worker = nullptr);

  /// Set the timeout duration for negotiators. If a negotiator does not respond
  /// within this time limit, then the negotiation will automatically be
  /// forfeited. This is important to prevent negotiations from getting hung
  /// forever.
  Negotiation& timeout_duration(rmf_traffic::Duration duration);

  /// Get the current timeout duration setting.
  rmf_traffic::Duration timeout_duration() const;

  using TableViewPtr = rmf_traffic::schedule::Negotiation::Table::ViewerPtr;
  using StatusUpdateCallback =
    std::function<void (uint64_t conflict_version, TableViewPtr table_view)>;
  
  /// Register a callback with this Negotiation manager that triggers
  /// on negotiation status updates.
  ///
  /// \param[in] cb
  ///   The callback function to be called upon status updates.
  void on_status_update(StatusUpdateCallback cb);

  using StatusConclusionCallback =
    std::function<void (uint64_t conflict_version, bool success)>;
  
  /// Register a callback with this Negotiation manager that triggers
  /// on negotiation status conclusions.
  ///
  /// \param[in] cb
  ///   The callback function to be called upon status conclusions.
  void on_conclusion(StatusConclusionCallback cb);

  /// Get a Negotiation::TableView that provides a view into what participants are
  /// proposing.
  ///
  /// This function does not care about table versioning.
  /// \param[in] conflict_version
  ///   The conflict version of the negotiation
  /// \param[in] sequence
  ///   The sequence of participant ids. Follows convention of other sequences
  ///   (ie. The last ParticipantId is the owner of the table)
  ///
  /// \return A TableView into what participants are proposing.
  TableViewPtr table_view(
    uint64_t conflict_version,
    const std::vector<rmf_traffic::schedule::ParticipantId>& sequence) const;

  /// Set the number of negotiations to retain.
  ///
  /// \param[in] count
  ///   The number of negotiations to retain
  void set_retained_history_count(uint count);

  /// Register a negotiator with this Negotiation manager.
  ///
  /// \param[in] for_participant
  ///   The ID of the participant that this negotiator will work for
  ///
  /// \param[in] negotiator
  ///   The negotiator interface to use for this participant
  ///
  /// \return a handle that should be kept by the caller. When this handle
  /// expires, this negotiator will be automatically unregistered.
  std::shared_ptr<void> register_negotiator(
    rmf_traffic::schedule::ParticipantId for_participant,
    std::unique_ptr<rmf_traffic::schedule::Negotiator> negotiator);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATION_HPP
