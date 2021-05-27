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

#include <rmf_traffic_ros2/Route.hpp>
#include <rmf_traffic_ros2/schedule/Itinerary.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rclcpp/logging.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
class Negotiation::Implementation
{
public:
  rclcpp::Node& node;
  rmf_traffic::schedule::NegotiationRoom negotiation_room;
  rmf_traffic::Duration timeout = std::chrono::seconds(15);
  std::shared_ptr<rmf_traffic::schedule::NegotiationRoom::SubscriptionCallbacks>
  subscription_callbacks =
    std::make_shared<rmf_traffic::schedule::NegotiationRoom::SubscriptionCallbacks>();


  using Repeat = rmf_traffic_msgs::msg::NegotiationRepeat;
  using RepeatSub = rclcpp::Subscription<Repeat>;
  using RepeatPub = rclcpp::Publisher<Repeat>;
  RepeatSub::SharedPtr repeat_sub;
  RepeatPub::SharedPtr repeat_pub;

  using Notice = rmf_traffic_msgs::msg::NegotiationNotice;
  using NoticeSub = rclcpp::Subscription<Notice>;
  using NoticePub = rclcpp::Publisher<Notice>;
  NoticeSub::SharedPtr notice_sub;
  NoticePub::SharedPtr notice_pub;

  using Refusal = rmf_traffic_msgs::msg::NegotiationRefusal;
  using RefusalPub = rclcpp::Publisher<Refusal>;
  RefusalPub::SharedPtr refusal_pub;

  using Proposal = rmf_traffic_msgs::msg::NegotiationProposal;
  using ProposalSub = rclcpp::Subscription<Proposal>;
  using ProposalPub = rclcpp::Publisher<Proposal>;
  ProposalSub::SharedPtr proposal_sub;
  ProposalPub::SharedPtr proposal_pub;

  using Rejection = rmf_traffic_msgs::msg::NegotiationRejection;
  using RejectionSub = rclcpp::Subscription<Rejection>;
  using RejectionPub = rclcpp::Publisher<Rejection>;
  RejectionSub::SharedPtr rejection_sub;
  RejectionPub::SharedPtr rejection_pub;

  using Forfeit = rmf_traffic_msgs::msg::NegotiationForfeit;
  using ForfeitSub = rclcpp::Subscription<Forfeit>;
  using ForfeitPub = rclcpp::Publisher<Forfeit>;
  ForfeitSub::SharedPtr forfeit_sub;
  ForfeitPub::SharedPtr forfeit_pub;

  using Conclusion = rmf_traffic_msgs::msg::NegotiationConclusion;
  using ConclusionSub = rclcpp::Subscription<Conclusion>;
  ConclusionSub::SharedPtr conclusion_sub;

  using Ack = rmf_traffic_msgs::msg::NegotiationAck;
  using AckPub = rclcpp::Publisher<Ack>;
  AckPub::SharedPtr ack_pub;

  rmf_traffic::schedule::NegotiationRoom::PublisherCallbacks publisher_callbacks
  {
    std::bind(&Negotiation::Implementation::send_refusal, this,
      std::placeholders::_1),
    std::bind(&Negotiation::Implementation::send_ack, this,
      std::placeholders::_1),
    std::bind(&Negotiation::Implementation::send_proposal, this,
      std::placeholders::_1),
    std::bind(&Negotiation::Implementation::send_rejection, this,
      std::placeholders::_1),
    std::bind(&Negotiation::Implementation::send_forfeit, this,
      std::placeholders::_1),
  };

  Implementation(
    rclcpp::Node& node_,
    std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer_,
    std::shared_ptr<rmf_traffic::schedule::NegotiationRoom::Worker> worker_)
  : node(node_),
    negotiation_room(
      std::move(viewer_),
      std::move(publisher_callbacks),
      subscription_callbacks,
      std::move(worker_))
  {
    // TODO(MXG): Make the QoS configurable
    const auto qos = rclcpp::ServicesQoS().reliable().keep_last(1000);

    repeat_sub = node.create_subscription<Repeat>(
      NegotiationRepeatTopicName, qos,
      [&](const Repeat::UniquePtr msg)
      {
        this->receive_repeat_request(*msg);
      });

    repeat_pub = node.create_publisher<Repeat>(
      NegotiationRepeatTopicName, qos);

    notice_sub = node.create_subscription<Notice>(
      NegotiationNoticeTopicName, qos,
      [&](const Notice::UniquePtr msg)
      {
        this->receive_notice(*msg);
      });

    notice_pub = node.create_publisher<Notice>(
      NegotiationNoticeTopicName, qos);

    refusal_pub = node.create_publisher<Refusal>(
      NegotiationRefusalTopicName, qos);

    proposal_sub = node.create_subscription<Proposal>(
      NegotiationProposalTopicName, qos,
      [&](const Proposal::UniquePtr msg)
      {
        this->receive_proposal(*msg);
      });

    proposal_pub = node.create_publisher<Proposal>(
      NegotiationProposalTopicName, qos);

    rejection_sub = node.create_subscription<Rejection>(
      NegotiationRejectionTopicName, qos,
      [&](const Rejection::UniquePtr msg)
      {
        this->receive_rejection(*msg);
      });

    rejection_pub = node.create_publisher<Rejection>(
      NegotiationRejectionTopicName, qos);

    forfeit_sub = node.create_subscription<Forfeit>(
      NegotiationForfeitTopicName, qos,
      [&](const Forfeit::UniquePtr msg)
      {
        this->receive_forfeit(*msg);
      });

    forfeit_pub = node.create_publisher<Forfeit>(
      NegotiationForfeitTopicName, qos);

    conclusion_sub = node.create_subscription<Conclusion>(
      NegotiationConclusionTopicName, qos,
      [&](const Conclusion::UniquePtr msg)
      {
        this->receive_conclusion(*msg);
      });

    ack_pub = node.create_publisher<Ack>(
      NegotiationAckTopicName, qos);
  }

  void receive_repeat_request(const Repeat& msg)
  {
    if (subscription_callbacks->repeat)
    {
      subscription_callbacks->repeat(convert(msg));
    }
  }

  void receive_notice(const Notice& msg)
  {
    if (subscription_callbacks->notice)
    {
      subscription_callbacks->notice(convert(msg));
    }
  }

  void receive_proposal(const Proposal& msg)
  {
    if (subscription_callbacks->proposal)
    {
      subscription_callbacks->proposal(convert(msg));
    }
  }

  void receive_rejection(const Rejection& msg)
  {
    if (subscription_callbacks->rejection)
    {
      subscription_callbacks->rejection(convert(msg));
    }
  }

  void receive_forfeit(const Forfeit& msg)
  {
    if (subscription_callbacks->forfeit)
    {
      subscription_callbacks->forfeit(convert(msg));
    }
  }

  void receive_conclusion(const Conclusion& msg)
  {
    if (subscription_callbacks->conclusion)
    {
      subscription_callbacks->conclusion(convert(msg));
    }
  }

  void send_ack(const rmf_traffic::schedule::NegotiationRoom::Ack& ack)
  {
    ack_pub->publish(convert(ack));
  }

  void send_refusal(
    const rmf_traffic::schedule::NegotiationRoom::Refusal& refusal)
  {
    refusal_pub->publish(convert(refusal));
  }

  void send_proposal(
    const rmf_traffic::schedule::NegotiationRoom::Proposal& proposal)
  {
    proposal_pub->publish(convert(proposal));
  }

  void send_rejection(
    const rmf_traffic::schedule::NegotiationRoom::Rejection& rejection)
  {
    rejection_pub->publish(convert(rejection));
  }

  void send_forfeit(
    const rmf_traffic::schedule::NegotiationRoom::Forfeit& forfeit)
  {
    forfeit_pub->publish(convert(forfeit));
  }

  std::shared_ptr<void> register_negotiator(
    const rmf_traffic::schedule::ParticipantId for_participant,
    rmf_traffic::schedule::NegotiationRoomInternal::NegotiatorPtr negotiator)
  {
    return negotiation_room.register_negotiator(for_participant,
        std::move(negotiator));
  }

  void set_retained_history_count(uint count)
  {
    negotiation_room.set_retained_history_count(count);
  }

  TableViewPtr table_view(
    uint64_t conflict_version,
    const std::vector<rmf_traffic::schedule::ParticipantId>& sequence) const
  {
    return negotiation_room.table_view(conflict_version, sequence);
  }

  void on_status_update(StatusUpdateCallback cb)
  {
    negotiation_room.on_status_update(cb);
  }

  void on_conclusion(StatusConclusionCallback cb)
  {
    negotiation_room.on_conclusion(cb);
  }
};

//==============================================================================
Negotiation::Negotiation(
  rclcpp::Node& node,
  std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer,
  std::shared_ptr<rmf_traffic::schedule::NegotiationRoom::Worker> worker)
: _pimpl(rmf_utils::make_unique_impl<Implementation>(
      node, std::move(viewer), std::move(worker)))
{
  // Do nothing
}

//==============================================================================
void Negotiation::on_status_update(StatusUpdateCallback cb)
{
  _pimpl->on_status_update(cb);
}

//==============================================================================
void Negotiation::on_conclusion(StatusConclusionCallback cb)
{
  _pimpl->on_conclusion(cb);
}

//==============================================================================
Negotiation& Negotiation::timeout_duration(rmf_traffic::Duration duration)
{
  _pimpl->timeout = duration;
  return *this;
}

//==============================================================================
rmf_traffic::Duration Negotiation::timeout_duration() const
{
  return _pimpl->timeout;
}

//==============================================================================
Negotiation::TableViewPtr Negotiation::table_view(
  uint64_t conflict_version,
  const std::vector<rmf_traffic::schedule::ParticipantId>& sequence) const
{
  return _pimpl->table_view(conflict_version, sequence);
}

//==============================================================================
std::shared_ptr<void> Negotiation::register_negotiator(
  rmf_traffic::schedule::ParticipantId for_participant,
  std::unique_ptr<rmf_traffic::schedule::Negotiator> negotiator)
{
  return _pimpl->register_negotiator(for_participant, std::move(negotiator));
}

} // namespace schedule

rmf_traffic_msgs::msg::NegotiationForfeit convert(
  const rmf_traffic::schedule::NegotiationRoom::Forfeit& from)
{
  rmf_traffic_msgs::msg::NegotiationForfeit output;
  output.conflict_version = from.conflict_version;
  output.table = convert(from.table);
  return output;
}

rmf_traffic_msgs::msg::NegotiationKey convert(
  const rmf_traffic::schedule::NegotiationRoom::Key& from)
{
  rmf_traffic_msgs::msg::NegotiationKey output;
  output.participant = from.participant;
  output.version = from.version;
  return output;
}

rmf_traffic_msgs::msg::NegotiationRejection convert(
  const rmf_traffic::schedule::NegotiationRoom::Rejection& from)
{
  rmf_traffic_msgs::msg::NegotiationRejection output;
  output.table = convert(from.table);
  output.conflict_version = from.conflict_version;
  output.alternatives = convert(from.alternatives);
  output.rejected_by = from.rejected_by;
  return output;
}

rmf_traffic_msgs::msg::NegotiationProposal convert(
  const rmf_traffic::schedule::NegotiationRoom::Proposal& from)
{
  rmf_traffic_msgs::msg::NegotiationProposal output;
  output.to_accommodate = convert(from.to_accommodate);
  output.conflict_version = from.conflict_version;
  output.itinerary = convert(from.itinerary);
  output.for_participant = from.for_participant;
  output.proposal_version = from.proposal_version;
  return rmf_traffic_msgs::msg::NegotiationProposal();
}

std::vector<rmf_traffic_msgs::msg::NegotiationKey> convert(
  const std::vector<rmf_traffic::schedule::NegotiationRoom::Key>& from)
{
  std::vector<rmf_traffic_msgs::msg::NegotiationKey> output;
  output.reserve(from.size());
  for (const auto& key : from)
  {
    output.emplace_back(convert(key));
  }
  return output;
}

rmf_traffic::schedule::NegotiationRoom::Conclusion convert(
  const rmf_traffic_msgs::msg::NegotiationConclusion& from)
{
  rmf_traffic::schedule::NegotiationRoom::Conclusion output;
  output.table.reserve(from.table.size());
  for (const auto& key : from.table)
  {
    output.table.emplace_back(convert(key));
  }
  output.conflict_version = from.conflict_version;
  output.resolved = from.resolved;
  return output;
}

rmf_traffic::schedule::NegotiationRoom::Key convert(
  const rmf_traffic_msgs::msg::NegotiationKey& from)
{
  rmf_traffic::schedule::NegotiationRoom::Key output{};
  output.version = from.version;
  output.participant = from.participant;
  return output;
}

//==============================================================================
rmf_traffic::schedule::Negotiation::VersionedKeySequence convert(
  const std::vector<rmf_traffic_msgs::msg::NegotiationKey>& from)
{
  rmf_traffic::schedule::Negotiation::VersionedKeySequence output;
  output.reserve(from.size());
  for (const auto& key : from)
    output.push_back({key.participant, key.version});

  return output;
}

//==============================================================================
std::vector<rmf_traffic_msgs::msg::NegotiationKey> convert(
  const rmf_traffic::schedule::Negotiation::VersionedKeySequence& from)
{
  std::vector<rmf_traffic_msgs::msg::NegotiationKey> output;
  output.reserve(from.size());
  for (const auto& key : from)
  {
    rmf_traffic_msgs::msg::NegotiationKey msg;
    msg.participant = key.participant;
    msg.version = key.version;
    output.push_back(msg);
  }

  return output;
}

rmf_traffic_msgs::msg::NegotiationRefusal
convert(const rmf_traffic::schedule::NegotiationRoom::Refusal& from)
{
  rmf_traffic_msgs::msg::NegotiationRefusal output;
  output.conflict_version = from.conflict_version;
  return output;
}

rmf_traffic_msgs::msg::NegotiationAck
convert(const rmf_traffic::schedule::NegotiationRoom::Ack& from)
{
  rmf_traffic_msgs::msg::NegotiationAck output;
  output.conflict_version = from.conflict_version;
  output.acknowledgments.reserve(from.acknowledgments.size());
  for (const auto& acknowledgement : from.acknowledgments)
  {
    rmf_traffic_msgs::msg::NegotiationParticipantAck participant_ack;
    participant_ack.participant = acknowledgement.participant;
    participant_ack.itinerary_version = acknowledgement.itinerary_version;
    participant_ack.updating = acknowledgement.updating;
    output.acknowledgments.push_back(participant_ack);
  }
  return output;
}

rmf_traffic::schedule::NegotiationRoom::Repeat
convert(const rmf_traffic_msgs::msg::NegotiationRepeat& from)
{
  rmf_traffic::schedule::NegotiationRoom::Repeat output;
  output.conflict_version = from.conflict_version;
  output.table = from.table;
  return output;
}

rmf_traffic::schedule::NegotiationRoom::Notice
convert(const rmf_traffic_msgs::msg::NegotiationNotice& from)
{
  rmf_traffic::schedule::NegotiationRoom::Notice output;
  output.conflict_version = from.conflict_version;
  output.participants = from.participants;
  return output;
}

rmf_traffic::schedule::NegotiationRoom::Proposal
convert(const rmf_traffic_msgs::msg::NegotiationProposal& from)
{
  rmf_traffic::schedule::NegotiationRoom::Proposal output;
  output.conflict_version = from.conflict_version;
  output.itinerary = convert(from.itinerary);
  output.proposal_version = from.proposal_version;
  output.for_participant = from.for_participant;
  output.to_accommodate.reserve(from.to_accommodate.size());
  for (const auto& to_accomodate : from.to_accommodate)
  {
    output.to_accommodate.emplace_back(convert(to_accomodate));
  }
  return output;
}

rmf_traffic::schedule::NegotiationRoom::Rejection
convert(const rmf_traffic_msgs::msg::NegotiationRejection& from)
{
  rmf_traffic::schedule::NegotiationRoom::Rejection output;
  output.conflict_version = from.conflict_version;
  output.table.reserve(from.table.size());
  for (const auto& key : from.table)
  {
    output.table.emplace_back(convert(key));
  }
  output.rejected_by = from.rejected_by;
  output.alternatives = convert(from.alternatives);
  return output;
}

rmf_traffic::schedule::NegotiationRoom::Forfeit
convert(const rmf_traffic_msgs::msg::NegotiationForfeit& from)
{
  rmf_traffic::schedule::NegotiationRoom::Forfeit output;
  output.table.reserve(from.table.size());
  for (const auto& key : from.table)
  {
    output.table.emplace_back(convert(key));
  }
  output.conflict_version = from.conflict_version;
  return output;
}
} // namespace rmf_traffic_ros2
