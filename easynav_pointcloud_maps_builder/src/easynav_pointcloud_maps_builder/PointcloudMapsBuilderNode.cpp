// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Implementation of the PointcloudMapsBuilderNode class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_pointcloud_maps_builder/PointcloudMapsBuilderNode.hpp"
#include "easynav_common/types/PointPerception.hpp"

namespace easynav
{

PointcloudMapsBuilderNode::PointcloudMapsBuilderNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pointcloud_maps_builder_node", options)
{
  cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (!has_parameter("downsample_resolution")) {
    declare_parameter("downsample_resolution", 1.0);
  }
  if (!has_parameter("sensors")) {
    declare_parameter("sensors", std::vector<std::string>());
  }

  if (!has_parameter("perception_default_frame")) {
    declare_parameter("perception_default_frame", "map");
  }

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "map_builder_pointcloud/cloud_filtered", rclcpp::QoS(1).transient_local().reliable());

  register_handler(std::make_shared<PointPerceptionHandler>());
}

PointcloudMapsBuilderNode::~PointcloudMapsBuilderNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
PointcloudMapsBuilderNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  std::vector<std::string> sensors;
  get_parameter("sensors", sensors);
  get_parameter("downsample_resolution", downsample_resolution_);
  get_parameter("perception_default_frame", perception_default_frame_);

  for (const auto & sensor_id : sensors) {
    std::string topic, msg_type, group;

    if (!has_parameter(sensor_id + ".topic")) {
      declare_parameter(sensor_id + ".topic", topic);
    }
    if (!has_parameter(sensor_id + ".type")) {
      declare_parameter(sensor_id + ".type", msg_type);
    }
    if (!has_parameter(sensor_id + ".group")) {
      declare_parameter(sensor_id + ".group", group);
    }

    get_parameter(sensor_id + ".topic", topic);
    get_parameter(sensor_id + ".type", msg_type);
    get_parameter(sensor_id + ".group", group);

    RCLCPP_DEBUG(get_logger(),
                  "Loaded sensor parameters: id=%s topic=%s type=%s group=%s",
                  sensor_id.c_str(), topic.c_str(), msg_type.c_str(), group.c_str());

    auto handler_it = handlers_.find(group);
    if (handler_it == handlers_.end()) {
      RCLCPP_WARN(get_logger(), "No handler for group [%s]", group.c_str());
      continue;
    }

    auto ptr = handler_it->second->create(sensor_id);
    auto sub = handler_it->second->create_subscription(*this, topic, msg_type, ptr, cbg_);

    perceptions_[group].emplace_back(PerceptionPtr{ptr, sub});

    RCLCPP_DEBUG(get_logger(), "Creating perception for sensor %s", sensor_id.c_str());
    RCLCPP_DEBUG(get_logger(), "Handler group = %s", group.c_str());
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PointcloudMapsBuilderNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_activate();
  return CallbackReturnT::SUCCESS;
}
CallbackReturnT
PointcloudMapsBuilderNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_deactivate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PointcloudMapsBuilderNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_.reset();

  return CallbackReturnT::SUCCESS;
}

void PointcloudMapsBuilderNode::cycle()
{
    // Finish cycle if no new perceptions
  if (std::none_of(perceptions_["points"].begin(), perceptions_["points"].end(),
    [](const auto & p)
    {return p.perception->new_data;}))
  {
    return;
  }

  if (pub_->get_subscription_count() > 0) {
    auto point_perceptions = get_point_perceptions(perceptions_["points"]);
    auto processed_perceptions = PointPerceptionsOpsView(point_perceptions);
      // Fuse perceptions if the frame_id is different from default and downsample
    if (!point_perceptions.empty() && point_perceptions[0] &&
      point_perceptions[0]->frame_id != perception_default_frame_)
    {
      processed_perceptions.downsample(downsample_resolution_).fuse(perception_default_frame_);
    } else {
      processed_perceptions.downsample(downsample_resolution_);
    }

    auto downsampled_points = processed_perceptions.as_points();
    if (downsampled_points.empty()) {
      return;
    }

    auto msg = points_to_rosmsg(downsampled_points);
    msg.header.frame_id = point_perceptions[0]->frame_id;

    if (point_perceptions[0]->stamp.nanoseconds() != 0) {
      msg.header.stamp = point_perceptions[0]->stamp;
    } else {
      msg.header.stamp = now();
    }

    pub_->publish(msg);

      // Mark perceptions as not new after published
    for (auto & p : perceptions_["points"]) {
      if (p.perception->new_data) {
        p.perception->new_data = false;
      }
    }
  }
}

void
PointcloudMapsBuilderNode::register_handler(std::shared_ptr<PerceptionHandler> handler)
{
  handlers_[handler->group()] = handler;
}

}  // namespace easynav
