// Copyright (c) 2012, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RVIZ_COMMON__PROPERTIES__ROS_TOPIC_MULTI_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__ROS_TOPIC_MULTI_PROPERTY_HPP_

#include <string>
#include <unordered_set>

#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

class RVIZ_COMMON_PUBLIC RosTopicMultiProperty : public RosTopicProperty
{
  Q_OBJECT
public:

  void setMessageTypes(const std::unordered_set<QString> & message_types);

  std::unordered_set<QString> getMessageTypes() const
  {return message_types_;}

protected Q_SLOTS:
  virtual void fillTopicList() override;

private:
  // hide the parent class methods which only take a single type
  using RosTopicProperty::setMessageType;
  using RosTopicProperty::getMessageType;

  std::unordered_set<QString> message_types_; // TODO is there a QT-friendly type?
};

}  // end namespace properties
}  // end namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__ROS_TOPIC_MULTI_PROPERTY_HPP_
