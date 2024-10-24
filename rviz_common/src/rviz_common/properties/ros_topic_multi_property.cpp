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


#include <map>
#include <string>
#include <algorithm>

#include <QApplication>  // NOLINT: cpplint can't handle Qt imports

#include "rviz_common/properties/ros_topic_multi_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace rviz_common
{
namespace properties
{

RosTopicMultiProperty::RosTopicMultiProperty(
  const QString & name,
  const QString & default_value,
  const std::vector<QString> & message_types,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: RosTopicProperty(name, default_value, "", description, parent, changed_slot, receiver),
  message_types_(message_types)
{}

void RosTopicMultiProperty::setMessageTypes(const std::vector<QString> & message_types)
{
  message_types_ = message_types;
}

void RosTopicMultiProperty::fillTopicList()
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  clearOptions();

  std::map<std::string, std::vector<std::string>> published_topics =
    rviz_ros_node_.lock()->get_topic_names_and_types();

  for (const auto & topic : published_topics) {
    // Only add topics whose type matches.
    for (const auto & type : topic.second) {
      if (std::find(message_types_.begin(), message_types_.end(), QString::fromStdString(type))
      != message_types_.end())
        addOptionStd(topic.first);
    }
  }
  sortOptions();
  QApplication::restoreOverrideCursor();
}

}  // end namespace properties
}  // end namespace rviz_common
