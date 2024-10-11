// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2017, Bosch Software Innovations GmbH.
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


#include <memory>
#include <string>
#include <utility>
#include <algorithm>
#include <vector>

#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include "sensor_msgs/image_encodings.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/properties/ros_topic_multi_property.hpp"
#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"
#include "rviz_default_plugins/displays/image/image_display.hpp"
#include "rviz_default_plugins/displays/image/ros_image_texture_iface.hpp"
#include "rviz_default_plugins/displays/image/get_transport_from_topic.hpp"

#include <rviz_common/logging.hpp> // TODO remove when done

// TODO This may have introduced a segfault upon closing the program, but it is infrequent.
// It's possible this existed before my changes.

namespace rviz_default_plugins
{
namespace displays
{

ImageDisplay::ImageDisplay()
: ImageDisplay(std::make_unique<ROSImageTexture>()) {}

ImageDisplay::ImageDisplay(std::unique_ptr<ROSImageTextureIface> texture)
: texture_(std::move(texture))
{
  delete this->topic_property_;
  this->topic_property_ = new rviz_common::properties::RosTopicMultiProperty(
    "Topic",
    "",
    std::vector<QString>(),
    "Image transport topic to subscribe to.",
    this,
    SLOT(updateTopic()));

  delete this->qos_profile_property_;
  this->qos_profile_property_ = new rviz_common::properties::QosProfileProperty(this->topic_property_, rclcpp::QoS(5));

  normalize_property_ = new rviz_common::properties::BoolProperty(
    "Normalize Range",
    true,
    "If set to true, will try to estimate the range of possible values from the received images.",
    this,
    SLOT(updateNormalizeOptions()));

  min_property_ = new rviz_common::properties::FloatProperty(
    "Min Value",
    0.0,
    "Value which will be displayed as black.",
    this,
    SLOT(updateNormalizeOptions()));

  max_property_ = new rviz_common::properties::FloatProperty(
    "Max Value",
    1.0,
    "Value which will be displayed as white.",
    this,
    SLOT(updateNormalizeOptions()));

  median_buffer_size_property_ = new rviz_common::properties::IntProperty(
    "Median window",
    5,
    "Window size for median filter used for computing min/max.",
    this,
    SLOT(updateNormalizeOptions()));

  got_float_image_ = false;
}

void ImageDisplay::onInitialize()
{
  ITDClass::onInitialize();

  updateNormalizeOptions();
  setupScreenRectangle();

  setupRenderPanel();

  render_panel_->getRenderWindow()->setupSceneAfterInit(
    [this](Ogre::SceneNode * scene_node) {
      scene_node->attachObject(screen_rect_.get());
    });
}

ImageDisplay::~ImageDisplay() = default;

void ImageDisplay::onEnable()
{
  subscribe();
}

void ImageDisplay::onDisable()
{
  unsubscribe();
  clear();
}

// Need a signature with pass by reference for image_transport_.subscribe
void ImageDisplay::incomingMessage(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg){
  ImageTransportDisplay<sensor_msgs::msg::Image>::incomingMessage(img_msg);
}

void ImageDisplay::subscribe(){
  if (!isEnabled()) {
    return;
  }

  // TODO this should really be in onInitialize but setStatus does not work there
  // Populate topic message types based on installed image_transport plugins
  image_transport::ImageTransport image_transport_(rviz_ros_node_.lock()->get_raw_node());
  std::vector<std::string> transports = image_transport_.getLoadableTransports();
  std::vector<QString> message_types;
  // Map to message types
  const std::unordered_map<std::string, std::string> transport_message_types_ =
  {
      {"raw",             "sensor_msgs/msg/Image"},
      {"compressed",      "sensor_msgs/msg/CompressedImage"},
      {"compressedDepth", "sensor_msgs/msg/CompressedImage"},
      {"theora",          "theora_image_transport/msg/Packet"},
      {"zstd",            "sensor_msgs/msg/CompressedImage"},
  };
  std::string transports_str = "";
  rviz_common::properties::StatusProperty::Level transports_status_level =
    rviz_common::properties::StatusProperty::Ok;
  for (std::string & transport : transports){
    transport = transport.substr(transport.find_last_of('/') + 1);
    try{
      message_types.push_back(QString::fromStdString(transport_message_types_.at(transport)));
      transports_str += transport + ", ";
    }
    catch (const std::out_of_range & e){
      transports_status_level = rviz_common::properties::StatusProperty::Warn;
      transports_str += "(unknown: " + transport + "), ";
    }
  }
  setStatusStd(
      transports_status_level,
      "Image Transports",
      transports_str);
  // Remove duplicates
  message_types.erase(std::unique(message_types.begin(), message_types.end() ), message_types.end());
  // Update the message types to allow in the topic_property_
  ((rviz_common::properties::RosTopicMultiProperty*)topic_property_)->setMessageTypes(message_types);

  if (topic_property_->isEmpty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      QString("Error subscribing: Empty topic name"));
    return;
  }

  try {
    rclcpp::Node::SharedPtr node = rviz_ros_node_.lock()->get_raw_node();
    image_transport::ImageTransport image_transport_(node);
    // This part differs from the parent class. ImageTransportDisplay uses an
    // image_transport::SubscriberFilter, which requires a different callback for each transport
    // type. image_transport::Subscriber only requires one callback for "raw" and the other types
    // are automatically converted.
    subscription_ = image_transport_.subscribe(
      rviz_default_plugins::displays::getBaseTopicFromTopic(topic_property_->getTopicStd()),
      (uint32_t)qos_profile.get_rmw_qos_profile().depth, // TODO try without cast
      &ImageDisplay::incomingMessage,
      this,
      new image_transport::TransportHints(
        node.get(),
        getTransportFromTopic(topic_property_->getStdString()),
        "image_transport"));

    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      QString("Error subscribing: ") + e.what());
  }
}

void ImageDisplay::unsubscribe(){
  subscription_.shutdown();
}

void ImageDisplay::updateNormalizeOptions()
{
  if (got_float_image_) {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_->setNormalizeFloatImage(
      normalize, min_property_->getFloat(), max_property_->getFloat());
    texture_->setMedianFrames(median_buffer_size_property_->getInt());
  } else {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void ImageDisplay::clear()
{
  texture_->clear();
}

void ImageDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
  try {
    texture_->update();

    // make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_->getWidth();
    float img_height = texture_->getHeight();

    if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0) {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if (img_aspect > win_aspect) {
        screen_rect_->setCorners(
          -1.0f, 1.0f * win_aspect / img_aspect, 1.0f, -1.0f * win_aspect / img_aspect, false);
      } else {
        screen_rect_->setCorners(
          -1.0f * img_aspect / win_aspect, 1.0f, 1.0f * img_aspect / win_aspect, -1.0f, false);
      }
    }
  } catch (UnsupportedImageEncoding & e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Image", e.what());
  }
}

void ImageDisplay::reset()
{
  ITDClass::reset();
  clear();
}

/* This is called by incomingMessage(). */
void ImageDisplay::processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  bool got_float_image = msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
    msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
    msg->encoding == sensor_msgs::image_encodings::MONO16;

  if (got_float_image != got_float_image_) {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }
  texture_->addMessage(msg);
}

void ImageDisplay::setupScreenRectangle()
{
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "ImageDisplayObject" << count++;

  screen_rect_ = std::make_unique<Ogre::Rectangle2D>(true);
  screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

  ss << "Material";
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(ss.str());
  material_->setSceneBlending(Ogre::SBT_REPLACE);
  material_->setDepthWriteEnabled(false);
  material_->setDepthCheckEnabled(false);

  Ogre::TextureUnitState * tu =
    material_->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureName(texture_->getName());
  tu->setTextureFiltering(Ogre::TFO_NONE);
  tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

  material_->setCullingMode(Ogre::CULL_NONE);
  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  screen_rect_->setBoundingBox(aabInf);
  screen_rect_->setMaterial(material_);
}

void ImageDisplay::setupRenderPanel()
{
  render_panel_ = std::make_unique<rviz_common::RenderPanel>();
  render_panel_->resize(640, 480);
  render_panel_->initialize(context_);
  setAssociatedWidget(render_panel_.get());

  static int count = 0;
  render_panel_->getRenderWindow()->setObjectName(
    "ImageDisplayRenderWindow" + QString::number(count++));
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::ImageDisplay, rviz_common::Display)
