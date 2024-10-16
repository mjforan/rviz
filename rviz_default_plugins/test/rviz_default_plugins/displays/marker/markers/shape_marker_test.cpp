// Copyright (c) 2018, Bosch Software Innovations GmbH.
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


#include <gmock/gmock.h>

#include <memory>
#include <string>

#include <OgreEntity.h>
#include <OgreMesh.h>

#include "visualization_msgs/msg/marker.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include "rviz_default_plugins/displays/marker/markers/shape_marker.hpp"

#include "../../../scene_graph_introspection.hpp"
#include "markers_test_fixture.hpp"
#include "../marker_messages.hpp"

using namespace ::testing;  // NOLINT

TEST_F(MarkersTestFixture, no_transform_does_not_try_to_set_scene_node) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ShapeMarker>();

  EXPECT_CALL(*frame_manager_, transform(_, _, _, _, _)).WillOnce(Return(false));  // NOLINT

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::CUBE));

  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), "rviz_cube.mesh");
  ASSERT_FALSE(entity->isVisible());
}

TEST_F(MarkersTestFixture, positions_and_orientations_are_set_correctly) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ShapeMarker>();

  Ogre::Vector3 position(0, 1, 0);
  Ogre::Quaternion orientation(0, 0, 1, 0);
  mockValidTransform(position, orientation);

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::CUBE));

  EXPECT_THAT(marker_->getPosition(), Vector3Eq(position));
  EXPECT_THAT(marker_->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 0.7071f, -0.7071f)));

  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), "rviz_cube.mesh");

  auto shape_scene_node = entity
    ->getParentSceneNode()  // OffsetNode
    ->getParentSceneNode();  // SceneNode

  EXPECT_THAT(shape_scene_node->getPosition(), Vector3Eq(Ogre::Vector3::ZERO));
  EXPECT_THAT(shape_scene_node->getScale(), Vector3Eq(Ogre::Vector3(1, -0.2f, 0.2f)));
}

TEST_F(MarkersTestFixture, different_types_result_in_different_meshes) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ShapeMarker>();
  mockValidTransform();

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::SPHERE));

  auto cylinder_entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), "rviz_cylinder.mesh");
  auto sphere_entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), "rviz_sphere.mesh");
  EXPECT_FALSE(cylinder_entity);
  EXPECT_TRUE(sphere_entity);

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::CYLINDER));

  cylinder_entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), "rviz_cylinder.mesh");
  sphere_entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), "rviz_sphere.mesh");
  EXPECT_TRUE(cylinder_entity);
  EXPECT_FALSE(sphere_entity);
}
