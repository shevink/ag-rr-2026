// Copyright 2024 AG SOSE 2026
// SPDX-License-Identifier: MIT

#ifndef AG_MAP_GAZEBO__MAP_SWITCHER_NODE_HPP_
#define AG_MAP_GAZEBO__MAP_SWITCHER_NODE_HPP_

#include <memory>
#include <string>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

#include <ag_map_gazebo/srv/load_map.hpp>

namespace ag_map_gazebo
{

struct PlaneScale
{
  double width;
  double height;
};

class MapSwitcherNode : public rclcpp::Node
{
public:
  MapSwitcherNode();
  virtual ~MapSwitcherNode() = default;

private:
  void handleLoadMap(
    const std::shared_ptr<ag_map_gazebo::srv::LoadMap::Request> request,
    std::shared_ptr<ag_map_gazebo::srv::LoadMap::Response> response);

  bool removeEntity(const std::string & world, const std::string & name);
  bool spawnGroundPlane(const std::string & world, const std::string & map_name);
  std::string getTexturePath(const std::string & map_name) const;
  PlaneScale getMapScale(const std::string & map_name) const;
  std::string buildGroundPlaneSdf(const std::string & texture_path, const PlaneScale & scale) const;

  rclcpp::Service<ag_map_gazebo::srv::LoadMap>::SharedPtr service_;
  gz::transport::Node gz_node_;
  std::string world_name_;
  std::string model_name_;
  std::string textures_dir_;
};

}  // namespace ag_map_gazebo

#endif  // AG_MAP_GAZEBO__MAP_SWITCHER_NODE_HPP_
