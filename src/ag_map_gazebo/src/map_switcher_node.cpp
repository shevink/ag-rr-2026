#include "ag_map_gazebo/map_switcher_node.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace
{
const std::unordered_map<std::string, ag_map_gazebo::PlaneScale> & mapScales()
{
  static const std::unordered_map<std::string, ag_map_gazebo::PlaneScale> kScales = {
    {"easy_map",   {8, 5}},
    {"medium_map", {8, 6}},
    {"hard_map",   {10, 10}},
    {"maze_map",   {8, 8}},
  };
  return kScales;
}
}  // namespace

namespace ag_map_gazebo
{

MapSwitcherNode::MapSwitcherNode()
: rclcpp::Node("map_switcher"),
  world_name_("empty"),
  model_name_("ground_plane")
{
  auto pkg_share = ament_index_cpp::get_package_share_directory("ag_map_gazebo");
  textures_dir_ = pkg_share + "/textures/";

  service_ = this->create_service<ag_map_gazebo::srv::LoadMap>(
    "load_map",
    std::bind(&MapSwitcherNode::handleLoadMap, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "MapSwitcherNode started. Waiting for service calls.");
}

bool MapSwitcherNode::removeEntity(
  const std::string & world, const std::string & name)
{
  gz::msgs::Entity req;
  req.set_name(name);
  req.set_type(gz::msgs::Entity::MODEL);

  gz::msgs::Boolean res;
  bool result;
  std::string service = "/world/" + world + "/remove";

  if (!gz_node_.Request(service, req, 5000, res, result)) {
    RCLCPP_WARN(this->get_logger(), "Failed to remove '%s' from world '%s'",
      name.c_str(), world.c_str());
    return false;
  }
  return res.data();
}

bool MapSwitcherNode::spawnGroundPlane(
  const std::string & world, const std::string & map_name)
{
  std::string texture_path = getTexturePath(map_name);
  if (texture_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Unknown map: '%s'", map_name.c_str());
    return false;
  }

  gz::msgs::EntityFactory req;
  req.set_sdf(buildGroundPlaneSdf(texture_path, getMapScale(map_name)));
  req.set_name(model_name_);
  req.set_allow_renaming(false);

  gz::msgs::Boolean res;
  bool result;
  std::string service = "/world/" + world + "/create";

  if (!gz_node_.Request(service, req, 5000, res, result) || !res.data()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to spawn ground plane");
    return false;
  }

  return true;
}

std::string MapSwitcherNode::getTexturePath(const std::string & map_name) const
{
  if (mapScales().count(map_name)) {
    return textures_dir_ + map_name + ".png";
  }
  return "";
}

ag_map_gazebo::PlaneScale MapSwitcherNode::getMapScale(const std::string & map_name) const
{
  return mapScales().at(map_name);
}

std::string MapSwitcherNode::buildGroundPlaneSdf(
  const std::string & texture_path, const PlaneScale & scale) const
{
  std::ostringstream sdf;
  sdf << "<?xml version='1.0'?>"
      << "<sdf version='1.7'>"
      << "<model name='" << model_name_ << "'>"
      << "  <static>true</static>"
      << "  <link name='link'>"
      << "    <collision name='collision'>"
      << "      <geometry>"
      << "        <plane><normal>0 0 1</normal></plane>"
      << "      </geometry>"
      << "    </collision>"
      << "    <visual name='visual'>"
      << "      <pose>" << scale.width / 2.0 << " " << scale.height / 2.0 << " 0 0 0 0</pose>"
      << "      <geometry>"
      << "        <plane><normal>0 0 1</normal>"
      << "          <size>" << scale.width << " "
      <<               scale.height << "</size>"
      << "        </plane>"
      << "      </geometry>"
      << "      <material>"
      << "        <diffuse>1 1 1 1</diffuse>"
      << "        <pbr>"
      << "          <metal>"
      << "            <albedo_map>" << texture_path << "</albedo_map>"
      << "            <metalness>0.0</metalness>"
      << "            <roughness>1.0</roughness>"
      << "          </metal>"
      << "        </pbr>"
      << "      </material>"
      << "    </visual>"
      << "  </link>"
      << "</model>"
      << "</sdf>";
  return sdf.str();
}


void MapSwitcherNode::handleLoadMap(
  const std::shared_ptr<ag_map_gazebo::srv::LoadMap::Request> request,
  std::shared_ptr<ag_map_gazebo::srv::LoadMap::Response> response)
{
  const auto & map_name = request->map_name;

  if (getTexturePath(map_name).empty()) {
    std::vector<std::string> keys;
    for (const auto & kv : mapScales()) {
      keys.push_back(kv.first);
    }
    std::sort(keys.begin(), keys.end());
    std::string valid_list;
    for (std::size_t i = 0; i < keys.size(); ++i) {
      if (i > 0) {valid_list += ", ";}
      valid_list += keys[i];
    }
    response->success = false;
    response->message = "Unknown map: '" + map_name +
      "'. Valid maps: " + valid_list;
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  // Remove the existing ground plane — retry to ensure Gazebo fully
  // processes the removal before spawning a replacement at z=0.
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (removeEntity(world_name_, model_name_)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  // Give rendering time to flush before spawning the replacement
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Spawn new ground plane with requested texture (retry once on failure)
  for (int attempt = 0; attempt < 2; ++attempt) {
    if (spawnGroundPlane(world_name_, map_name)) {
      response->success = true;
      response->message = "Loaded map '" + map_name + "'";
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  response->success = false;
  response->message = "Failed to load map '" + map_name + "' after retry";
  RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
}

}  // namespace ag_map_gazebo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ag_map_gazebo::MapSwitcherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
