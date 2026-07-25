#include <yaml-cpp/yaml.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
template <typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}
class ParamsLoad
{
public:
  ParamsLoad(const std::string & config_file)
  {
    YAML::Node config = YAML::LoadFile(config_file);
    std::string map_path = yaml_get_value<std::string>(config, "map_path");
    resolution = yaml_get_value<double>(config, "resolution");
    map_origin = yaml_get_value<std::vector<double>>(config, "map_origin");
  }

  double global_map_size_x, global_map_size_y;
  double resolution;
  std::vector<double> map_origin;
};