// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      robot.h
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#pragma once


#include <tuple>
#include <string>
#include <vector>
#include <random>
#include <set>
#include <random>


#include <glm/mat4x4.hpp> // mat4
#include <toml++/toml.hpp>

#include "ikparams.h"
#include "joint.h"
#include "link.h"
#include "poses.h"


using ParentJointChild = std::tuple<std::string, std::string, std::string>;

class Robot {

 private:
  std::string name_;
  std::string root_path_;
  std::vector<Link> links_;
  std::vector<Joint > joints_;

  Joint *p_root_joint_;
  Link *p_root_link_;
  Link *p_target_link_;
  size_t degrees_of_freedom_;

  std::vector<size_t> nonfixed_joint_indexes_;

  glm::mat4 root_transform_;
  glm::mat4 target_transform_;
  std::set<ParentJointChild> joint_link_chains_;

  Joint& find_joint_from_name(const std::string& joint_name);
  Link& find_link_from_name(const std::string& link_name);

  void find_root_link();
  void validate();
  void create_links(toml::table& top_level_data);
  void create_one_link(toml::table& link_table);
  void create_joints(toml::table& top_level_data);
  void create_one_joint(toml::table& joint_table);
  void add_link(const Link& link);
  void add_joint(Joint& ajoint, const std::string& parent_name, const std::string& child_name);

  std::vector<float> saved_angles_;
  std::vector<glm::mat4> saved_transforms_;
  float cyclic_coordinate_descent_single_joint(Joint& joint, float cyclic_coordinate_descent_cost);

  void save_config();
  void restore_config();
  void propagate_transform(Joint &joint);

  Joint& get_ith_nonfixed_joint(size_t idx);

 public:

  size_t degrees_of_freedom() const {
    return degrees_of_freedom_;
  }

  Robot(const std::string& name, const std::string &root_path, toml::table& top_level_data);
  const std::string& name() const { return name_; }
  void info() const;

  bool forward_kinematic(const std::vector<float>& angles);
  bool inverse_kinematic(IKParams& ik_params);

  void generate_blender_script(const std::string& filename) const;
  Pose get_current_pose() const;
  void init_from_pose(const Pose& pose);

};
