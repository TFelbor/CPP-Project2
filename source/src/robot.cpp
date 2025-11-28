// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      robot.cpp
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#include <algorithm>
#include <glm/trigonometric.hpp>
#include <glm/mat4x4.hpp>
#include <map>
#include <memory>
#include <string>

#include <toml++/toml.hpp>
#include <vector>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include "context.h"
#include "joint.h"
#include "link.h"
#include "logger.h"
#include "stlmesh.h"
#include "robot.h"
#include "poses.h"
#include "glm_util.h"


// needed to parse a number in the toml file into a float
// example angle_min = 1.57
static float
get_float(const std::string& joint_name, toml::table& a_table, const std::string& a_key)
{
  const auto& toml_node = a_table[a_key];
  if (not toml_node) {
    throw robot::exception("Key '{}' is not found", a_key);
  }

  float v = 0.0f;
  v = *(toml_node.value<float>());
  return v;
}

// parse a vector of three floats
// example rpy = [-3.141594, 0, 0]
static
glm::vec3 get_vec3(const std::string& joint_name, toml::table& a_table, const std::string& a_key)
{
  glm::vec3 res3d;
  auto a_3d_array = a_table[a_key];

  size_t idx = 0;

  a_3d_array.as_array()->for_each([&res3d, &idx, &joint_name, &a_key](const toml::node& item) {
    float v =  *(item.value<float>());
    if (idx >= 3) {
      throw robot::exception(
          "Too many entries in vector '{}' for joint '{}'", a_key,
          joint_name);
    }
    res3d[idx] = v;
    ++idx;
  });

  if (idx != 3) {
    throw robot::exception(
        "Too few entries in vector '{}' for joint '{}'", a_key,
        joint_name);
  }
  return res3d;
}

//
// create a robot based on a config file
// [info]
// robot_name = "Kinova Gen3 6DF with vision"
// root_path =  "../meshes/6dof"
//

Robot::Robot(const std::string& name, const std::string &root_path, toml::table& top_level_data)
    : name_{name},
      root_path_{root_path},
      root_transform_{glm::mat4(1.0f)},
      target_transform_{glm::mat4(1.0f)}
{
  logger::info("Creating links for {}", name_);
  create_links(top_level_data);
  logger::info("Creating joints for {}", name_);
  create_joints(top_level_data);
  logger::info("Validating robot consistency");
  validate();

  // compute the degrees of freedom and
  // quick access to non fixed joints
  
  degrees_of_freedom_ = 0;
  for (size_t idx = 0; idx < joints_.size(); ++idx) {
    if (joints_[idx].has_degree_of_freedom()) {
      ++degrees_of_freedom_;
      joints_[idx].idx_nonfixed(nonfixed_joint_indexes_.size());
      nonfixed_joint_indexes_.push_back(idx);
    }
  }
}

void
Robot::create_one_link(toml::table& link_table)
{
  // list mandatory entries for each link
  std::map<std::string, toml::node_type> link_entries{
      {"name", toml::node_type::string},
      {"path", toml::node_type::string}
  };
  try{
    // validate the type
    for (const auto& an_entry : link_entries) {
      auto& [a_key, a_type] = an_entry;
      const auto& link_entry_node = link_table[a_key];
      if (not link_entry_node) {
          throw robot::exception("Missing '{0}' entry in table 'joints'", a_key);
      }
      if (a_type == toml::node_type::string and not link_entry_node.is_string()) {
        throw robot::exception("Entry '{}' is not of a string", a_key);
      }
    }

    // read the associated mesh and compute the bbox

    std::string link_name;
    link_name = *(link_table["name"].value<std::string>());

    std::string stl_file_name;
    stl_file_name = *(link_table["path"].value<std::string>());

    Link link(link_name, stl_file_name);

    std::string stl_filepath = root_path_ + "/" + stl_file_name;
    auto& mesh = StlMeshFactory::instance().create_from_stlfile(link_name, stl_filepath);
    auto& bbox = mesh.get_bbox();
    link.set_bbox(bbox);

    // take care of optional configuration
    // istarget = 1 (optional)

    link.is_target(false);
    const auto& link_istarget_node = link_table["istarget"];
    if (not link_istarget_node) {
      link.is_target(false);
    } else if(not link_istarget_node.is_number()) { 
      throw robot::exception("Entry 'istarget' must be a number");
    } else {
      int value;
      value = *(link_istarget_node.value<int>());
      logger::debug("Found target for  {} {}", link_name, value);
      link.is_target(value == 1);
    }

    //   isroot   = 1   (optional)
    link.is_root(false);
    const auto& link_isroot_node = link_table["isroot"];
    if (not link_isroot_node) {
      link.is_root(false);
    } else if(not link_isroot_node.is_number()) { 
      throw robot::exception("Entry 'isroot' must be a number");
    } else {
      int value;
      value = *(link_isroot_node.value<int>());
      logger::debug("Found root for  {} {}", link_name, value);
      link.is_root(value == 1);
    }

    add_link(link);
    mesh.info();

  } catch (const robot::exception& err) {
    logger::fatal(err.what());
    std::exit(1);
  }

}

void
Robot::create_links(toml::table& top_level_data)
{
  auto list_of_links = top_level_data["links"];

  // iterates over all the  [[links]]
  //
  list_of_links.as_array()->for_each([this](toml::table& item) {
    create_one_link(item);
  });
}

void
Robot::create_one_joint(toml::table& joint_table)
{
  // list mandatory entries for each joint
  std::map<std::string, toml::node_type> joint_entries{
      {"name", toml::node_type::string},
      {"type", toml::node_type::string},
      {"parent", toml::node_type::string},
      {"child", toml::node_type::string},
      {"xyz", toml::node_type::array},
      {"rpy", toml::node_type::array},
      {"axis", toml::node_type::array},
      {"angle_min", toml::node_type::floating_point},
      {"angle_max", toml::node_type::floating_point},
      {"angle_default", toml::node_type::floating_point}};

  try{
    // validate the type
    for (const auto& an_entry : joint_entries) {
      auto& [a_key, a_type] = an_entry;
      const auto& joint_entry_node = joint_table[a_key];
      if (not joint_entry_node) {
          throw robot::exception("Missing '{0}' entry in table 'joints'", a_key);
      }
      if (a_type == toml::node_type::string and not joint_entry_node.is_string()) {
        throw robot::exception("Entry '{}' is not of a string", a_key);
      }
      if (a_type == toml::node_type::array and not joint_entry_node.is_array()) {
        throw robot::exception("Entry '{}' is not of an array", a_key);
      }
      if (a_type == toml::node_type::floating_point and not joint_entry_node.is_number()) {
        throw robot::exception("Entry '{}' is not of a number", a_key);
      }
    }
    
    std::string joint_name;
    joint_name = *(joint_table["name"].value<std::string>());
    //std::cout << fmt::format("Joint name: '{}'\n", joint_name);

    Joint ajoint(joint_name);
    glm::vec3 translation3d = get_vec3(joint_name, joint_table, "xyz");
    glm::vec3 rpy3d = get_vec3(joint_name, joint_table, "rpy");

    ajoint.set_local(translation3d, rpy3d);

    std::string joint_type;
    joint_type = *(joint_table["type"].value<std::string>());
    float angle_min = get_float(joint_name, joint_table, "angle_min");
    float angle_max = get_float(joint_name, joint_table, "angle_max");

    ajoint.set_joint_type(joint_type, angle_min, angle_max);

    std::string parent_name;
    parent_name = *(joint_table["parent"].value<std::string>());

    std::string child_name;
    child_name = *(joint_table["child"].value<std::string>());

    add_joint(ajoint, parent_name, child_name);

  } catch (const robot::exception& err) {
    logger::fatal(err.what());
    std::exit(1);
  }
}

void
Robot::create_joints(toml::table& top_level_data)
{
  // create the joints
  //
  auto list_of_joints = top_level_data["joints"];


  // iterates over all the  [[joints]]
  //
  list_of_joints.as_array()->for_each([this](toml::table& item) {
    create_one_joint(item);
  });
}

//
//   <<<<LINK>>> <<JOINT>> <<<<LINK>>>>
//
Link&
Robot::find_link_from_name(const std::string& link_name)
{
  for (auto& a_link : links_) {
    if (a_link.name() == link_name) {
      return a_link;
    }
  }
  throw robot::exception("Cannot find link with name '{}'", link_name);
}

Joint&
Robot::find_joint_from_name(const std::string& joint_name)
{
  for (auto& a_joint : joints_) {
    if (a_joint.name() == joint_name) {
      return a_joint;
    }
  }
  throw robot::exception("Cannot find joint with name '{}'", joint_name);
}


void
Robot::add_joint(Joint& a_joint, const std::string& parent_name, const std::string& child_name)
{
  const std::string& joint_name = a_joint.name();

  bool named_is_used = std::ranges::any_of( joints_,
      [&joint_name](const Joint& ith_joint) -> bool {
        return ith_joint.name() == joint_name;
      });

  if (named_is_used) {
    throw robot::exception("Joint name '{}' already used, aborting.", joint_name);
  }

  // validate the parent and child link
  // have already been created

  [[maybe_unused]] Link& parent_link = find_link_from_name(parent_name);
  [[maybe_unused]] Link& child_link = find_link_from_name(child_name);

  // validation of the correctness of the joint
  // * cannot have a joint on an identical existing joint
  // * cannot have a joint creating a loop

  ParentJointChild pjc_item{parent_name, joint_name, child_name};
  if (joint_link_chains_.count(pjc_item) != 0) {
    throw robot::exception(
        "Already have a joint name '{}' between parent and child links",
        joint_name);
  }

  ParentJointChild pjc_item_inverse{child_name, joint_name, parent_name};
  if (joint_link_chains_.count(pjc_item_inverse) != 0) {
    throw robot::exception(
        "Already have a joint name '{}' between parent and child links",
        joint_name);
  }

  // validation of the correctness of the joint
  // * cannot have a joint with re-convergent parents
  for (auto& a_pjc_item : joint_link_chains_) {
    auto& child_name_used = std::get<2>(a_pjc_item);
    if (child_name_used == child_name) {
      throw robot::exception(
          "Joint  name '{}', child link '{}' is already used, aborting",
          joint_name, child_name);
    }

    auto& parent_name_used = std::get<0>(a_pjc_item);
    if (child_name_used == parent_name and parent_name_used == child_name) {
      throw robot::exception(
          "Joint  name '{}', parent link '{}' is already used, aborting",
          joint_name, child_name);
    }
  }
  joint_link_chains_.insert(pjc_item);
  joints_.push_back(a_joint);
}


void
Robot::add_link(const Link& a_link)
{
  const std::string& link_name = a_link.name();

  bool named_is_used = std::ranges::any_of(links_,
    [&link_name](const Link& ith_link) -> bool {
      return ith_link.name() == link_name;
    });

  if (named_is_used) {
    throw robot::exception("Link name '{}' already used, aborting", link_name);
  }

  links_.push_back(a_link);
}


//
// find_root_link()
// make sure that only 1 root link exists

void
Robot::find_root_link()
{
  std::map<std::string, int> link_name_counter;
  for (auto& a_pjc : joint_link_chains_) {
    auto& parent_name = std::get<0>(a_pjc);
    if (link_name_counter.count(parent_name) == 0) {
      link_name_counter.insert(std::make_pair(parent_name, 0));
    }
    auto& child_name = std::get<2>(a_pjc);
    if (link_name_counter.count(child_name) == 0) {
      link_name_counter.insert(std::make_pair(child_name, 1));
    } else  {
      ++link_name_counter[child_name];
    }
  }
  bool found_it = false;
  std::string root_link_name = "";
  for (const auto& [link_name, counter] : link_name_counter) {
    if (counter == 0) {
      if (found_it) {
        throw robot::exception("Multiple root links found with name '{}' and '{}'", root_link_name, link_name);
      }
      root_link_name = link_name;
      found_it = true;
    }
  }
  if (not found_it) {
    throw robot::exception("Cannot find root link");
  }
  p_root_link_ = std::addressof(find_link_from_name(root_link_name));
 }


//
// various sanity check on the robots
// and adding pointers for chain linking the Links and the Joints
//

void
Robot::validate()
{
  // step 1. find the root link
  find_root_link();

  // step 2. creates pointer links
  for (auto& a_pjc : joint_link_chains_) {
    Link* parent_link_ptr = std::addressof(find_link_from_name(  std::get<0>(a_pjc)));
    Joint* joint_ptr       = std::addressof(find_joint_from_name( std::get<1>(a_pjc)));
    Link* child_link_ptr  = std::addressof(find_link_from_name(  std::get<2>(a_pjc)));

    parent_link_ptr->add_child_joint(joint_ptr);
    child_link_ptr->add_parent_link(parent_link_ptr);
    joint_ptr->add_parent_link(parent_link_ptr);
    joint_ptr->add_child_link(child_link_ptr);
  }

  // step 3. set target link and verify only one target is set
  bool found_it = false;
  p_target_link_ = nullptr;
  for (auto& a_link : links_) {
    logger::debug("Checking if link {} is the target", a_link.name());
    if (a_link.is_target()) {
      logger::debug("... found it [1]");
      if (not found_it) {
        p_target_link_ = std::addressof(a_link);
        found_it = true;
        logger::debug("... found it [2]");
      } else {
        throw robot::exception("Multiple target found with name '{}' and '{}'",
            p_target_link_->name(), a_link.name());
      }
    }
  }
  if (not found_it) {
    throw robot::exception("Could not find a target link");
  }
}


void
Robot::info() const
{
  logger::info("+++++++++++++++++++++++++++++++++++++++");
  logger::info("Information for robot '{}'", name_);
  logger::info("Number of links     {}", links_.size());
  logger::info("Number of joints    {}", joints_.size());
  logger::info("Degrees of freedom  {}", degrees_of_freedom());
  logger::info("Root link           {}", p_root_link_->name());
  logger::info("Target link         {}", p_target_link_->name());
  logger::info("+++++++++++++++++++++++++++++++++++++++");
}


void
Robot::propagate_transform(Joint &joint)
{
  Link* plink = joint.parent_link();

  std::vector<Link*> plinks;
  plinks.push_back(plink);

  do {
    plink = plinks[0];
    plinks.erase(plinks.begin());

    plink->foreach_child_joint([&plinks](Joint* pjoint) {
      glm::mat4 local_transform = pjoint->get_local_transform();
      Link* p_child_link = pjoint->child_link();
      Link* p_parent_link = p_child_link->parent_link();

      const glm::mat4& parent_transform = p_parent_link->get_transform();
      p_child_link->set_transform(parent_transform * local_transform);
      plinks.push_back(p_child_link);
    });
  } while (plinks.size() != 0);

}



Pose
Robot::get_current_pose() const
{
  Pose pose;
  for(const auto& link: links_) {
    pose.add_link_transform(link.name(), link.get_transform(), link.is_root(), link.is_target());
  }

  for(const auto& joint: joints_) {
    pose.add_joint_angle(joint.name(), joint.get_angle());
  }

  return pose;
}

void
Robot::init_from_pose(const Pose& pose)
{
  for(auto& link: links_) {
    const glm::mat4& transform = pose.get_link_transform(link.name());
    link.set_transform(transform);
  }

  for(auto& joint: joints_) {
    const float angle = pose.get_joint_angle(joint.name());
    joint.set_angle(angle);
  }

}


void
Robot::save_config()
{
  saved_angles_.clear();
  for(const auto& joint: joints_) {
    if (joint.has_degree_of_freedom()) {
      saved_angles_.push_back(joint.get_angle());
    }
  }

  saved_transforms_.clear();
  for(auto& link: links_) {
    const glm::mat4& transform = link.get_transform();
    saved_transforms_.push_back(transform);
  }
}

void
Robot::restore_config()
{
  size_t idx;

  idx = 0;
  for(auto& joint: joints_) {
    if (joint.has_degree_of_freedom()) {
      joint.set_angle(saved_angles_[idx]);
      ++idx;
    }
  }

  idx = 0;
  for(auto& link: links_) {
    link.set_transform(saved_transforms_[idx]);
    ++idx;
  }
}

Joint&
Robot::get_ith_nonfixed_joint(size_t idx)
{
  assert(idx < nonfixed_joint_indexes_.size());
  size_t idx_nonfixed = nonfixed_joint_indexes_[idx];

  assert(idx == joints_[idx_nonfixed].idx_nonfixed());
  return joints_[idx_nonfixed];
}

//
// forward kinematic
//

bool
Robot::forward_kinematic(const std::vector<float>& angles)
{
  std::vector<float> angles_in_degrees;
  for(const auto& value: angles) {
    angles_in_degrees.push_back(glm::degrees(value));
  }


  logger::info("Compute forward kinematic with angles {:g}", fmt::join(angles_in_degrees, ","));
  
  logger::info("---- FK Student code start here");
  // code start

  if (angles.size() != nonfixed_joint_indexes_.size()) {
    logger::error("ERROR: Mismatch between provided angles: ({}) and degrees of freedom: ({})", 
                  angles.size(), nonfixed_joint_indexes_.size());
    return false;
  }

  // Step 1: We need to set the angles for all non-fixed joints
  for (size_t i = 0; i < angles.size(); ++i) {
    Joint& joint = get_ith_nonfixed_joint(i);
    joint.set_angle(angles[i]);
  }

  // Step 2: We need to propagate transforms from the root down to the leaves
  // We'll use a lambda to recursively update the global transforms
  std::function<void(Link*)> recursively_update_transforms = [&](Link* link) {
    link->foreach_child_joint([&](Joint* joint) {
      Link* child_link = joint->child_link();
      
      // The child's global transform = ParentGlobal * JointLocal
      glm::mat4 child_global = link->get_transform() * joint->get_local_transform();
      child_link->set_transform(child_global);
      
      // Use recursion to update the children of this link
      recursively_update_transforms(child_link);
    });
  };

  // Make sure the root starts with its base transform 
  p_root_link_->set_transform(root_transform_);
  recursively_update_transforms(p_root_link_);

  // code end
  logger::info("---- FK Student code finish here");


  const glm::mat4& target_transform = p_target_link_->get_transform();
  std::vector<float> xyz_rpy = glmutil::get_xyz_rpy(target_transform);
  logger::info("Target position (x, y, z) = ({}, {}, {})", xyz_rpy[0], xyz_rpy[1] , xyz_rpy[2]);
  logger::info("Target rotation (r, p, y) = ({}, {}, {})",
      glm::degrees(xyz_rpy[3]), 
      glm::degrees(xyz_rpy[4]),
      glm::degrees(xyz_rpy[5]));
  std::cout << "Target Transform = "; std::cout << target_transform << std::endl;

  return true ;
}

// Helper function to help optimize the process of a single joint in CCD
// Returns the distance to target after optimization
float 
Robot::CCD_single_joint(Joint& joint, float CCD_cost)
{
    float best_angle = joint.get_angle();
    float min_dist = CCD_cost;

    auto [min_a, max_a] = joint.get_min_max_angles();
    
    // Our sampling strategy: iteration over the valid range
    float step = glm::radians(2.0f); // 2 degree step size
    
    // Extract target position
    glm::vec3 target_pos = glm::vec3(target_transform_[3]);

    for (float angle = min_a; angle <= max_a; angle += step) {
        joint.set_angle(angle);
        propagate_transform(joint); // Update chain for this trial
        
        // Check distance
        const glm::mat4& ee_transform = p_target_link_->get_transform();
        glm::vec3 ee_pos = glm::vec3(ee_transform[3]);
        float dist = glm::distance(ee_pos, target_pos);
        
        if (dist < min_dist) {
            min_dist = dist;
            best_angle = angle;
        }
    }

    // Set joint to the best found angle
    joint.set_angle(best_angle);
    propagate_transform(joint);
    
    return min_dist;
}

//
// inverse kinematic
// the param is not const to allow iteration of the rnd generator
//
bool
Robot::inverse_kinematic(IKParams& ik_params)
{
  std::vector<float> rpy_in_degrees;
  for(const auto& value: ik_params.rpy_) {
    rpy_in_degrees.push_back(glm::degrees(value));
  }

  logger::info("Target position is ({:g})", fmt::join(ik_params.xyz_, ","));
  logger::info("Target rotation is ({:g})", fmt::join(rpy_in_degrees, ","));


  target_transform_ = glmutil::to_matrix4x4(ik_params.xyz_, ik_params.rpy_);
  float ccd_distance_to_target = std::numeric_limits<float>::max();
  


  logger::info("---- IK Student code start here");
  // student code start

  // Initialize the distance
  glm::vec3 target_pos = glm::vec3(target_transform_[3]);
  const glm::mat4& start_ee_transform = p_target_link_->get_transform();
  ccd_distance_to_target = glm::distance(glm::vec3(start_ee_transform[3]), target_pos);

  // CCD Loop
  for(size_t iter = 0; iter < ik_params.nb_iterations_; ++iter) {
      
      // Iterate joints from End-Effector to Root (in reverse order)
      for (auto it = nonfixed_joint_indexes_.rbegin(); it != nonfixed_joint_indexes_.rend(); ++it) {
          Joint& joint = joints_[*it];
          
          // Optimize this specific joint
          ccd_distance_to_target = CCD_single_joint(joint, ccd_distance_to_target);
      }

      // Check for convergence
      if (ccd_distance_to_target < 0.01f) {
          logger::info("Converged at iteration {}", iter);
          break;
      }
      
      if (iter % 10 == 0) {
          logger::debug("Iteration {}, Distance: {}", iter, ccd_distance_to_target);
      }
  }

  // student code end
  logger::info("---- IK Student code end here");

  logger::info("Final distance after {} iterations is {}", ik_params.nb_iterations_, ccd_distance_to_target);

  std::vector<float> angles_in_degrees;
  for(const auto& joint: joints_) {
    if (joint.has_degree_of_freedom()) {
      angles_in_degrees.push_back(glm::degrees(joint.get_angle()));
    }
  }
  logger::info("Joint angles are ({:g})", fmt::join(angles_in_degrees,","));

  return true;
}
