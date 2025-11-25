// header-start
////////////////////////////////////////////////////////////////////////////////
// \file      main.cpp
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

//#include <toml/value.hpp>
#include <toml++/toml.hpp>

#include <fmt/format.h>

#include <exception>
#include <glm/trigonometric.hpp>
#include <glm/vec3.hpp>
#include <iostream>
#include <map>
#include <random>
#include <regex>
#include <string>
#include <vector>
#include <filesystem>

#include "logger.h"
#include "robot.h"
#include "util.h"
#include "poses.h"
#include "context.h"
#include "ikparams.h"

#ifndef PROG_NAME
const std::string PROG_NAME{"ik.exe"};
#endif

#ifndef PROG_VERSION
const std::string PROG_VERSION{"0.00"};
#endif

#ifndef PROG_GITID
const std::string PROG_GITID{"????+0"};
#endif

#ifdef NDEBUG
const std::string COMPILE_MODE{"Release"};
#else
const std::string COMPILE_MODE{"Debug  "};
#endif

///////////////////////////////////////////////////////////////////////////////
//
// Class MainConfig is a possible  way of dealing with console based
// option: it perform all the options processing and checking
//
//

struct MainConfig {
  MainConfig(const int argc, const char* const argv[]);
  void print_version() const;
  void print_help() const;
  void print_banner() const;
  Robot create_robot();
  void check_valid_entries(toml::table& top_level_data);
  void processing(Robot& robot);
  void parse_list_of_numbers(const std::string& current_arg,
                             const std::string& values);
  glm::vec3 get_vec3(const std::string& joint_name,
                     toml::table& a_table,
                     const std::string& a_key);
  float get_float(const std::string& joint_name,
                  toml::table& a_table,
                  const std::string& a_key);

  const std::vector<std::vector<float>>& get_angles() const { return angles_; }
  const std::string& blender_filename() const { return output_blender_script_; }


  std::string program_name_;
  std::string program_version_;
  std::string program_gitid_;
  std::string program_option_;

  // argument parameters

  std::string ik_method_;
  std::string output_file_name_;
  std::string config_file_name_;
  std::string output_blender_script_;

  size_t nb_threads_;
  size_t nb_iterations_;
  uint32_t seed_;

  std::vector<std::vector<float>> angles_;
  std::vector<float> xyz_;
  std::vector<float> rpy_;
};

void
MainConfig::check_valid_entries(toml::table& top_level_data)
{
  //
  // check for the valid entries
  //
  std::map<std::string, std::string> valid_entries{
    {"info", "table"},
    {"links", "array"},
    {"joints", "array"}
  };

  for (const auto& an_entry : valid_entries) {
    auto& [a_key, a_type] = an_entry;
    const auto& node_value = top_level_data[a_key];
    if (not node_value) {
      std::string err_str = fmt::format("Variable '{}' not found in config file", a_key);
      throw std::domain_error(err_str);
    }
    if (a_type == "table" and not node_value.is_table()) {
      throw std::domain_error(
          fmt::format("Variable '{}' has wrong type, must be a table with [...]\n", a_key));
    }
    if (a_type == "array" and not node_value.is_array()) {
      throw std::domain_error(
          fmt::format("Variable '{}' has wrong type, must be an array with [[...]]\n", a_key));
    }
  }
} 

Robot
MainConfig::create_robot()
{
  try {
    //
    // parse and check the robot config file
    //
    logger::info("Parsing config file {0}", config_file_name_);
    toml::table top_level_data = toml::parse_file(config_file_name_);
    check_valid_entries(top_level_data);

    //
    // create the robot
    //

    const auto& tbl_info = top_level_data["info"];

    // robot name
    const auto& robot_name_key = "robot_name";
    const auto& robot_name_node = tbl_info[robot_name_key];
    std::string robot_name;
    if (not robot_name_node) {
      throw robot::exception("Key '{}' is not found in 'info'", robot_name_key);
    }
    robot_name = *(robot_name_node.value<std::string>());

    logger::info("Creating robot {0}", robot_name);

    // robot root path

    const auto& dir_name_key = "root_path";
    const auto& dir_name_node = tbl_info[dir_name_key];
    std::string dir_name;

    if (not dir_name_node) {
      throw robot::exception("Key '{}' is not found in 'info'", dir_name_key);
    }

    dir_name = *(dir_name_node.value<std::string>());

    auto dir_path = std::filesystem::path(dir_name);
    if (not std::filesystem::is_directory(dir_path)) {
      logger::error("The 'root_path' entry '{0}' is not a valid directory", dir_name);
      std::exit(1);
    }

    Robot robot(robot_name, dir_name,  top_level_data);
    return robot;

  } catch (const toml::parse_error& error) {
    logger::fatal(error.what());
    std::exit(1);
  } catch (const std::exception& error) {
    logger::fatal(error.what());
    std::exit(1);
  }
}

//
// print banner
//
//                     1         2         3         4         5
void
MainConfig::print_banner() const
{
  //const std::string compile_date = __DATE__;
  const std::string compile_time = __TIMESTAMP__;
  const std::string title = fmt::format("Forward and Inverse Kinematic ({} version {})", program_option_, program_version_);
  const std::string config = fmt::format("Compiled on {} from source ID {}", compile_time, program_gitid_);

  int w{76};
  fmt::print("##{}##\n", std::string(w, '#'));
  fmt::print("##{}##\n", std::string(w, ' '));
  fmt::print("##{:^{}}##\n", title, w);
  fmt::print("##{:^{}}##\n", config, w);
  fmt::print("##{}##\n", std::string(w, ' '));
  fmt::print("##{}##\n", std::string(w, '#'));
}

void
MainConfig::print_version() const
{
  std::cout << program_name_ << " " << program_version_ << std::endl;
}

void
MainConfig::print_help() const
{
  fmt::print("Usage: {0} [options]\n", program_name_);
  fmt::print("Options:\n");

  std::vector<std::array<std::string, 3>> options = {
      {"-h | --help",  "",            "Display this help"},
      {"--version",    ""  ,          "Display the program version"},
      {"--config",     "filename",    "Name of the input robot config file in toml format, default to config.toml"},
      {"--blender",    "filename",    "Name of the output blender script, default to my_script.py"},
      {"--angles",     "float,...",   "The angles of the joints in degrees for FK computation, can be repeated"},
      {"--xyz",        "float,...",   "The target position for IK computation, 3 values"},
      {"--rpy",        "float,...",   "The target rotation for IK computation, 3 values"},
      {"--method",     "string",      "Extra option to enable variants in CCD IK computation"},
      {"--iterations", "integer",     "Number of iterations for the CCD IK computation"},
      {"--seed",       "integer",     "Force the seed used by the random number generator"}};

  for (const auto& option : options) {
    fmt::print("{0:<13}{1}{2:<12}{3}{4}\n", option[0], Util::csi("italic"),
               option[1], Util::csi("normal"), option[2]);
  }
}

//
// angles are in degree

void
MainConfig::parse_list_of_numbers(const std::string& current_arg, const std::string& values)
{
  std::vector<float> angles;

  try {
    // split string with ,

    std::regex sep(",");
    std::sregex_token_iterator it{values.cbegin(), values.cend(), sep, -1};
    std::sregex_token_iterator end;

    for (; it != end; ++it) {
      std::string sub_str = *it;
      float avalue = std::stof(sub_str);
      if (current_arg == "--angles") {
        angles.push_back(glm::radians(avalue));
      } else if (current_arg == "--xyz") {
        xyz_.push_back(avalue);
      } else if (current_arg == "--rpy") {
        rpy_.push_back(glm::radians(avalue));
      } else {
        throw robot::exception("Runtime error '{}' is not an expected value", current_arg);
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "Error: argument " << current_arg << " requires a number";
    std::cerr << ", but got " << values << std::endl;
    std::cerr << "Error code: " << error.what() << std::endl;
    std::exit(1);
  }
  if (current_arg == "--angles") {
    angles_.push_back(std::move(angles));
  }
}

//
//
//
MainConfig::MainConfig(const int argc, const char* const argv[])
{
  // default parameters values
 
  nb_threads_ = 1;
  program_name_ = PROG_NAME;
  program_version_ = PROG_VERSION;
  program_gitid_ = PROG_GITID;
  program_option_ = COMPILE_MODE;

  output_file_name_ = "results.txt";
  output_blender_script_ = "my_script.py";
  config_file_name_ = "config.toml";

  nb_iterations_ = 100;
  seed_ = std::random_device{}();

  for (int idx = 1; idx < argc; ++idx) {
    std::string current_arg = argv[idx];
    if (current_arg == "-h" || current_arg == "--help") {
      print_help();
      std::exit(0);
    }
    if (current_arg == "--version") {
      print_version();
      std::exit(0);
    }
    if (current_arg == "--debug") {
      logger::enable_debug();
      continue;
    }

    // from this point on all arguments
    // have one parameter

    idx++;
    if (idx >= argc) {
      std::cerr << "Error: argument " << current_arg
                << " needs a value, or unknown" << std::endl;
      std::exit(1);
    }
    std::string current_value{argv[idx]};

    if (current_arg == "--output") {
      output_file_name_ = argv[idx];
      continue;
    }

    if (current_arg == "--blender") {
      output_blender_script_ = argv[idx];
      continue;
    }

    if (current_arg == "--config") {
      config_file_name_ = argv[idx];
      continue;
    }

    if ((current_arg == "--iterations") or
        (current_arg == "--seed")) {
      size_t v;
      try {
        v = std::stoul(current_value);
      } catch (const std::exception& error) {
        std::cerr << "Error: argument " << current_arg << " requires a number";
        std::cerr << ", but got " << current_value << std::endl;
        std::exit(1);
      }
      if (current_arg == "--seed") {
      seed_ = v;
      } else {
        nb_iterations_ = v;
      }
      continue;
    }

    if ((current_arg == "--xyz") or
        (current_arg == "--rpy") or
        (current_arg == "--angles")) {
      parse_list_of_numbers(current_arg, current_value);
      continue;
    }

    // free text argument to be passed
    // to the IK algorithm
    if (current_arg == "--method") {
      ik_method_ = argv[idx];
      continue;
    }
  }
}

// standard main with MainConfig

int
main(int argc, char* argv[])
{

  MainConfig main_config(argc, argv);
  main_config.print_banner();


  Robot robot = main_config.create_robot();
  robot.info();

  // Recording the robot position to be used
  // to generate script file
  // (example to a blender python script)

  Poses poses;
  {
    auto pose = robot.get_current_pose();
    poses.add_pose(pose);
  }


  // forward kinematic check parameters

  for(const auto& angles: main_config.get_angles()) {
    if (angles.size() != robot.degrees_of_freedom()) {
      logger::error("Incorrect number of angles for the angle parameters, got {}, but 0 or {} expected",
        angles.size(), robot.degrees_of_freedom());
      std::exit(-1);
    }
  }

  if (main_config.angles_.size() > 0) {
    logger::info("---------------------------------");
    logger::info("--   Start Forward Kinematic   --");
    logger::info("---------------------------------");

    for(const auto& angles : main_config.get_angles()) {
      logger::info("---------------------------------");
      auto result = robot.forward_kinematic(angles);
      if (not result) {
        logger::error("Cannot compute forward kinematic, quit now");
        std::exit(-1);
      }
      auto pose = robot.get_current_pose();
      poses.add_pose(pose);
    }
  }

  // inverse kinematic, check parameters
 
  if (not((main_config.xyz_.size() == 3 and main_config.rpy_.size() == 3) or
          (main_config.xyz_.size() == 0 and main_config.rpy_.size() == 0))) {
    logger::error("Incorrect value for the IK parameters (xyz or rpy)");
    std::exit(-1);
  }

  if (main_config.xyz_.size() == 3 and main_config.rpy_.size() == 3) {

    logger::info("---------------------------------");
    logger::info("--   Start Inverse Kinematic   --");
    logger::info("---------------------------------");
    logger::info("Using seed {}", main_config.seed_);

    IKParams ik_params(
        main_config.xyz_, 
        main_config.rpy_, 
        main_config.seed_, 
        main_config.nb_iterations_,
        main_config.ik_method_);
    robot.inverse_kinematic(ik_params);
    auto pose = robot.get_current_pose();
    poses.add_pose(pose);
  }

  logger::info("---------------------------------");
  logger::info("--    Write Blender Script     --");
  logger::info("---------------------------------");

  poses.write_blender_script(main_config.blender_filename());

  return 0;
}
