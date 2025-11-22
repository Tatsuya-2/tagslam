// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// NOTE: This executable is disabled on ROS2 Humble due to rosbag2_transport::Player API limitations.
// For Humble, use: ros2 bag play <bag_file> (in terminal 1)
//              and ros2 launch tagslam tagslam.launch.py use_sim_time:=True ... (in terminal 2)
// This file is preserved for future compatibility with ROS2 Jazzy/Rolling.

#include <filesystem>
#include <rosbag2_transport/recorder.hpp>
#include <tagslam/enhanced_player.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/sync_and_detect.hpp>
#include <tagslam/tagslam.hpp>

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("tagslam_from_bag"));
}

static void printTopics(
  const std::string & label, const std::vector<std::string> & topics)
{
  for (const auto & t : topics) {
    LOG_INFO(label << ": " << t);
  }
}

#if 0
static std::vector<std::string> merge(
  const std::vector<std::vector<std::string>> & v)
{
  {
    std::vector<std::string> all;
    for (const auto & vv : v) {
      all.insert(all.end(), vv.begin(), vv.end());
    }
    return (all);
  }
}
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);

  auto tagslam_node = std::make_shared<tagslam::TagSLAM>(node_options);
  exec.add_node(tagslam_node);

  const auto recorded_topics = tagslam_node->getPublishedTopics();
  printTopics("recorded topic", recorded_topics);

  const std::string in_uri =
    tagslam_node->declare_parameter<std::string>("in_bag", "");
  if (in_uri.empty()) {
    LOG_ERROR("must provide valid in_bag parameter!");
    return (-1);
  }
  LOG_INFO("using input bag: " << in_uri);
  if (!std::filesystem::exists(in_uri)) {
    LOG_ERROR("cannot find input bag: " << in_uri);
  }
  rclcpp::NodeOptions player_options;
  using Parameter = rclcpp::Parameter;
  const auto images = tagslam_node->getImageTopics();
  const auto tags = tagslam_node->getTagTopics();
  const auto odoms = tagslam_node->getOdomTopics();
  // const auto in_topics = merge({images, tags, odoms});
  // printTopics("taglslam input topics", in_topics);

  player_options.parameter_overrides(
    {Parameter("storage.uri", in_uri),  // Parameter("play.topics", in_topics),
     Parameter("play.clock_publish_on_topic_publish", true),
     Parameter("play.start_paused", true), Parameter("play.rate", 1000.0),
     Parameter("play.disable_keyboard_controls", true)});
  auto player_node =
    std::make_shared<tagslam::EnhancedPlayer>("rosbag_player", player_options);
  player_node->get_logger().set_level(rclcpp::Logger::Level::Warn);
  exec.add_node(player_node);

  std::shared_ptr<tagslam::SyncAndDetect> sync_node;
  if (!player_node->hasTopics(tags)) {
    if (!player_node->hasImageTopics(images)) {
      LOG_ERROR("no images nor tag topics found in bag, tagslam may hang!");
    } else {
      LOG_INFO(
        "detecting from raw images is slow, consider using sync_and_detect!");
      rclcpp::NodeOptions node_options;
      node_options.use_intra_process_comms(true);
      sync_node = std::make_shared<tagslam::SyncAndDetect>(node_options);
      exec.add_node(sync_node);
    }
  }
  if (!odoms.empty() && !player_node->hasTopics(odoms)) {
    LOG_ERROR("odom topics not in bag, tagslam may hang!");
  }

  const std::string out_uri =
    tagslam_node->declare_parameter<std::string>("out_bag", "");
  size_t num_frames =
    tagslam_node->get_parameter_or<int>("max_number_of_frames", 0);

  std::shared_ptr<rosbag2_transport::Recorder> recorder_node;
  if (!out_uri.empty()) {
    LOG_INFO("writing detected tags to bag: " << out_uri);
    rclcpp::NodeOptions recorder_options;
    recorder_options.parameter_overrides(
      {Parameter("storage.uri", out_uri),
       Parameter("record.disable_keyboard_controls", true),
       Parameter("record.topics", recorded_topics)});

    recorder_node = std::make_shared<rosbag2_transport::Recorder>(
      "rosbag_recorder", recorder_options);
    exec.add_node(recorder_node);
  } else {
    LOG_INFO("no out_bag parameter set, publishing as messages!");
  }

  while (player_node->play_next() && rclcpp::ok()) {
    exec.spin_some();
    if (tagslam_node->getNumberOfFrames() >= num_frames) {
      break;
    }
  }
  if (tagslam_node->getNumberOfFrames() < num_frames) {
    // only finalize if we have not reached max number of frames
    tagslam_node->finalize();
  }
  rclcpp::shutdown();
  return 0;
}
