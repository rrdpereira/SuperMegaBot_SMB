/*
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: 14.06.2019
 */

#include "smb_planner_rviz/planning_interactive_markers.h"

namespace smb_navigation_rviz
{

PlanningInteractiveMarkers::PlanningInteractiveMarkers(
    const ros::NodeHandle& nh)
    : nh_(nh), marker_server_("smb_planning_markers"), frame_id_("odom"),
      initialized_(false)
{
}

void PlanningInteractiveMarkers::setFrameId(const std::string& frame_id)
{
  frame_id_ = frame_id;
  set_pose_marker_.header.frame_id = frame_id_;
  marker_prototype_.header.frame_id = frame_id_;
}

void PlanningInteractiveMarkers::initialize()
{
  createMarkers();
  initialized_ = true;
}

void PlanningInteractiveMarkers::createMarkers()
{
  // First we set up the set point marker.
  set_pose_marker_.header.frame_id = frame_id_;
  set_pose_marker_.name = "set_pose";
  set_pose_marker_.scale = 1.0;
  set_pose_marker_.controls.clear();

  constexpr double kSqrt2Over2 = sqrt(2.0) / 2.0;

  // Set up controls: x, y, z, and yaw.
  visualization_msgs::InteractiveMarkerControl control;
  set_pose_marker_.controls.clear();
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = kSqrt2Over2;
  control.orientation.z = 0;

  // To add DoF to the markers, uncomment these lines:
  control.name = "rotate_yaw";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  set_pose_marker_.controls.push_back(control);
  //  control.interaction_mode =
  //      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  //  control.name = "move z";
  //  set_pose_marker_.controls.push_back(control);

  control.orientation.w = kSqrt2Over2;
  control.orientation.x = kSqrt2Over2;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move x";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = kSqrt2Over2;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move y";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = 0.9239;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.3827;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move x_y";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = 0.3827;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.9239;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move y_x";
  set_pose_marker_.controls.push_back(control);

  // Create a marker prototype, as the default style for all markers in the
  // marker map:
  marker_prototype_.header.frame_id = frame_id_;
  marker_prototype_.scale = 0.2;
  control.markers.clear();
  control.name = "arrow";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  visualization_msgs::Marker default_marker;
  default_marker.type = visualization_msgs::Marker::ARROW;
  default_marker.color.a = 1.0;
  default_marker.color.r = 0.5;
  default_marker.color.g = 0.0;
  default_marker.color.b = 1.0;
  default_marker.scale.x = 0.25;
  default_marker.scale.y = 0.15;
  default_marker.scale.z = 0.10;
  control.markers.push_back(default_marker);
  visualization_msgs::Marker text_marker;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.scale.z = 0.25;
  text_marker.pose.position.z = 0.5;
  text_marker.text = "placeholder";
  text_marker.color.a = 1.0;
  text_marker.color.r = 1.0;
  text_marker.color.g = 0.0;
  text_marker.color.b = 0.5;
  text_marker.id = 1;
  control.markers.push_back(text_marker);

  marker_prototype_.controls.push_back(control);
}

void PlanningInteractiveMarkers::enableSetPoseMarker(
    const geometry_msgs::Pose& pose)
{
  set_pose_marker_.pose = pose;

  marker_server_.insert(set_pose_marker_);
  marker_server_.setCallback(
      set_pose_marker_.name,
      boost::bind(&PlanningInteractiveMarkers::processSetPoseFeedback, this,
                  _1));
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::disableSetPoseMarker()
{
  marker_server_.erase(set_pose_marker_.name);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::setPose(const geometry_msgs::Pose& pose)
{
  set_pose_marker_.pose = pose;
  marker_server_.setPose(set_pose_marker_.name, set_pose_marker_.pose);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::processSetPoseFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    if (pose_updated_function_)
    {
      pose_updated_function_(feedback->pose);
    }
  }

  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::enableMarker(const std::string& id,
                                              const geometry_msgs::Pose& pose)
{
  auto search = marker_map_.find(id);
  if (search != marker_map_.end())
  {
    // Already exists, just update the pose and enable it.
    search->second.pose = pose;
    marker_server_.insert(search->second);
    marker_server_.applyChanges();
    return;
  }

  // Doesn't exist yet... Have to create it from prototype.
  marker_map_[id] = marker_prototype_;
  marker_map_[id].name = id;
  marker_map_[id].controls[0].markers[1].text = id;
  marker_map_[id].pose = pose;
  marker_server_.insert(marker_map_[id]);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::updateMarkerPose(
    const std::string& id, const geometry_msgs::Pose& pose)
{
  auto search = marker_map_.find(id);
  if (search == marker_map_.end())
  {
    return;
  }

  search->second.pose = pose;
  marker_server_.setPose(id, pose);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::disableMarker(const std::string& id)
{
  marker_server_.erase(id);
  marker_server_.applyChanges();
}

} // end namespace smb_navigation_rviz
