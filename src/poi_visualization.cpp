#include <ros/ros.h>

#include <homer_mapnav_msgs/GetPointsOfInterest.h>
#include <homer_mapnav_msgs/ModifyPOI.h>
#include <homer_mapnav_msgs/NavigateToPOI.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

ros::Publisher modify_poi_pub;
ros::Publisher navigate_to_poi_pub;
ros::ServiceClient m_get_pois_client;

float marker_z_offset = 0.05;

interactive_markers::MenuHandler menu_handler;
bool marker_editing = false;
std::vector<homer_mapnav_msgs::PointOfInterest> pois;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

void executeDriveToAction(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_DEBUG_STREAM("'drive to' button of [" + feedback->marker_name +
                   "] has been clicked");
  homer_mapnav_msgs::NavigateToPOI msg;
  msg.poi_name = feedback->marker_name;
  navigate_to_poi_pub.publish(msg);
}

void initMenu()
{
  menu_handler.insert("drive to", &executeDriveToAction);
}

bool findPoi(std::string name, homer_mapnav_msgs::PointOfInterest &found_poi)
{
  for (auto poi : pois)
  {
    if (poi.name == name)
    {
      found_poi = poi;
      return true;
    }
  }
  return false;
}

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // Checks if the user releases the mouse button
  //  If so, the poi will be changed
  //  Otherwise, the update of the poi is blocked to avoid
  //  resetting of the poi position while dragging the marker

  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
  {
    ROS_DEBUG_STREAM(
        "Marker [ " + feedback->marker_name + " ] changed and is now at "
        << feedback->pose.position.x << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z);

    homer_mapnav_msgs::PointOfInterest old_poi;
    if (findPoi(feedback->marker_name, old_poi))
    {
      homer_mapnav_msgs::ModifyPOI msg;
      msg.poi = old_poi;
      msg.old_name = old_poi.name;
      msg.poi.pose = feedback->pose;
      modify_poi_pub.publish(msg);
    }
    else
    {
      ROS_WARN_STREAM("COULDN'T FIND MARKER");
    }

    marker_editing = false;
  }
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN)
  {
    marker_editing = true;
  }
}

void idle()
{
  if (!marker_editing)
  {
    server->clear();
    homer_mapnav_msgs::GetPointsOfInterest get_pois_srv;

    if (!m_get_pois_client.call(get_pois_srv))
    {
      ROS_DEBUG_STREAM("No POIs received!");
      return;
    }
    pois = get_pois_srv.response.poi_list.pois;

    for (int i = 0; i < pois.size(); i++)
    {
      // create an interactive marker for our server
      visualization_msgs::InteractiveMarker int_poi_marker;
      int_poi_marker.header.frame_id = "map";
      int_poi_marker.header.stamp = ros::Time::now();
      int_poi_marker.name = pois[i].name;
      int_poi_marker.description = pois[i].name;

      int_poi_marker.pose = pois[i].pose;

      int_poi_marker.pose.position.z = marker_z_offset;

      int_poi_marker.scale = 0.3;

      // create a arrow marker
      visualization_msgs::Marker poi_arrow_marker;
      poi_arrow_marker.type = visualization_msgs::Marker::ARROW;
      poi_arrow_marker.pose.position.x -= 0.08;
      poi_arrow_marker.scale.x = 0.16;
      poi_arrow_marker.scale.y = 0.05;
      poi_arrow_marker.scale.z = 0.04;
      poi_arrow_marker.color.r = 0.5;
      poi_arrow_marker.color.g = 0.5;
      poi_arrow_marker.color.b = 0.5;
      poi_arrow_marker.color.a = 1.0;

      // create a non-interactive control which contains the arrow
      visualization_msgs::InteractiveMarkerControl poi_arrow_control;
      poi_arrow_control.always_visible = true;
      poi_arrow_control.markers.push_back(poi_arrow_marker);

      // add the control to the interactive marker
      // int_poi_marker.controls.push_back( poi_arrow_control );

      visualization_msgs::InteractiveMarkerControl control;

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.markers.push_back(poi_arrow_marker);
      control.name = "move_plane";
      control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
      int_poi_marker.controls.push_back(control);
      control.markers.clear();

      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "move_x";
      control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_poi_marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "move_y";
      control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_poi_marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "rotate_z";
      control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_poi_marker.controls.push_back(control);

      server->insert(int_poi_marker, &processFeedback);

      visualization_msgs::InteractiveMarkerControl menu_control;

      menu_control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::BUTTON;
      menu_control.always_visible = true;

      menu_handler.apply(*server, pois[i].name);
      int_poi_marker.controls.push_back(menu_control);
    }
    server->applyChanges();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poi_visualization_marker");

  ros::NodeHandle n;

  initMenu();

  server.reset(new interactive_markers::InteractiveMarkerServer(
      "poi_visualization", "", false));

  m_get_pois_client = n.serviceClient<homer_mapnav_msgs::GetPointsOfInterest>(
      "/map_manager/get_pois");

  modify_poi_pub =
      n.advertise<homer_mapnav_msgs::ModifyPOI>("/map_manager/modify_POI", 3);

  navigate_to_poi_pub = n.advertise<homer_mapnav_msgs::NavigateToPOI>(
      "/homer_navigation/navigate_to_POI", 3);

  ros::Rate loopRate(10);
  while (ros::ok())
  {
    loopRate.sleep();
    ros::spinOnce();
    idle();
  }
}
