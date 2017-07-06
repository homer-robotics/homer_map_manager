#include <ros/ros.h>

#include <homer_mapnav_msgs/GetPointsOfInterest.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <list>


// offset of the z value for every marker to improve modification 
// of the marker orientation by the user
float marker_z_offset = 0.05;

//Robot* lisa;

interactive_markers::MenuHandler menu_handler;

// Indicates if the user is changing the position of the marker,
//  so the position of the marker in rviz won't be reset during editing by the user.
bool marker_editing = false;

std::vector<homer_mapnav_msgs::PointOfInterest> pois;

// create an interactive marker server
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

// ----- Button Actions -----

// Execute the 'drive to poi' action that is invoked by the corresponding 
// button in the context menu
void executeDriveToAction( 
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO_STREAM("'drive to' button of [" + 
			feedback->marker_name + "] has been clicked");
	//lisa->driveToPOI(feedback->marker_name);
}

// Execute the 'grip object' action that is invoked by the corresponding 
// button in the context menu
void executeGripObjectAction( 
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO_STREAM("'grip object' button of [" + 
			feedback->marker_name + "] has been clicked");
    //lisa->driveToPOI(feedback->marker_name);
	//lisa->gripNearestObject();
}

// Execute the 'find person' action that is invoked by the corresponding 
// button in the context menu
void executeFindPersonAction( 
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO_STREAM("'find person' button of [" + 
			feedback->marker_name + "] has been clicked");
    //lisa->driveToPOI(feedback->marker_name);
	//lisa->searchPersons();
}


void initMenu()
{
	menu_handler.insert("drive to", &executeDriveToAction );
	menu_handler.insert("grip object", &executeGripObjectAction );
	menu_handler.insert("find person", &executeFindPersonAction );
}




// Try to find the poi with the given name, return true if the poi exists
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


// process the actions performed on the marker
void processFeedback( const 
		visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	// Checks if the user releases the mouse button
	//  If so, the poi will be changed
	//  Otherwise, the update of the poi is blocked to avoid 
	//  resetting of the poi position while dragging the marker

	if (feedback->event_type == 
			visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
	{
		ROS_INFO_STREAM("Marker [ " + feedback->marker_name 
				+ " ] changed and is now at "
				<< feedback->pose.position.x << ", " << feedback->pose.position.y
				<< ", " << feedback->pose.position.z );


		homer_mapnav_msgs::PointOfInterest old_poi;
		if (findPoi(feedback->marker_name, old_poi))
		{
			//lisa->modifyPOI(old_poi.name, old_poi.name, 
					//feedback->pose.position.x, 
					//feedback->pose.position.y, 
					//feedback->pose.position.z - marker_z_offset, 
					//tf::getYaw(feedback->pose.orientation));
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



int main(int argc, char** argv)
{
	ros::init(argc, argv, "poi_visualization_marker");

	ros::NodeHandle n;
	//lisa = new Robot(&n); // This defines the robot instance

	initMenu();

	server.reset( new 
			interactive_markers::InteractiveMarkerServer("poi_visualization","",false) );

	homer_mapnav_msgs::GetPointsOfInterest get_pois_srv;

	ros::ServiceClient m_get_pois_client = 
		n.serviceClient<homer_mapnav_msgs::GetPointsOfInterest>("/map_manager/get_pois");

	ros::Rate loopRate(2);

	while(ros::ok())
	{
		loopRate.sleep();
		ros::spinOnce();
		if ( ! marker_editing )
		{
			server->clear();

			if ( ! m_get_pois_client.call(get_pois_srv))
			{
				ROS_DEBUG_STREAM("No POIs received!");
				continue;
			}
			pois = get_pois_srv.response.poi_list.pois;


			for (int i=0; i<pois.size(); i++)
			{

				// create an interactive marker for our server
				visualization_msgs::InteractiveMarker int_poi_marker;
				int_poi_marker.header.frame_id = "map";
				int_poi_marker.header.stamp=ros::Time::now();
				int_poi_marker.name = pois[i].name;
				int_poi_marker.description = pois[i].name;

				int_poi_marker.pose = pois[i].pose;

				int_poi_marker.pose.position.z += marker_z_offset;

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
				poi_arrow_control.markers.push_back( poi_arrow_marker );

				// add the control to the interactive marker
				//int_poi_marker.controls.push_back( poi_arrow_control );




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



				// Add the interactive marker to our collection &
				//  tell the server to call processFeedback() when feedback arrives for it
				server->insert( int_poi_marker, &processFeedback );


				visualization_msgs::InteractiveMarkerControl menu_control;

				menu_control.interaction_mode = 
					visualization_msgs::InteractiveMarkerControl::BUTTON;
				menu_control.always_visible = true;

				// Connect the menu to the marker
				menu_handler.apply( *server, pois[i].name );
				int_poi_marker.controls.push_back(menu_control);
			}

			// 'commit' changes and send to all clients
			server->applyChanges();
		}
	}

	//delete lisa;
}
