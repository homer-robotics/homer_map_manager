# homer_map_manager

## Introduction 

The map_manager is the center of communication between homer_mapping and homer_navigation
The interaction of these nodes can be seen in the screenshot of the rqt_graph.

It manages the map currently created by the mapping as well as other map layers. Currently these are the SLAM map, the current laser data in a further layer and a masking layer in which obstacles or free areas can be drawn into the map with the help of the GUI.
Each time a SLAM map is sent from the mapping node, it is overwritten with all other map layers (in the order SLAM, masking, laser data) and sent as a merged map.
In addition, map_manager manages all created Points of Interest (POIs), which are used, for example, as destinations for navigation.
The node is also responsible for saving and loading map layers and POIs. The SLAM level and the masking level are taken into account.

## Topics 

#### Publisher 
* `/map`: The current map composed of all activated map layers. This is displayed in the GUI and used for navigation.
* `/map_manager/poi_list`: Sends a vector with all current POIs. This Publisher is always triggered when a POI changes or a new one is added.
* `/map_manager/loaded_map`: When a map is loaded, this topic sends the loaded SLAM layer to the homer_mapping node.
* `/map_manager/mask_slam`: Via the GUI the SLAM map can be changed. These modifications are sent from the map_manager to the homer_mapping via this topic.

#### Subscriber

* `/homer_mapping/slam_map (nav_msgs/OccupancyGrid)`: This will receive the current SLAM map.
* `/map_manager/save_map (map_messages/SaveMap)`: Receives the command to save the map including the file name.
* `/map_manager/load_map (map_messages/SaveMap)`: Loads a map and replaces all previous map layers with the loaded ones.
* `/map_manager/toggle_map_visibility (map_messages/MapLayers)`: This option activates or deactivates individual map layers. Deactivated map layers are no longer taken into account when merging the map and are therefore not displayed in the GUI or used for navigation.
* `/scan (nav_msgs/LaserScan)`: The current laser scan drawn into the laser scan layer.
* `/map_manager/add_POI (map_messages/PointOfInterest)`: This can be used to add a POI.
* `/map_manager/modify_POI (map_messages/ModifyPOI)`: With this you can change an existing POI (Name, Position,...)
* `/map_manager/delete_POI (map_messages/DeletePointOfInterest)`: This can be used to delete an existing POI.
* `/map_manager/modify_map (map_messages/ModifyMap)`: This topic is used to send the coordinates of the polygons that have been masked via the GUI. The map layer to be modified (SLAM or masking layer) is also specified.
* `/map_manager/reset_maps (std_msgs/Empty)`: This will reset all map layers.
