#include <homer_map_manager/Managers/MapManager.h>
#include <omp.h>

MapManager::MapManager(ros::NodeHandle *nh) {
    m_MapPublisher = nh->advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

    m_map_layers.push_back(homer_mapnav_msgs::MapLayers::SLAM_LAYER);
    m_map_layers.push_back(homer_mapnav_msgs::MapLayers::KINECT_LAYER);
    m_map_layers.push_back(homer_mapnav_msgs::MapLayers::RAPID_MAP);
    m_map_layers.push_back(homer_mapnav_msgs::MapLayers::MASKING_LAYER);
    m_laser_layers.push_back(homer_mapnav_msgs::MapLayers::HOKUYO_FRONT);
    m_laser_layers.push_back(homer_mapnav_msgs::MapLayers::SICK_LAYER);

    // enable all map layers

    for (int i = 0; i < m_map_layers.size(); i++) {
        m_MapVisibility[m_map_layers[i]] = true;
    }

    for (int i = 0; i < m_laser_layers.size(); i++) {
        m_MapVisibility[m_laser_layers[i]] = true;
    }

    //try {
        //m_TransformListener.waitForTransform("/base_link", "/laser",
                                             //ros::Time(0), ros::Duration(3));
        //m_TransformListener.lookupTransform("/base_link", "/laser",
                                            //ros::Time(0), m_sick_transform);
        //m_TransformListener.lookupTransform("/base_link", "/hokuyo_front",
                                            //ros::Time(0), m_hokuyo_transform);
        //m_got_transform = true;
    //} catch (tf::TransformException ex) {
        //ROS_ERROR("%s", ex.what());
        //m_got_transform = false;
    //}
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    m_pose = boost::make_shared<geometry_msgs::PoseStamped>(pose);
}

MapManager::~MapManager() {}

void MapManager::updateMapLayer(int type,
                                nav_msgs::OccupancyGrid::ConstPtr layer) {
    m_MapLayers[type] = layer;
    if (type == homer_mapnav_msgs::MapLayers::SLAM_LAYER) {
        sendMergedMap();
    }
}

void MapManager::clearMapLayers() { m_MapLayers.clear(); }

void MapManager::toggleMapVisibility(int type, bool state) {
    ROS_INFO_STREAM("MapManager: " << type << ": " << state);
    m_MapVisibility[type] = state;
}

void MapManager::updateLaser(int layer,
                             const sensor_msgs::LaserScan::ConstPtr &msg) {
    m_laserLayers[layer] = msg;
}

nav_msgs::OccupancyGrid::ConstPtr MapManager::getMapLayer(int type) {
    if (m_MapLayers.find(type) == m_MapLayers.end())
        return nav_msgs::OccupancyGrid::ConstPtr();
    return m_MapLayers[type];
}

/**
 * Sends the SLAM map (OccupancyGrid) and (if available and enabled) other
 * merged map layers to the gui node
 *
 */
void MapManager::sendMergedMap() {
    if (m_MapLayers.find(homer_mapnav_msgs::MapLayers::SLAM_LAYER) ==
        m_MapLayers.end()) {
        ROS_DEBUG_STREAM("SLAM map is missing!");
        return;
    }
    int k;
    nav_msgs::OccupancyGrid mergedMap(
        *(m_MapLayers[homer_mapnav_msgs::MapLayers::SLAM_LAYER]));
    // bake maps
    for (int j = 1; j < m_map_layers.size(); j++) {
        if (m_MapLayers.find(m_map_layers[j]) != m_MapLayers.end() &&
            m_MapVisibility[m_map_layers[j]]) {
            // calculating parallel on 8 threads
            omp_set_num_threads(8);
            size_t mapsize = m_MapLayers[m_map_layers[j]]->info.height *
                             m_MapLayers[m_map_layers[j]]->info.width;
            const std::vector<signed char> *tempdata =
                &m_MapLayers[m_map_layers[j]]->data;
            const int frei = homer_mapnav_msgs::ModifyMap::FREE;
            signed char currentvalue = 0;
#pragma omp parallel for private(currentvalue) shared(mapsize, tempdata, \
                                                      mergedMap)
            for (int i = 0; i < mapsize; i++) {
                currentvalue = tempdata->at(i);
                if (currentvalue > 50 || currentvalue == frei) {
                    mergedMap.data[i] = currentvalue;
                }
            }
        }
    }
    // bake Lasers

    //try {
        //int data;
        //if (m_got_transform) {
            //for (int j = 0; j < m_laser_layers.size(); j++) {
                //if (m_laserLayers.find(m_laser_layers[j]) !=
                        //m_laserLayers.end() &&
                    //m_MapVisibility[m_laser_layers[j]]) {
                    //if (m_laser_layers[j] ==
                        //homer_mapnav_msgs::MapLayers::SICK_LAYER) {
                        //data = homer_mapnav_msgs::ModifyMap::BLOCKED;
                    //} else if (m_laser_layers[j] ==
                               //homer_mapnav_msgs::MapLayers::HOKUYO_BACK) {
                        //data = homer_mapnav_msgs::ModifyMap::HOKUYO;
                    //} else if (m_laser_layers[j] ==
                               //homer_mapnav_msgs::MapLayers::HOKUYO_FRONT) {
                        //data = homer_mapnav_msgs::ModifyMap::HOKUYO;
                    //}
                    //tf::StampedTransform pose_transform;
                    //m_TransformListener.waitForTransform(
                        //"/map", "/base_link",
                        //m_laserLayers[m_laser_layers[j]]->header.stamp,
                        //ros::Duration(0.09));
                    //m_TransformListener.lookupTransform(
                        //"/map", "/base_link",
                        //m_laserLayers[m_laser_layers[j]]->header.stamp,
                        //pose_transform);

                    ////	pose_transform.setTransform(Transform);

                    //for (int i = 0;
                         //i < m_laserLayers[m_laser_layers[j]]->ranges.size();
                         //i++) {
                        //if (m_laserLayers[m_laser_layers[j]]->ranges[i] <
                                //m_laserLayers[m_laser_layers[j]]->range_max &&
                            //m_laserLayers[m_laser_layers[j]]->ranges[i] >
                                //m_laserLayers[m_laser_layers[j]]->range_min) {
                            //float alpha =
                                //m_laserLayers[m_laser_layers[j]]->angle_min +
                                //i *
                                    //m_laserLayers[m_laser_layers[j]]
                                        //->angle_increment;
                            //geometry_msgs::Point point;
                            //tf::Vector3 pin;
                            //tf::Vector3 pout;
                            //pin.setX(
                                //cos(alpha) *
                                //m_laserLayers[m_laser_layers[j]]->ranges[i]);
                            //pin.setY(
                                //sin(alpha) *
                                //m_laserLayers[m_laser_layers[j]]->ranges[i]);
                            //pin.setZ(0);

                            //if (m_laser_layers[j] ==
                                //homer_mapnav_msgs::MapLayers::SICK_LAYER) {
                                //pout = pose_transform * m_sick_transform * pin;
                            //} else if (m_laser_layers[j] ==
                                       //homer_mapnav_msgs::MapLayers::
                                           //HOKUYO_FRONT) {
                                //pout =
                                    //pose_transform * m_hokuyo_transform * pin;
                            //}
                            //point.x = pout.x();
                            //point.y = pout.y();
                            //point.z = 0;

                            //Eigen::Vector2i map_point = map_tools::toMapCoords(
                                //point,
                                //m_MapLayers
                                    //[homer_mapnav_msgs::MapLayers::SLAM_LAYER]
                                        //->info.origin,
                                //m_MapLayers
                                    //[homer_mapnav_msgs::MapLayers::SLAM_LAYER]
                                        //->info.resolution);
                            //k = map_point.y() *
                                    //m_MapLayers[homer_mapnav_msgs::MapLayers::
                                                    //SLAM_LAYER]
                                        //->info.width +
                                //map_point.x();
                            //if (k < 0 ||
                                //k > m_MapLayers[homer_mapnav_msgs::MapLayers::
                                                    //SLAM_LAYER]
                                        //->data.size()) {
                                //continue;
                            //} else {
                                //mergedMap.data[k] = data;
                            //}
                        //}
                    //}
                //}
            //}
        //} else {
            //try {
                //if (m_TransformListener.waitForTransform("/base_link", "/laser",
                                                         //ros::Time(0),
                                                         //ros::Duration(0.1))) {
                    //m_TransformListener.lookupTransform(
                        //"/base_link", "/laser", ros::Time(0), m_sick_transform);
                    //m_TransformListener.lookupTransform(
                        //"/base_link", "/hokuyo_front", ros::Time(0),
                        //m_hokuyo_transform);
                    //m_got_transform = true;
                //}
            //} catch (tf::TransformException ex) {
                //ROS_ERROR("%s", ex.what());
                //m_got_transform = false;
            //}
        //}
    //} catch (tf::TransformException ex) {
        //ROS_ERROR_STREAM(ex.what());
    //}
    mergedMap.header.stamp = ros::Time::now();
    mergedMap.header.frame_id = "map";

    m_MapPublisher.publish(mergedMap);
    ROS_DEBUG_STREAM("Publishing map");
}
