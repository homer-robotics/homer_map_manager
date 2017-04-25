#include <homer_map_manager/Managers/MapManager.h>

MapManager::MapManager(ros::NodeHandle *nh)
{
  m_MapPublisher = nh->advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

  m_map_layers.push_back(homer_mapnav_msgs::MapLayers::KINECT_LAYER);
  m_map_layers.push_back(homer_mapnav_msgs::MapLayers::RAPID_MAP);

  m_MapVisibility[homer_mapnav_msgs::MapLayers::MASKING_LAYER] = true;

  // enable all map layers
  for (int i = 0; i < m_map_layers.size(); i++)
  {
    m_MapVisibility[m_map_layers[i]] = true;
  }

  m_MaskingManager = 0;
}

MapManager::~MapManager()
{
}

void MapManager::updateMapLayer(int type,
                                nav_msgs::OccupancyGrid::ConstPtr layer)
{
  m_MapLayers[type] = layer;
  sendMergedMap();
}

void MapManager::clearMapLayers()
{
  m_MapLayers.clear();
}

void MapManager::toggleMapVisibility(int type, bool state)
{
  ROS_INFO_STREAM("MapManager: " << type << ": " << state);
  m_MapVisibility[type] = state;
}

nav_msgs::OccupancyGrid::ConstPtr MapManager::getMapLayer(int type)
{
  if (m_MapLayers.find(type) == m_MapLayers.end())
    return nav_msgs::OccupancyGrid::ConstPtr();
  return m_MapLayers[type];
}

/**
 * Sends the SLAM map (OccupancyGrid) and (if available and enabled) other
 * merged map layers to the gui node
 *
 */
void MapManager::sendMergedMap()
{
  if (m_MapLayers.find(homer_mapnav_msgs::MapLayers::SLAM_LAYER) ==
      m_MapLayers.end())
  {
    ROS_DEBUG_STREAM("SLAM map is missing!");
    return;
  }
  int k;
  nav_msgs::OccupancyGrid mergedMap(
      *(m_MapLayers[homer_mapnav_msgs::MapLayers::SLAM_LAYER]));


  if(m_MapVisibility[homer_mapnav_msgs::MapLayers::MASKING_LAYER])
  {
      if(m_MaskingManager)
      {
          m_MaskingManager->applyMasking(mergedMap); 
      }
  }

  // bake maps
  for (int j = 0; j < m_map_layers.size(); j++)
  {
    if (m_MapLayers.find(m_map_layers[j]) != m_MapLayers.end() &&
        m_MapVisibility[m_map_layers[j]])
    {
      size_t mapsize = m_MapLayers[m_map_layers[j]]->info.height *
                       m_MapLayers[m_map_layers[j]]->info.width;
      const std::vector<signed char> *tempdata =
          &m_MapLayers[m_map_layers[j]]->data;
      const int frei = homer_mapnav_msgs::ModifyMap::FREE;
      signed char currentvalue = 0;
      for (int i = 0; i < mapsize; i++)
      {
        currentvalue = tempdata->at(i);
        if (currentvalue > 50 || currentvalue == frei)
        {
          mergedMap.data[i] = currentvalue;
        }
      }
    }
  }
  mergedMap.header.stamp = ros::Time::now();
  mergedMap.header.frame_id = "map";

  m_MapPublisher.publish(mergedMap);
  ROS_DEBUG_STREAM("Publishing map");
}
