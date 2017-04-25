#ifndef MaskingManager_H
#define MaskingManager_H

#include <ros/ros.h>

#include <homer_mapnav_msgs/ModifyMap.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#include <sstream>

/**
 * @class  MaskingManager
 * @brief  Manages a map that can overwrite values in the SLAM map or store it
 * in a separate layer
 * @author Malte Knauf, David Gossow
 */
class MaskingManager
{
public:
  /** @brief The constructor. */
  MaskingManager(nav_msgs::MapMetaData mapInfo);

  /** @brief The destructor. */
  virtual ~MaskingManager();

  nav_msgs::OccupancyGrid::ConstPtr
  updateMapInfo(const nav_msgs::MapMetaData &mapInfo);

  /** modifies either the masking layer or the slam layer (accordingly to the
   * given map layer in the msg */
  nav_msgs::OccupancyGrid::ConstPtr
  modifyMap(homer_mapnav_msgs::ModifyMap::ConstPtr msg);

  /** resets the masking map layer */
  nav_msgs::OccupancyGrid::ConstPtr resetMap();

  /** replaces the masking map layer */
  void replaceMap(nav_msgs::OccupancyGrid map);

  void applyMasking(nav_msgs::OccupancyGrid& map);

private:
  /** stores the masking values in the dedicated masking map */
  nav_msgs::OccupancyGrid m_MaskingMap;
  /** stores the masking values that are afterwards sent to the slam map */
  nav_msgs::OccupancyGrid m_SlamMap;

  /** tools to draw masking polygons */
  void drawPolygon(std::vector<geometry_msgs::Point> vertices, int value,
                   int mapLayer);
  void drawLine(std::vector<int> &data, int startX, int startY, int endX,
                int endY, int value);
  void fillPolygon(std::vector<int> &data, int x, int y, int value);

  struct mapPoint{
  int x = 0;
  int y = 0;
  char value = homer_mapnav_msgs::ModifyMap::BLOCKED;
  };

  std::vector<mapPoint> m_modified_points;

};

#endif
