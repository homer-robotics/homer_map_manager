#include <homer_map_manager/Managers/MaskingManager.h>

using namespace std;

MaskingManager::MaskingManager(nav_msgs::MapMetaData mapInfo)
{
  m_MaskingMap.info = mapInfo;
  m_MaskingMap.data.resize(m_MaskingMap.info.width * m_MaskingMap.info.height);
  std::fill(m_MaskingMap.data.begin(), m_MaskingMap.data.end(),
            homer_mapnav_msgs::ModifyMap::NOT_MASKED);

  m_SlamMap.info = mapInfo;
  m_SlamMap.data.resize(m_SlamMap.info.width * m_SlamMap.info.height);
  std::fill(m_SlamMap.data.begin(), m_SlamMap.data.end(),
            homer_mapnav_msgs::ModifyMap::NOT_MASKED);
}

MaskingManager::~MaskingManager()
{
}

nav_msgs::OccupancyGrid::ConstPtr
MaskingManager::updateMapInfo(const nav_msgs::MapMetaData &mapInfo)
{
  if (m_SlamMap.info.width < mapInfo.width ||
      m_SlamMap.info.height < mapInfo.height)
  {
    m_SlamMap.info = mapInfo;

    int x_add_left = (m_MaskingMap.info.origin.position.x -
                      mapInfo.origin.position.x + 0.025) /
                     mapInfo.resolution;
    int y_add_up = (m_MaskingMap.info.origin.position.y -
                    mapInfo.origin.position.y + 0.025) /
                   mapInfo.resolution;
    ROS_INFO_STREAM("x add " << x_add_left << " y add " << y_add_up);

    std::vector<signed char> tmpData = m_MaskingMap.data;

    m_MaskingMap.data.resize(mapInfo.width * mapInfo.height);
    std::fill(m_MaskingMap.data.begin(), m_MaskingMap.data.end(),
              homer_mapnav_msgs::ModifyMap::NOT_MASKED);

    for (int x = 0; x < m_MaskingMap.info.width; x++)
    {
      for (int y = 0; y < m_MaskingMap.info.height; y++)
      {
        int i = y * m_MaskingMap.info.width + x;
        int j = (y + y_add_up) * mapInfo.width + x + x_add_left;
        m_MaskingMap.data[j] = tmpData[i];
      }
    }
    m_MaskingMap.info = mapInfo;
  }
  return boost::make_shared<const ::nav_msgs::OccupancyGrid>(m_MaskingMap);
}

nav_msgs::OccupancyGrid::ConstPtr
MaskingManager::modifyMap(homer_mapnav_msgs::ModifyMap::ConstPtr msg)
{
  // reset SLAM mask map before each masking
  m_SlamMap.data.resize(m_SlamMap.info.height * m_SlamMap.info.width);
  std::fill(m_SlamMap.data.begin(), m_SlamMap.data.end(),
            homer_mapnav_msgs::ModifyMap::NOT_MASKED);
  drawPolygon(msg->region, msg->maskAction, msg->mapLayer);

  nav_msgs::OccupancyGrid::ConstPtr ret;
  if (msg->mapLayer == 0)
  {
    ret = boost::make_shared<const ::nav_msgs::OccupancyGrid>(m_SlamMap);
  }
  else
  {
    ret = boost::make_shared<const ::nav_msgs::OccupancyGrid>(m_MaskingMap);
  }
  return ret;
}

nav_msgs::OccupancyGrid::ConstPtr MaskingManager::resetMap()
{
  m_MaskingMap.info.width = 1;
  m_MaskingMap.info.height = 1;
  m_MaskingMap.info.resolution = 1;
  m_MaskingMap.info.origin.position.x = 0;
  m_MaskingMap.info.origin.position.y = 0;
  m_MaskingMap.data.resize(m_MaskingMap.info.width * m_MaskingMap.info.height);
  std::fill(m_MaskingMap.data.begin(), m_MaskingMap.data.end(),
            homer_mapnav_msgs::ModifyMap::NOT_MASKED);

  m_SlamMap.info = m_MaskingMap.info;
  m_SlamMap.data.resize(m_SlamMap.info.width * m_SlamMap.info.height);
  std::fill(m_SlamMap.data.begin(), m_SlamMap.data.end(),
            homer_mapnav_msgs::ModifyMap::NOT_MASKED);

  nav_msgs::OccupancyGrid::ConstPtr ret =
      boost::make_shared<const ::nav_msgs::OccupancyGrid>(m_MaskingMap);
  return ret;
}

void MaskingManager::replaceMap(nav_msgs::OccupancyGrid map)
{
  if (map.data.size() != 0)
    m_MaskingMap = map;
  else
    std::fill(m_MaskingMap.data.begin(), m_MaskingMap.data.end(),
              homer_mapnav_msgs::ModifyMap::NOT_MASKED);
}

void MaskingManager::drawPolygon(vector<geometry_msgs::Point> vertices,
                                 int value, int mapLayer)
{
  if (vertices.size() == 0)
  {
    ROS_INFO_STREAM("No vertices given!");
    return;
  }
  // make temp. map
  std::vector<int> data(m_MaskingMap.info.width * m_MaskingMap.info.height);
  std::fill(data.begin(), data.end(), 0);

  // draw the lines surrounding the polygon
  for (unsigned int i = 0; i < vertices.size(); i++)
  {
    int i2 = (i + 1) % vertices.size();
    drawLine(data, vertices[i].x, vertices[i].y, vertices[i2].x, vertices[i2].y,
             1);
  }
  // calculate a point in the middle of the polygon
  float midX = 0;
  float midY = 0;
  for (unsigned int i = 0; i < vertices.size(); i++)
  {
    midX += vertices[i].x;
    midY += vertices[i].y;
  }
  midX /= vertices.size();
  midY /= vertices.size();
  // fill polygon
  fillPolygon(data, (int)midX, (int)midY, 1);

  // copy polygon to masking map or slam map (according to parameter mapLayer)
  for (int i = 0; i < data.size(); i++)
  {
    if (data[i] != 0)
    {
      switch (mapLayer)
      {
        case 0:  // SLAM map
          m_SlamMap.data[i] = value;
          break;
        case 1:  // Kinect Map. apply masking to masking map
        case 2:  // masking map
          m_MaskingMap.data[i] = value;
          break;
      }
    }
  }
}

void MaskingManager::drawLine(std::vector<int> &data, int startX, int startY,
                              int endX, int endY, int value)
{
  // bresenham algorithm
  int x, y, t, dist, xerr, yerr, dx, dy, incx, incy;
  // compute distances
  dx = endX - startX;
  dy = endY - startY;

  // compute increment
  if (dx < 0)
  {
    incx = -1;
    dx = -dx;
  }
  else
  {
    incx = dx ? 1 : 0;
  }

  if (dy < 0)
  {
    incy = -1;
    dy = -dy;
  }
  else
  {
    incy = dy ? 1 : 0;
  }

  // which distance is greater?
  dist = (dx > dy) ? dx : dy;
  // initializing
  x = startX;
  y = startY;
  xerr = dx;
  yerr = dy;

  // compute cells
  for (t = 0; t < dist; t++)
  {
    data[x + m_MaskingMap.info.width * y] = value;

    xerr += dx;
    yerr += dy;
    if (xerr > dist)
    {
      xerr -= dist;
      x += incx;
    }
    if (yerr > dist)
    {
      yerr -= dist;
      y += incy;
    }
  }
}

void MaskingManager::fillPolygon(std::vector<int> &data, int x, int y,
                                 int value)
{
  int index = x + m_MaskingMap.info.width * y;
  if (value != data[index])
  {
    data[index] = value;
    fillPolygon(data, x + 1, y, value);
    fillPolygon(data, x - 1, y, value);
    fillPolygon(data, x, y + 1, value);
    fillPolygon(data, x, y - 1, value);
  }
}
