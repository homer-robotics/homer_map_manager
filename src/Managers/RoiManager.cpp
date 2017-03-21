#include <homer_map_manager/Managers/RoiManager.h>

RoiManager::RoiManager(ros::NodeHandle *nh)
{
  m_ROIsPublisher = nh->advertise<homer_mapnav_msgs::RegionsOfInterest>(
      "/map_manager/roi_list", 1, true);
  m_highest_id = 0;  // start with 0 so first ROI gets the 1
  m_Rois.clear();
}

RoiManager::RoiManager(std::vector<homer_mapnav_msgs::RegionOfInterest> rois)
{
  m_Rois = rois;
}

std::vector<homer_mapnav_msgs::RegionOfInterest> RoiManager::getList()
{
  return m_Rois;
}

std::string RoiManager::getROIName(int id)
{
  if (roiExists(id))
  {
    std::vector<homer_mapnav_msgs::RegionOfInterest>::iterator it;
    for (it = m_Rois.begin(); it != m_Rois.end(); it++)
    {
      if (it->id == id)
      {
        return it->name;
      }
    }
    return "";
  }
  else
  {
    return "";
  }
}

std::vector<homer_mapnav_msgs::RegionOfInterest>
RoiManager::pointInsideRegionOfInterest(const geometry_msgs::PointStamped point)
{
  std::vector<homer_mapnav_msgs::RegionOfInterest> rois;
  geometry_msgs::PointStamped mpoint;
  tf_listener.waitForTransform("/map", point.header.frame_id, ros::Time::now(),
                               ros::Duration(5.0));
  tf_listener.transformPoint("/map", point, mpoint);
  std::vector<homer_mapnav_msgs::RegionOfInterest>::iterator it;
  bool inside = false;
  for (it = m_Rois.begin(); it != m_Rois.end(); it++)
  {
    inside = false;
    int i, j;
    // code idea from
    // http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
    j = it->points.size() - 1;
    for (i = 0; i < it->points.size(); i++)
    {
      if (((it->points[i].y > mpoint.point.y) !=
           (it->points[j].y > mpoint.point.y)) &&
          (mpoint.point.x < (it->points[j].x - it->points[i].x) *
                                    (float)(mpoint.point.y - it->points[i].y) /
                                    (it->points[j].y - it->points[i].y) +
                                it->points[i].x))
      {
        inside = !inside;
      }
      j = i;
    }
    if (inside)
    {
      rois.push_back(*it);
    }
  }
  return rois;
}

bool RoiManager::addRegionOfInterest(
    const homer_mapnav_msgs::RegionOfInterest::ConstPtr &roi)
{
  ROS_INFO_STREAM("Recieved new roi.");
  if (roiExists(roi->id))
  {
    ROS_INFO_STREAM("id exists");
    std::ostringstream stream;
    stream << "Roi with ID " << roi->id << " (name: " << roi->name
           << ") already exists! Modifiying Roi.";
    ROS_WARN_STREAM(stream.str());
    return modifyRegionOfInterest(roi);
  }
  else
  {
    ROS_INFO_STREAM("new id");
    // copy roi & assigning new id
    homer_mapnav_msgs::RegionOfInterest new_roi = *roi;
    new_roi.id = (m_highest_id + 1);

    ROS_INFO_STREAM("Adding ROI '" << new_roi.name << "' with ID " << new_roi.id
                                   << ".");

    // insert into list
    m_Rois.push_back(new_roi);

    setHighestId();
    broadcastRoiList();
    return true;
  }
}

bool RoiManager::modifyRegionOfInterest(
    const homer_mapnav_msgs::RegionOfInterest::ConstPtr &roi)
{
  std::vector<homer_mapnav_msgs::RegionOfInterest>::iterator it;

  for (it = m_Rois.begin(); it != m_Rois.end(); it++)
  {
    if (it->id == roi->id)
    {
      *it = *roi;
      setHighestId();
      broadcastRoiList();
      return true;
    }
  }

  ROS_ERROR_STREAM("Cannot modify: ROI does not exist!");

  return false;
}

void RoiManager::replaceROIList(
    std::vector<homer_mapnav_msgs::RegionOfInterest> roilist)
{
  m_Rois = roilist;
  setHighestId();
  broadcastRoiList();
}

bool RoiManager::roiExists(int id)
{
  ROS_INFO_STREAM("ID: " << id);
  ROS_INFO_STREAM("roi exists?");
  ROS_INFO_STREAM("Number Rois: ");
  ROS_INFO_STREAM(m_Rois.size());
  if (m_Rois.size() != 0)
  {
    std::vector<homer_mapnav_msgs::RegionOfInterest>::iterator it;

    for (it = m_Rois.begin(); it != m_Rois.end(); it++)
    {
      if (it->id == id)
      {
        return true;
      }
    }
  }
  ROS_INFO_STREAM("Return false");
  return false;
}

bool RoiManager::roiExists(std::string name)
{
  ROS_INFO_STREAM("name: " << name);
  ROS_INFO_STREAM("roi exists?");
  ROS_INFO_STREAM("Number Rois: ");
  ROS_INFO_STREAM(m_Rois.size());
  if (m_Rois.size() != 0)
  {
    std::vector<homer_mapnav_msgs::RegionOfInterest>::iterator it;

    for (it = m_Rois.begin(); it != m_Rois.end(); it++)
    {
      if (it->name == name)
      {
        return true;
      }
    }
  }
  ROS_INFO_STREAM("Return false");
  return false;
}

bool RoiManager::deleteRegionOfInterest(std::string name)
{
  ROS_INFO_STREAM("Delete Roi with name: " << name);
  std::vector<homer_mapnav_msgs::RegionOfInterest>::iterator it;

  bool modified = false;
  for (it = m_Rois.begin(); it != m_Rois.end();)
  {
    if (it->name == name)
    {
      ROS_INFO_STREAM("Erasing all ROIs with name " << name << ".");
      it = m_Rois.erase(it);
      modified = true;
    }
    else
    {
      it++;
    }
  }

  if (modified)
  {
    ROS_INFO_STREAM("modified");
    broadcastRoiList();
    return true;
  }

  ROS_ERROR_STREAM("ROI " << name << " does not exist.");

  return false;
}

bool RoiManager::deleteRegionOfInterest(int id)
{
  std::vector<homer_mapnav_msgs::RegionOfInterest>::iterator it;

  for (it = m_Rois.begin(); it != m_Rois.end(); it++)
  {
    if (it->id == id)
    {
      ROS_INFO_STREAM("Erasing ROI with ID " << id << ".");
      it = m_Rois.erase(it);
      broadcastRoiList();
      return true;
    }
  }

  ROS_ERROR_STREAM("ROI with ID " << id << " does not exist.");

  return false;
}

void RoiManager::broadcastRoiList()
{
  std::ostringstream stream;
  ROS_INFO_STREAM("Broadcast ROI.");
  // print the current list
  std::vector<homer_mapnav_msgs::RegionOfInterest>::iterator it;
  stream << "Contents of ROI list:\n";
  homer_mapnav_msgs::RegionsOfInterest roiMsg;
  for (it = m_Rois.begin(); it != m_Rois.end(); it++)
  {
    stream << "    ROI '" << it->name << "', '" << it->type << "', [ "
           << it->points[0].x << " " << it->points[0].y << " ; "
           << " " << it->points[1].x << " " << it->points[1].y << " ; "
           << " " << it->points[2].x << " " << it->points[2].y << " ; "
           << " " << it->points[3].x << " " << it->points[3].y << " ], '"
           << it->remarks << "'\n";
  }
  //  ROS_INFO_STREAM( stream.str() );
  roiMsg.rois = m_Rois;
  ROS_INFO_STREAM("roiMsg.rois");
  m_ROIsPublisher.publish(roiMsg);
  ROS_DEBUG_STREAM(stream.str());
}

void RoiManager::setHighestId()
{
  ROS_INFO_STREAM("Set global variable highest_id.");
  ROS_INFO_STREAM("Find highest id of all ROIs.");
  ROS_INFO_STREAM("current highest id: " << m_highest_id);
  std::vector<homer_mapnav_msgs::RegionOfInterest>::iterator it;
  for (it = m_Rois.begin(); it != m_Rois.end(); it++)
  {
    ROS_INFO_STREAM("Roi: " << it->name << ", " << it->id);
    if (it->id >= m_highest_id)
    {
      m_highest_id = it->id;
      ROS_INFO_STREAM("set new highest id: " << m_highest_id);
    }
  }
}
