#include <homer_map_manager/Managers/PoiManager.h>

#include <sstream>

#include <homer_mapnav_msgs/PointsOfInterest.h>

#include <ros/ros.h>

using namespace std;


PoiManager::PoiManager(ros::NodeHandle *nh)
{
    m_POIsPublisher = nh->advertise<homer_mapnav_msgs::PointsOfInterest>("/map_manager/poi_list", 1);
    m_Pois.clear();
}

PoiManager::PoiManager ( std::vector<homer_mapnav_msgs::PointOfInterest> pois )
{
  //copy POIs
  m_Pois = pois;
}

std::vector<homer_mapnav_msgs::PointOfInterest> PoiManager::getList()
{
  return m_Pois;
}

bool PoiManager::addPointOfInterest (const homer_mapnav_msgs::PointOfInterest::ConstPtr &poi )
{
    //make sure there's no POI with the same name

    if ( poiExists ( poi->name ) )
    {
      ostringstream stream;
      stream << "Poi with name " << poi->name << " already exists! Doing nothing.";
      ROS_WARN_STREAM ( stream.str() );
      return false;
    }

    //copy poi & assigning new id
    homer_mapnav_msgs::PointOfInterest new_poi= *poi;

    ROS_INFO_STREAM ("Adding POI '" << new_poi.name << "'.");

    //insert into list
    m_Pois.push_back ( new_poi );

    broadcastPoiList();
    return true;
}

bool PoiManager::modifyPointOfInterest (const homer_mapnav_msgs::ModifyPOI::ConstPtr &poi )
{
  std::string name = poi->old_name;

  std::vector<homer_mapnav_msgs::PointOfInterest>::iterator it;

  for ( it=m_Pois.begin() ; it != m_Pois.end(); it++ )
  {
    if ( it->name == name )
    {
      *it=poi->poi;
      broadcastPoiList();
      return true;
    }
  }

  ROS_ERROR_STREAM ( "Cannot modify: POI does not exist!" );

  return false;
}

void PoiManager::replacePOIList(std::vector<homer_mapnav_msgs::PointOfInterest> poilist)
{
    m_Pois = poilist;
    broadcastPoiList();
}

bool PoiManager::poiExists ( std::string name )
{
  std::vector<homer_mapnav_msgs::PointOfInterest>::iterator it;

  for ( it=m_Pois.begin() ; it != m_Pois.end(); it++ )
  {
    if ( it->name == name )
    {
      return true;
    }
  }

  return false;
}

bool PoiManager::deletePointOfInterest (std::string name )
{
  std::vector< homer_mapnav_msgs::PointOfInterest >::iterator it;

  for ( it=m_Pois.begin() ; it != m_Pois.end(); ) //it++ ) Iterator darf nur dann erhöht werden, wenn kein POI gelöscht wird.
  {
    if ( it->name == name )
    {
      ROS_INFO_STREAM ("Erasing POI " << name << ".");

      // ! Achtung !
      // Wenn letztes Element gelöscht wird, wird it auf .end() gesetzt
      // der nachfolgende Aufruf von it++ führt zu einem Fehler.
      // Daher Iterator nur erhöhen, wenn kein erase durchgeführt wurde.

      it = m_Pois.erase ( it );
      broadcastPoiList();
      return true;
    }else{
      it++;
    }
  }

  ROS_ERROR_STREAM ("POI " << name << " does not exist.");

  return false;
}

void PoiManager::broadcastPoiList() {
  ostringstream stream;
  //print the current list
  std::vector< homer_mapnav_msgs::PointOfInterest >::iterator it;
  stream << "Contents of POI list:\n";
  homer_mapnav_msgs::PointsOfInterest poiMsg;
  for ( it = m_Pois.begin(); it != m_Pois.end(); it++ ) {
    stream << "    POI " << it->name << "', " << it->type
           << ", (" << it->pose.position.x << "," << it->pose.position.y << "), '" << it->remarks << "'\n";
  }
  poiMsg.pois = m_Pois;
  ros::Rate poll_rate(10);
  m_POIsPublisher.publish(poiMsg);
  ROS_DEBUG_STREAM( stream.str() );
}

