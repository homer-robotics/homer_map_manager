#ifndef MAP_MANAGER_NODE_H
#define MAP_MANAGER_NODE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <homer_map_manager/Managers/MapManager.h>
#include <homer_map_manager/Managers/PoiManager.h>
#include <homer_map_manager/Managers/RoiManager.h>
#include <homer_map_manager/Managers/MaskingManager.h>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include <homer_mapnav_msgs/GetPointsOfInterest.h>
#include <homer_mapnav_msgs/GetRegionsOfInterest.h>
#include <homer_mapnav_msgs/GetRegionOfInterestName.h>
#include <homer_mapnav_msgs/PointInsideRois.h>
#include <homer_mapnav_msgs/ModifyPOI.h>
#include <homer_mapnav_msgs/ModifyMap.h>
#include <homer_mapnav_msgs/MapLayers.h>
#include <homer_mapnav_msgs/SaveMap.h>
#include <homer_mapnav_msgs/LoadMap.h>
#include <homer_mapnav_msgs/RoiChange.h>
#include <sensor_msgs/LaserScan.h>

#include <homer_map_manager/MapIO/map_saver.h>
#include <homer_map_manager/MapIO/map_loader.h>
#include <homer_tools/loadRosConfig.h>
#include <homer_nav_libs/tools.h>

/** @class MapManagerNode
  * @author Malte Knauf, Viktor Seib
  * @brief This class handles incoming and outgoing messages and redirects them to the according modules
  */
class MapManagerNode
{
public:
    MapManagerNode(ros::NodeHandle* nh);
    ~MapManagerNode();

private:

    /** callback functions */

    /** callbacks of MapManagerModule */
    void callbackSLAMMap( const nav_msgs::OccupancyGrid::ConstPtr& msg );
    void callbackSaveMap( const std_msgs::String::ConstPtr& msg );
    void callbackLoadMap( const std_msgs::String::ConstPtr& msg );
    void callbackMapVisibility( const homer_mapnav_msgs::MapLayers::ConstPtr& msg );
    void callbackResetMaps( const std_msgs::Empty::ConstPtr& msg );

    /** laser scan callback */
    void callbackLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg );
    void callbackBackLaser( const sensor_msgs::LaserScan::ConstPtr& msg );
    void callbackFrontLaser(const sensor_msgs::LaserScan::ConstPtr& msg );

    /** callbacks of PointOfInterestManagerModule */
    void callbackAddPOI( const homer_mapnav_msgs::PointOfInterest::ConstPtr& msg );
    void callbackModifyPOI( const homer_mapnav_msgs::ModifyPOI::ConstPtr& msg );
    void callbackDeletePOI( const std_msgs::String::ConstPtr& msg );

    /** callbacks of RegionOfInterestManagerModule */
    void callbackAddROI( const homer_mapnav_msgs::RegionOfInterest::ConstPtr& msg );
    void callbackModifyROI( const homer_mapnav_msgs::RegionOfInterest::ConstPtr& msg );
    void callbackDeleteROIbyName( const std_msgs::String::ConstPtr& msg );
    void callbackDeleteROIbyID( const std_msgs::Int32::ConstPtr& msg );

    
    void callbackOctomapMap( const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void callbackRapidMap( const nav_msgs::OccupancyGrid::ConstPtr& msg);

    /** callback of MaskingMapModule */
    void callbackModifyMap( const homer_mapnav_msgs::ModifyMap::ConstPtr& msg );

    /** service of PointOfInterestModule to get a list with all Points Of Interest */
    bool getPOIsService(homer_mapnav_msgs::GetPointsOfInterest::Request& req,
                        homer_mapnav_msgs::GetPointsOfInterest::Response& res);

    /** service of RegionOfInterestModule to get a list with all Regions Of Interest */
    bool getROIsService(homer_mapnav_msgs::GetRegionsOfInterest::Request& req,
                        homer_mapnav_msgs::GetRegionsOfInterest::Response& res);
                        
    bool pointInsideRoisService(homer_mapnav_msgs::PointInsideRois::Request& req,
						homer_mapnav_msgs::PointInsideRois::Response& res);
						
    bool getROINameService(homer_mapnav_msgs::GetRegionOfInterestName::Request& req,
						homer_mapnav_msgs::GetRegionOfInterestName::Response& res);

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);    

	/** Service for saving a map**/
    bool saveMapService(homer_mapnav_msgs::SaveMap::Request& req,
						homer_mapnav_msgs::SaveMap::Response& res);

	/** Service for loading a map**/
    bool loadMapService(homer_mapnav_msgs::LoadMap::Request& req,
						homer_mapnav_msgs::LoadMap::Response& res);

	/** Service resetting the current map**/
    bool resetMapService(std_srvs::Empty::Request& req,
						std_srvs::Empty::Response& res);
    /** modules that are included in this node */
    MapManager* m_MapManager;
    PoiManager* m_POIManager;
    RoiManager* m_ROIManager;
    MaskingManager* m_MaskingManager;

    //subscriptions of MapManagerModule here
    ros::Subscriber m_SLAMMapSubscriber;
    ros::Subscriber m_SaveMapSubscriber;
    ros::Subscriber m_LoadMapSubscriber;
    ros::Subscriber m_MapVisibilitySubscriber;
    ros::Subscriber m_LaserScanSubscriber;
    ros::Subscriber m_BackLaserScanSubscriber;
    ros::Subscriber m_FrontLaserScanSubscriber;

    //subscriptions of PointOfInterestManagerModule
    ros::Subscriber m_AddPOISubscriber;
    ros::Subscriber m_ModifyPOISubscriber;
    ros::Subscriber m_DeletePOISubscriber;

    //subscriptions of RegionOfInterestManagerModule
    ros::Subscriber m_AddROISubscriber;
    ros::Subscriber m_ModifyROISubscriber;
    ros::Subscriber m_DeleteROIByIDSubscriber;
    ros::Subscriber m_DeleteROIByNameSubscriber;
    ros::Subscriber m_PoiInsideROISubscriber;
    ros::Subscriber m_PoseSubscriber;
    ros::Publisher  m_RoiPollPublisher;

    //subscriptions of MaskingMapModule
    ros::Subscriber m_ModifyMapSubscriber;
    ros::Subscriber m_ResetMapsSubscriber;

    /** publisher for loaded maps */
    ros::Publisher m_LoadedMapPublisher;

    /** publisher to mask slam map */
    ros::Publisher m_MaskSlamMapPublisher;

    /** service of PointOfInterstModule */
    ros::ServiceServer m_GetPOIsService;

    /** service of RegionOfInterstModule */
    ros::ServiceServer m_GetROIsService;
	ros::ServiceServer m_GetROINameService;
	ros::ServiceServer m_PoiInsideROISService;

	/** Service for saving the current Map to a file **/
	ros::ServiceServer m_SaveMapService;
	/** Resetting the current map**/
	ros::ServiceServer m_ResetMapService;
	/** Service for loading a previously saved map **/
	ros::ServiceServer m_LoadMapService;
	

	tf::TransformListener m_TransformListener;

    /** timestamp of last incoming laser scan message */
    ros::Time m_LastLaserTime;
    
    ros::Subscriber m_OctomapSubscriber;
    ros::Subscriber m_RapidMapSubscriber;
    
    nav_msgs::OccupancyGrid::ConstPtr m_highSensitiveMap;
    
    bool m_roi_polling;
    std::vector<int> m_ids;
    ros::Time m_lastROIPoll;
    float m_roi_polling_time;
};

#endif // MAP_MANAGER_NODE_H
