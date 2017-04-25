#include <homer_map_manager/map_manager_node.h>
//#include <gperftools/profiler.h>

MapManagerNode::MapManagerNode(ros::NodeHandle* nh)
{
  m_LastLaserTime = ros::Time::now();

  int mapSize = 1;
  float resolution = 1;
  ros::param::param("/map_manager/roi_updates", m_roi_polling, (bool)false);
  ros::param::param("/map_manager/roi_polling_time", m_roi_polling_time,
                    (float)0.5);
  m_lastROIPoll = ros::Time::now();
  m_MapManager = new MapManager(nh);
  m_POIManager = new PoiManager(nh);
  m_ROIManager = new RoiManager(nh);

  nav_msgs::MapMetaData mapInfo;
  mapInfo.width = mapSize / resolution;
  mapInfo.height = mapSize / resolution;
  mapInfo.resolution = resolution;
  mapInfo.origin.position.x = 0;
  mapInfo.origin.position.y = 0;
  mapInfo.origin.position.z = 0;
  mapInfo.origin.orientation.x = 0;
  mapInfo.origin.orientation.y = 0;
  mapInfo.origin.orientation.z = 0;
  mapInfo.origin.orientation.w = 1;

  m_MaskingManager = new MaskingManager(mapInfo);

  m_MapManager->updateMaskingManager(m_MaskingManager);

  // subscriptions of MapManagerModule
  m_RapidMapSubscriber = nh->subscribe("/rapid_mapping/map", 1,
                                       &MapManagerNode::callbackRapidMap, this);
  m_OctomapSubscriber = nh->subscribe(
      "/projected_map", 1, &MapManagerNode::callbackOctomapMap, this);
  m_SLAMMapSubscriber = nh->subscribe("/homer_mapping/slam_map", 1,
                                      &MapManagerNode::callbackSLAMMap, this);
  m_SaveMapSubscriber = nh->subscribe("/map_manager/save_map", 1,
                                      &MapManagerNode::callbackSaveMap, this);
  m_LoadMapSubscriber = nh->subscribe("/map_manager/load_map", 1,
                                      &MapManagerNode::callbackLoadMap, this);
  m_MapVisibilitySubscriber =
      nh->subscribe("/map_manager/toggle_map_visibility", 1,
                    &MapManagerNode::callbackMapVisibility, this);
  m_PoseSubscriber =
      nh->subscribe("/pose", 1, &MapManagerNode::poseCallback, this);

  // subscriptions of PoiManager
  m_AddPOISubscriber = nh->subscribe("/map_manager/add_POI", 20,
                                     &MapManagerNode::callbackAddPOI, this);
  m_ModifyPOISubscriber = nh->subscribe(
      "/map_manager/modify_POI", 100, &MapManagerNode::callbackModifyPOI, this);
  m_DeletePOISubscriber = nh->subscribe(
      "/map_manager/delete_POI", 100, &MapManagerNode::callbackDeletePOI, this);
  m_GetPOIsService = nh->advertiseService(
      "/map_manager/get_pois", &MapManagerNode::getPOIsService, this);

  // Services for Map Handling
  m_SaveMapService = nh->advertiseService(
      "/map_manager/save_map", &MapManagerNode::saveMapService, this);
  m_ResetMapService = nh->advertiseService(
      "/map_manager/reset_map", &MapManagerNode::resetMapService, this);
  m_LoadMapService = nh->advertiseService(
      "/map_manager/load_map", &MapManagerNode::loadMapService, this);

  // subscriptions of RoiManager
  m_AddROISubscriber = nh->subscribe("/map_manager/add_ROI", 20,
                                     &MapManagerNode::callbackAddROI, this);
  m_ModifyROISubscriber = nh->subscribe(
      "/map_manager/modify_ROI", 100, &MapManagerNode::callbackModifyROI, this);
  m_DeleteROIByNameSubscriber =
      nh->subscribe("/map_manager/delete_ROI_by_name", 100,
                    &MapManagerNode::callbackDeleteROIbyName, this);
  m_DeleteROIByIDSubscriber =
      nh->subscribe("/map_manager/delete_ROI_by_id", 100,
                    &MapManagerNode::callbackDeleteROIbyID, this);
  m_GetROIsService = nh->advertiseService(
      "/map_manager/get_rois", &MapManagerNode::getROIsService, this);
  m_GetROINameService = nh->advertiseService(
      "/map_manager/get_roi_name", &MapManagerNode::getROINameService, this);
  m_PoiInsideROISService =
      nh->advertiseService("/map_manager/point_inside_rois",
                           &MapManagerNode::pointInsideRoisService, this);
  if (m_roi_polling)
  {
    m_RoiPollPublisher = nh->advertise<homer_mapnav_msgs::RoiChange>(
        "/map_manager/roi_change", 1);
  }

  // subscribtions of MaskingMapModule
  m_ModifyMapSubscriber = nh->subscribe(
      "/map_manager/modify_map", 1, &MapManagerNode::callbackModifyMap, this);
  m_ResetMapsSubscriber = nh->subscribe(
      "/map_manager/reset_maps", 1, &MapManagerNode::callbackResetMaps, this);

  // loaded map publisher
  m_LoadedMapPublisher =
      nh->advertise<nav_msgs::OccupancyGrid>("/map_manager/loaded_map", 1);

  // mask slam publisher
  m_MaskSlamMapPublisher =
      nh->advertise<nav_msgs::OccupancyGrid>("/map_manager/mask_slam", 1);

  // load map file from config if present
  std::string mapfile;
  ros::param::get("/map_manager/default_mapfile", mapfile);
  if (mapfile != "")
  {
    std_msgs::String::Ptr mapfileMsg(new std_msgs::String);
    mapfileMsg->data = mapfile;
    callbackLoadMap(mapfileMsg);
  }
  m_POIManager->broadcastPoiList();
  m_ROIManager->broadcastRoiList();
}

MapManagerNode::~MapManagerNode()
{
  if (m_MapManager)
    delete m_MapManager;
  if (m_POIManager)
    delete m_POIManager;
  if (m_ROIManager)
    delete m_ROIManager;
  if (m_MaskingManager)
    delete m_MaskingManager;
}

void MapManagerNode::callbackSLAMMap(
    const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  nav_msgs::OccupancyGrid::ConstPtr maskingMap =
      m_MaskingManager->updateMapInfo(msg->info);
  m_MapManager->updateMapLayer(homer_mapnav_msgs::MapLayers::MASKING_LAYER,
                               maskingMap);
  m_MapManager->updateMapLayer(homer_mapnav_msgs::MapLayers::SLAM_LAYER, msg);
}

void MapManagerNode::callbackRapidMap(
    const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  m_MapManager->updateMapLayer(homer_mapnav_msgs::MapLayers::RAPID_MAP, msg);
}

void MapManagerNode::callbackOctomapMap(
    const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  m_MapManager->updateMapLayer(homer_mapnav_msgs::MapLayers::KINECT_LAYER, msg);

  //	nav_msgs::OccupancyGrid mergedMap;
  //
  //   	int width = 701;
  //    int height = 701;
  //    float resolution = 0.05;
  //
  //    int ByteSize = width * height;
  //
  //    mergedMap.header.stamp = ros::Time::now();
  //    mergedMap.header.frame_id = "map";
  //    mergedMap.info.width = width;
  //    mergedMap.info.height = height;
  //    mergedMap.info.resolution = resolution;
  //    mergedMap.info.origin.position.x = -height*resolution/2;
  //    mergedMap.info.origin.position.y = -width*resolution/2;
  //    mergedMap.info.origin.orientation.w = 1.0;
  //    mergedMap.info.origin.orientation.x = 0.0;
  //    mergedMap.info.origin.orientation.y = 0.0;
  //    mergedMap.info.origin.orientation.z = 0.0;
  //  	mergedMap.data.resize(ByteSize,-1);

  //	  for ( int y = 0; y < msg->info.height; y++ )
  //	  {
  //		for ( int x = 0; x < msg->info.width; x++ )
  //		{
  //		  	int i = x + y * msg->info.width;

  //
  //			//if cell is occupied by kinect obstacle merge cell with merged
  // map
  //			if(msg->data[i] ==
  // homer_mapnav_msgs::ModifyMap::BLOCKED)
  //			{

  //				Eigen::Vector2i point(x,y);
  //				geometry_msgs::Point tmp = map_tools::fromMapCoords( point
  //,msg->info.origin, msg->info.resolution);
  //				point = map_tools::toMapCoords(tmp , mergedMap.info.origin,
  // mergedMap.info.resolution);
  //				int k = point.y() * mergedMap.info.width +
  // point.x();
  //				mergedMap.data[k] =
  // homer_mapnav_msgs::ModifyMap::DEPTH;
  //			}
  //		}
  //	  }
  //
  //
  //	m_MapManager->updateMapLayer(homer_mapnav_msgs::MapLayers::KINECT_LAYER,
  // boost::make_shared<nav_msgs::OccupancyGrid>(mergedMap));
}

void MapManagerNode::callbackSaveMap(const std_msgs::String::ConstPtr& msg)
{
  MapGenerator map_saver(msg->data);
  nav_msgs::OccupancyGrid::ConstPtr SLAMMap =
      m_MapManager->getMapLayer(homer_mapnav_msgs::MapLayers::SLAM_LAYER);
  nav_msgs::OccupancyGrid::ConstPtr maskingMap =
      m_MapManager->getMapLayer(homer_mapnav_msgs::MapLayers::MASKING_LAYER);
  map_saver.save(SLAMMap, maskingMap, m_POIManager->getList(),
                 m_ROIManager->getList());
}

void MapManagerNode::callbackLoadMap(const std_msgs::String::ConstPtr& msg)
{
  m_MapManager->clearMapLayers();
  ROS_INFO_STREAM("Loading map from file " << msg->data);
  bool success;
  MapServer map_loader(msg->data, success);
  if (success)
  {
    ros::Rate poll_rate(10);
    while (m_LoadedMapPublisher.getNumSubscribers() == 0)
    {
      poll_rate.sleep();
    }
    m_LoadedMapPublisher.publish(map_loader.getSLAMMap());
    m_MapManager->updateMapLayer(
        homer_mapnav_msgs::MapLayers::SLAM_LAYER,
        boost::make_shared<nav_msgs::OccupancyGrid>(map_loader.getSLAMMap()));
    nav_msgs::OccupancyGrid::ConstPtr maskingMap =
        boost::make_shared<nav_msgs::OccupancyGrid>(map_loader.getMaskingMap());
    m_MaskingManager->replaceMap(map_loader.getMaskingMap());
    if (maskingMap->data.size() != 0)
    {
      m_MapManager->updateMapLayer(homer_mapnav_msgs::MapLayers::MASKING_LAYER,
                                   maskingMap);
    }
    m_POIManager->replacePOIList(map_loader.getPois());
    m_POIManager->broadcastPoiList();
    m_ROIManager->replaceROIList(map_loader.getRois());
    m_ROIManager->broadcastRoiList();
  }
  else
  {
    ROS_ERROR_STREAM("[MapManager] Could not open mapfile!!");
  }
}

void MapManagerNode::callbackAddPOI(
    const homer_mapnav_msgs::PointOfInterest::ConstPtr& msg)
{
  ROS_INFO_STREAM("callbackAddPOI");
  m_POIManager->addPointOfInterest(msg);
}

void MapManagerNode::callbackModifyPOI(
    const homer_mapnav_msgs::ModifyPOI::ConstPtr& msg)
{
  m_POIManager->modifyPointOfInterest(msg);
}

void MapManagerNode::callbackDeletePOI(const std_msgs::String::ConstPtr& msg)
{
  m_POIManager->deletePointOfInterest(msg->data);
}

void MapManagerNode::callbackAddROI(
    const homer_mapnav_msgs::RegionOfInterest::ConstPtr& msg)
{
  m_ROIManager->addRegionOfInterest(msg);
}

void MapManagerNode::callbackModifyROI(
    const homer_mapnav_msgs::RegionOfInterest::ConstPtr& msg)
{
  m_ROIManager->modifyRegionOfInterest(msg);
}

void MapManagerNode::callbackDeleteROIbyName(
    const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("Recieved /map_manager/delete_ROI_by_name");
  if (m_ROIManager->deleteRegionOfInterest(msg->data))
  {
    ROS_INFO_STREAM("ROI deleted.");
  }
  else
  {
    ROS_WARN_STREAM("Could not delete ROI.");
  }
}

void MapManagerNode::callbackDeleteROIbyID(const std_msgs::Int32::ConstPtr& msg)
{
  m_ROIManager->deleteRegionOfInterest(msg->data);
}

void MapManagerNode::callbackMapVisibility(
    const homer_mapnav_msgs::MapLayers::ConstPtr& msg)
{
  m_MapManager->toggleMapVisibility(msg->layer, msg->state);
}

void MapManagerNode::callbackModifyMap(
    const homer_mapnav_msgs::ModifyMap::ConstPtr& msg)
{
  nav_msgs::OccupancyGrid::ConstPtr maskingMap =
      m_MaskingManager->modifyMap(msg);
  if (msg->mapLayer == homer_mapnav_msgs::MapLayers::SLAM_LAYER ||
      msg->maskAction == homer_mapnav_msgs::ModifyMap::HIGH_SENSITIV)
  {
    m_MaskSlamMapPublisher.publish(maskingMap);
    m_highSensitiveMap = maskingMap;
  }
  else
  {
    m_MapManager->updateMapLayer(homer_mapnav_msgs::MapLayers::MASKING_LAYER,
                                 maskingMap);
  }
}

void MapManagerNode::callbackResetMaps(const std_msgs::Empty::ConstPtr& msg)
{
  nav_msgs::OccupancyGrid::ConstPtr maskingMap = m_MaskingManager->resetMap();
  m_MapManager->updateMapLayer(homer_mapnav_msgs::MapLayers::MASKING_LAYER,
                               maskingMap);
}

bool MapManagerNode::getPOIsService(
    homer_mapnav_msgs::GetPointsOfInterest::Request& req,
    homer_mapnav_msgs::GetPointsOfInterest::Response& res)
{
  res.poi_list.pois = m_POIManager->getList();
  return true;
}

bool MapManagerNode::getROIsService(
    homer_mapnav_msgs::GetRegionsOfInterest::Request& req,
    homer_mapnav_msgs::GetRegionsOfInterest::Response& res)
{
  res.roi_list.rois = m_ROIManager->getList();
  return true;
}

bool MapManagerNode::pointInsideRoisService(
    homer_mapnav_msgs::PointInsideRois::Request& req,
    homer_mapnav_msgs::PointInsideRois::Response& res)
{
  res.rois = m_ROIManager->pointInsideRegionOfInterest(req.point);
  return true;
}

bool MapManagerNode::getROINameService(
    homer_mapnav_msgs::GetRegionOfInterestName::Request& req,
    homer_mapnav_msgs::GetRegionOfInterestName::Response& res)
{
  res.name = m_ROIManager->getROIName(req.roi_id);
  return true;
}

void MapManagerNode::poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //m_MapManager->updatePose(msg);
  if (msg->header.stamp - m_lastROIPoll > ros::Duration(m_roi_polling_time) &&
      m_roi_polling)
  {
    m_lastROIPoll = msg->header.stamp;
    geometry_msgs::PointStamped posePoint;
    posePoint.header.frame_id = "/map";
    posePoint.header.stamp = msg->header.stamp;
    posePoint.point = msg->pose.position;
    std::vector<homer_mapnav_msgs::RegionOfInterest> rois;
    rois = m_ROIManager->pointInsideRegionOfInterest(posePoint);
    bool found = false;
    for (int i = 0; i < m_ids.size(); i++)
    {
      found = false;
      for (int j = 0; j < rois.size(); j++)
      {
        if (m_ids[i] == rois[j].id)
        {
          rois.erase(rois.begin() + j);
          found = true;
          break;
        }
      }
      if (!found)
      {
        homer_mapnav_msgs::RoiChange change;
        change.id = m_ids[i];
        change.name = m_ROIManager->getROIName(m_ids[i]);
        change.action = false;
        m_RoiPollPublisher.publish(change);
        m_ids.erase(m_ids.begin() + i);
        i--;
      }
    }
    for (int i = 0; i < rois.size(); i++)
    {
      homer_mapnav_msgs::RoiChange change;
      change.id = rois[i].id;
      change.name = m_ROIManager->getROIName(change.id);
      change.action = true;
      m_RoiPollPublisher.publish(change);
      m_ids.push_back(rois[i].id);
    }
  }
}

bool MapManagerNode::saveMapService(homer_mapnav_msgs::SaveMap::Request& req,
                                    homer_mapnav_msgs::SaveMap::Response& res)
{
  ROS_INFO_STREAM("Saving map " << req.folder.data);
  MapGenerator map_saver(std::string(req.folder.data));
  nav_msgs::OccupancyGrid::ConstPtr SLAMMap =
      m_MapManager->getMapLayer(homer_mapnav_msgs::MapLayers::SLAM_LAYER);
  nav_msgs::OccupancyGrid::ConstPtr maskingMap =
      m_MapManager->getMapLayer(homer_mapnav_msgs::MapLayers::MASKING_LAYER);
  map_saver.save(SLAMMap, maskingMap, m_POIManager->getList(),
                 m_ROIManager->getList());
  return true;
}

bool MapManagerNode::loadMapService(homer_mapnav_msgs::LoadMap::Request& req,
                                    homer_mapnav_msgs::LoadMap::Response& res)
{
  // load map file from config if present
  std::string mapfile = std::string(req.filename.data);
  if (mapfile != "")
  {
    ROS_INFO_STREAM("Loading map with filename: " << mapfile);
    std_msgs::String::Ptr mapfileMsg(new std_msgs::String);
    mapfileMsg->data = mapfile;
    callbackLoadMap(mapfileMsg);
  }
  else
  {
    ROS_ERROR_STREAM("Map filename is empty. Could not load map");
  }
}

bool MapManagerNode::resetMapService(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  ROS_INFO_STREAM("Resetting current map");
  nav_msgs::OccupancyGrid::ConstPtr maskingMap = m_MaskingManager->resetMap();
  m_MapManager->updateMapLayer(homer_mapnav_msgs::MapLayers::MASKING_LAYER,
                               maskingMap);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_manager");
  ros::NodeHandle nh;

  /*    char* pprofile    = std::getenv("MAPMANAGER_PROFILE");
      if (pprofile)
      {
          ProfilerStart(pprofile);
      }
  */

  MapManagerNode node(&nh);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    try
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    catch (exception& e)
    {
      std::cout << "Exception in main loop" << e.what() << std::endl;
    }
  }
  /*    if (pprofile)
      {
          ProfilerStop();
      }
  */
  return 0;
}
