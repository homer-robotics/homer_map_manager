#ifndef ROI_MANAGER_H
#define ROI_MANAGER_H

#include <geometry_msgs/PointStamped.h>
#include <homer_mapnav_msgs/RegionOfInterest.h>
#include <homer_mapnav_msgs/RegionsOfInterest.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <sstream>

/**
 * @class RoiManager (Roi = Region of interest)
 * @author Viktor Seib (R23)
 * @brief This class manages the List of regions of interest (ROIs)
  *
  * This class keeps a list of all ROIs within the current map. It provides the
 * usual functions
  * to edit the list.
  */
class RoiManager
{
public:
  /** The constructor of the class. */
  RoiManager(ros::NodeHandle* nh);

  /** constructor initializing the roi list */
  RoiManager(std::vector<homer_mapnav_msgs::RegionOfInterest> rois);

  /** Does nothing. */
  ~RoiManager()
  {
  }

  /** Adds a new ROI to the list, in contrast to POIs, several ROIs with the
   * same name are allowed
    * @param roi RegionOfInterest message with the ROI to be added
    * @return true if successful, false otherwise
    */
  bool
  addRegionOfInterest(const homer_mapnav_msgs::RegionOfInterest::ConstPtr& roi);

  /** Tests all Rois and returns a vector of ids in which the pose is
    * @param pose Pose which is tested to be inside the Rois
    * @return vector of ids in which the pose is
    */
  std::vector<homer_mapnav_msgs::RegionOfInterest>
  pointInsideRegionOfInterest(const geometry_msgs::PointStamped point);

  /** Replaces a ROI with a new one
    * @param roi RegionOfInterest to be inserted
    *            the ROI with the same ID as the new one is first deleted
    * @return true if the old ROI was found and could be deleted
    *         false otherwise
    */
  bool modifyRegionOfInterest(
      const homer_mapnav_msgs::RegionOfInterest::ConstPtr& roi);

  /** Deletes all ROIs with a certain name from the list
    * @param name Name of the ROIs to be deleted
    * @return true if the ROI was found and could be deleted
    *         false otherwise
    */
  bool deleteRegionOfInterest(std::string name);

  /** Deltes ROI with the given id
   * @param id ID of ROI to be deleted
   * @return true if the ROI was found and could be deleted
   *         false otherwise
   */
  bool deleteRegionOfInterest(int id);

  /**
   * @brief place the current roi list
   * @param roilist new roi list
   */
  void replaceROIList(std::vector<homer_mapnav_msgs::RegionOfInterest> roilist);

  /** Returns current ROI list
    * @return the ROI list
    */
  std::vector<homer_mapnav_msgs::RegionOfInterest> getList();

  /**
   * @brief Publishes a RegionsOfInterest Message with current ROIs

   */
  void broadcastRoiList();

  /** Returns name of ROI
    * @param id of ROI
    * @return ROI name
    */
  std::string getROIName(int id);

private:
  /** Looks for ROI with name in the list
    * @param id ID of the ROI
    */
  bool roiExists(int id);

  /** Looks for ROI with name in the list
    * @param name Name of the ROI
    */
  bool roiExists(std::string name);

  /**
    * @brief gets the highest id of all ROIs and save it into m_highest_id
    */
  void setHighestId();

  /** The copy constructor of the class.
    * It's kept private, because it will never be used.
    */
  RoiManager(const RoiManager& instance);

  /** Holds the ROI vector */
  std::vector<homer_mapnav_msgs::RegionOfInterest> m_Rois;

  /** publisher that publishes the current roi list */
  ros::Publisher m_ROIsPublisher;

  /** to set a new id, the current highest id is needed */
  int m_highest_id;

  tf::TransformListener tf_listener;
};
#endif
