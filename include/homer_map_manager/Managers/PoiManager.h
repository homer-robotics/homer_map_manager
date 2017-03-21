#ifndef POI_MANAGER_H
#define POI_MANAGER_H

#include <list>

#include <homer_mapnav_msgs/ModifyPOI.h>
#include <homer_mapnav_msgs/PointOfInterest.h>

#include <ros/ros.h>

/** @class PoiManager
 * @author Malte Knauf, David Gossow
 * @brief This class manages the List of points of interest (POIs)
  *
  * This class keeps a list of all POIs within the current map. It provides the
 * usual functions
  * to edit the list.
  */
class PoiManager
{
public:
  /** The constructor of the class. */
  PoiManager(ros::NodeHandle* nh);

  /** constructor initializing the poi list */
  PoiManager(std::vector<homer_mapnav_msgs::PointOfInterest> pois);

  /** Does nothing. */
  ~PoiManager()
  {
  }

  /** Adds a new POI to the list if no POI with the same name exists
    * @param poi pointer to the PointOfInterest message with the POI to be added
    * @return true if successful, false otherwise
    */
  bool
  addPointOfInterest(const homer_mapnav_msgs::PointOfInterest::ConstPtr& poi);

  /** Replaces a POI with a new one
    * @param poi pointer with the PointOfInterest to be inserted
    *            the POI with the same ID as the new one is first deleted
    * @return true if the old POI was found and could be deleted
    *         false otherwise
    */
  bool modifyPointOfInterest(const homer_mapnav_msgs::ModifyPOI::ConstPtr& poi);

  /** Deletes a POI with a certain name from the list
    * @param name Name of the POI to be deleted
    * @return true if the POI was found and could be deleted
    *         false otherwise
    */
  bool deletePointOfInterest(std::string name);

  /**
   * @brief place the current poi list
   * @param poilist new poi list
   */
  void replacePOIList(std::vector<homer_mapnav_msgs::PointOfInterest> poilist);

  /**
   * @brief Publishes a PointsOfInterest Message with current POIs

   */
  void broadcastPoiList();

  /** Returns current POI list
    * @return the POI list
    */
  std::vector<homer_mapnav_msgs::PointOfInterest> getList();

private:
  /** Looks for POI with name in the list
    * @param name Name of the POI
    */
  bool poiExists(std::string name);

  /** The copy constructor of the class.
    * It's kept private, because it will never be used.
    */
  PoiManager(const PoiManager& instance);

  /** Holds the POI vector */
  std::vector<homer_mapnav_msgs::PointOfInterest> m_Pois;

  /** publisher that publishes the current poi list */
  ros::Publisher m_POIsPublisher;
};
#endif
