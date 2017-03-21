/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include <homer_map_manager/MapIO/image_loader.h>
#include "nav_msgs/MapMetaData.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "yaml-cpp/yaml.h"

#include <homer_map_manager/MapIO/map_loader.h>

/** Trivial constructor */
MapServer::MapServer(const std::string fname, bool& success)
{
  success = false;
  std::string slammapfname = "";
  std::string maskingmapfname = "";
  double origin[3];
  int negate;
  double res, occ_th, free_th;
  std::string frame_id;
  frame_id = "map";
  // mapfname = fname + ".pgm";
  // std::ifstream fin((fname + ".yaml").c_str());
  std::ifstream fin(fname.c_str());
  if (fin.fail())
  {
    ROS_ERROR("Map_server could not open %s.", fname.c_str());
    return;
  }

  YAML::Node doc = YAML::LoadFile(fname);

  try
  {
    res = doc["resolution"].as<double>();
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
    return;
  }
  try
  {
    negate = doc["negate"].as<int>();
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a negate tag or it is invalid.");
    return;
  }
  try
  {
    occ_th = doc["occupied_thresh"].as<double>();
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an occupied_thresh tag or it is "
              "invalid.");
    return;
  }
  try
  {
    free_th = doc["free_thresh"].as<double>();
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
    return;
  }
  try
  {
    origin[0] = doc["origin"][0].as<double>();
    origin[1] = doc["origin"][1].as<double>();
    origin[2] = doc["origin"][2].as<double>();
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an origin tag or it is invalid.");
    return;
  }
  try
  {
    slammapfname = doc["image"].as<std::string>();
    // TODO: make this path-handling more robust
    if (slammapfname.size() == 0)
    {
      ROS_ERROR("The image tag cannot be an empty string.");
      return;
    }
    if (slammapfname[0] != '/')
    {
      // dirname can modify what you pass it
      char* fname_copy = strdup(fname.c_str());
      slammapfname = std::string(dirname(fname_copy)) + '/' + slammapfname;
      free(fname_copy);
    }
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an image tag or it is invalid.");
    return;
  }
  // get masking map image path if available
  if (doc["mask_image"])
  {
    maskingmapfname = doc["mask_image"].as<std::string>();
    // TODO: make this path-handling more robust
    if (maskingmapfname.size() == 0)
    {
      ROS_ERROR("The image tag cannot be an empty string.");
      return;
    }
    if (maskingmapfname[0] != '/')
    {
      //              // dirname can modify what you pass it
      char* fname_copy = strdup(fname.c_str());
      maskingmapfname =
          std::string(dirname(fname_copy)) + '/' + maskingmapfname;
      free(fname_copy);
    }
  }

  // get POIs if existent
  if (doc["pois"])
  {
    ROS_INFO_STREAM("Found " << doc["pois"].size() << " pois");
    for (size_t i = 0; i < doc["pois"].size(); ++i)
    {
      ROS_INFO_STREAM("load one poi." << i);
      std::string name;
      int type;
      float posX;
      float posY;
      float theta;
      std::string remarks;
      name = doc["pois"][i]["name"].as<std::string>();
      type = doc["pois"][i]["type"].as<int>();
      posX = doc["pois"][i]["x"].as<double>();
      posY = doc["pois"][i]["y"].as<double>();
      theta = doc["pois"][i]["theta"].as<double>();
      remarks = doc["pois"][i]["remarks"].as<std::string>("");

      homer_mapnav_msgs::PointOfInterest poi;
      poi.name = name;
      poi.type = type;
      poi.pose.position.x = posX;
      poi.pose.position.y = posY;
      poi.pose.position.z = 0.0;
      poi.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      poi.remarks = remarks;
      ROS_INFO_STREAM("Done, Saving in mapnav_msgs");

      poiList.push_back(poi);
      ROS_INFO_STREAM("loaded one poi");
    }
    ROS_INFO_STREAM("Done. Loaded all POIs.");
  }

  // get ROIs if existent
  if (doc["rois"])
  {
    ROS_INFO_STREAM("Found " << doc["rois"].size() << " rois");
    for (size_t i = 0; i < doc["rois"].size(); ++i)
    {
      std::string name;
      int type;
      float posX1;
      float posY1;
      float posX2;
      float posY2;
      float posX3;
      float posY3;
      float posX4;
      float posY4;
      int id;
      std::string remarks;

      // Read from file
      name = doc["rois"][i]["name"].as<std::string>();
      type = doc["rois"][i]["type"].as<int>();
      posX1 = doc["rois"][i]["x1"].as<double>();
      posY1 = doc["rois"][i]["y1"].as<double>();
      posX2 = doc["rois"][i]["x2"].as<double>();
      posY2 = doc["rois"][i]["y2"].as<double>();
      posX3 = doc["rois"][i]["x3"].as<double>();
      posY3 = doc["rois"][i]["y3"].as<double>();
      posX4 = doc["rois"][i]["x4"].as<double>();
      posY4 = doc["rois"][i]["y4"].as<double>();
      id = doc["rois"][i]["id"].as<int>();
      remarks = doc["rois"][i]["remarks"].as<std::string>("-");

      // save in roi-type
      homer_mapnav_msgs::RegionOfInterest roi;
      roi.name = name;
      roi.type = type;
      roi.points[0].x = posX1;
      roi.points[0].y = posY1;
      roi.points[0].z = 0.0;
      roi.points[1].x = posX2;
      roi.points[1].y = posY2;
      roi.points[1].z = 0.0;
      roi.points[2].x = posX3;
      roi.points[2].y = posY3;
      roi.points[2].z = 0.0;
      roi.points[3].x = posX4;
      roi.points[3].y = posY4;
      roi.points[3].z = 0.0;
      roi.id = id;
      roi.remarks = remarks;

      roiList.push_back(roi);
    }
  }

  ROS_INFO("Loading SLAM map from image \"%s\"", slammapfname.c_str());
  map_server::loadMapFromFile(&m_SLAMMap, slammapfname.c_str(), res, negate,
                              occ_th, free_th, origin);
  m_SLAMMap.info.map_load_time = ros::Time::now();
  m_SLAMMap.header.frame_id = frame_id;
  m_SLAMMap.header.stamp = ros::Time::now();
  ROS_INFO("Read a %d X %d SLAM map @ %.3lf m/cell", m_SLAMMap.info.width,
           m_SLAMMap.info.height, m_SLAMMap.info.resolution);

  if (maskingmapfname != "")
  {
    ROS_INFO("Loading masking map from image \"%s\"", maskingmapfname.c_str());
    map_server::loadMapFromFile(&m_MaskingMap, maskingmapfname.c_str(), res,
                                negate, occ_th, free_th, origin);
    m_MaskingMap.info.map_load_time = ros::Time::now();
    m_MaskingMap.header.frame_id = frame_id;
    m_MaskingMap.header.stamp = ros::Time::now();
    ROS_INFO("Read a %d X %d masking map @ %.3lf m/cell",
             m_MaskingMap.info.width, m_MaskingMap.info.height,
             m_MaskingMap.info.resolution);
  }
  success = true;
}

nav_msgs::OccupancyGrid MapServer::getSLAMMap()
{
  return m_SLAMMap;
}

nav_msgs::OccupancyGrid MapServer::getMaskingMap()
{
  return m_MaskingMap;
}

std::vector<homer_mapnav_msgs::PointOfInterest> MapServer::getPois()
{
  return poiList;
}

std::vector<homer_mapnav_msgs::RegionOfInterest> MapServer::getRois()
{
  return roiList;
}
