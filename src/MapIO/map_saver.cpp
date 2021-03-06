/*
 * map_saver
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

#include <homer_map_manager/MapIO/map_saver.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <experimental/filesystem>

#include <tf/tf.h>

using namespace std;

MapGenerator::MapGenerator(const std::string mapname)
{
    m_Mapname = mapname;
}

void MapGenerator::saveMapLayer(const nav_msgs::OccupancyGridConstPtr& map,
                                std::string fileName)
{
    ROS_INFO("Writing map occupancy data to %s", fileName.c_str());
    FILE* out = fopen(fileName.c_str(), "w");
    if (!out)
    {
        ROS_ERROR("Couldn't save map file to %s", fileName.c_str());
        return;
    }
    else if (!map)
    {
        ROS_ERROR("Can't save empty map file to %s", fileName.c_str());
        return;
    }

    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
            map->info.resolution, map->info.width, map->info.height);

    for (unsigned int y = 0; y < map->info.height; y++)
    {
        for (unsigned int x = 0; x < map->info.width; x++)
        {
            unsigned int i = x + (map->info.height - y - 1) * map->info.width;
            if (map->data[i] == -1)
            {
                fputc(205, out);
            }
            else if (map->data[i] < 20)
            {  // occ [0,0.2)
                fputc(254, out);
            }
            else if (map->data[i] > 65)
            {  // occ (0.65,1]
                fputc(000, out);
            }
            else
            {  // occ [0.2,0.65]
                fputc(205, out);
            }
        }
    }
    fclose(out);
}

void MapGenerator::save(
        const nav_msgs::OccupancyGridConstPtr& SLAMMap,
        const nav_msgs::OccupancyGridConstPtr& maskingMap,
        std::vector<homer_mapnav_msgs::PointOfInterest> poiList,
        std::vector<homer_mapnav_msgs::RegionOfInterest> roiList)
{
    std::experimental::filesystem::path directory;
    std::experimental::filesystem::path file;
    directory = std::experimental::filesystem::path(m_Mapname);

    if (!(directory.filename() == "."))
    {
        file = directory / directory.filename();
    }
    else
    {
        auto temp = directory;
        temp.remove_filename();
        file = directory / temp.filename();
    }

    if (!std::experimental::filesystem::is_directory(directory))
    {
        if(!std::experimental::filesystem::create_directories(directory))
        {
            ROS_ERROR("The map could not be saved under requested directory.");
            return;
        }
    }

    std::string SLAMMapdatafile = std::string(file.filename().c_str()) + "_SLAM.pgm";
    std::string maskingMapdatafile = "";
    if (!SLAMMap)
    {
        ROS_ERROR("Can't save empty map file to %s", SLAMMapdatafile.c_str());
        return;
    }
    saveMapLayer(SLAMMap, m_Mapname + "/" + SLAMMapdatafile);
    if (maskingMap != NULL)
    {
        maskingMapdatafile = std::string(file.filename().c_str()) + "_mask.pgm";
        saveMapLayer(maskingMap, m_Mapname + "/" + maskingMapdatafile);
    }
    std::string mapmetadatafile = m_Mapname + "/" + file.filename().c_str() + ".yaml";
    ROS_INFO("Writing map metadata to %s", mapmetadatafile.c_str());
    FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

    /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

   */

    geometry_msgs::Quaternion orientation = SLAMMap->info.origin.orientation;
    // btMatrix3x3 mat(btQuaternion(orientation.x, orientation.y, orientation.z,
    // orientation.w));
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(orientation, quat_tf);
    double yaw = tf::getYaw(quat_tf);
    // mat.getEulerYPR(yaw, pitch, roll);

    stringstream pois;
    if (!poiList.empty())
    {
        pois << "pois:\n";
        std::vector<homer_mapnav_msgs::PointOfInterest>::iterator it;
        for (it = poiList.begin(); it != poiList.end(); it++)
        {
            pois << " - name: " << it->name << "\n";
            pois << "   type: " << it->type << "\n";
            pois << "   x: " << it->pose.position.x << "\n";
            pois << "   y: " << it->pose.position.y << "\n";
            pois << "   theta: " << tf::getYaw(it->pose.orientation) << "\n";
            if (it->remarks == "-")
            {
                pois << "   remarks: "
                     << "\n";
            }
            else
            {
                pois << "   remarks: " << it->remarks << "\n";
            }
            // pois << "   remarks: " << it->remarks << "\n";
        }
    }
    string poiStr = pois.str();

    stringstream rois;
    if (!roiList.empty())
    {
        rois << "rois:\n";
        std::vector<homer_mapnav_msgs::RegionOfInterest>::iterator it;
        for (it = roiList.begin(); it != roiList.end(); it++)
        {
            rois << " - name: " << it->name << "\n";
            rois << "   type: " << it->type << "\n";
            rois << "   x1: " << it->points[0].x << "\n";
            rois << "   y1: " << it->points[0].y << "\n";
            rois << "   x2: " << it->points[1].x << "\n";
            rois << "   y2: " << it->points[1].y << "\n";
            rois << "   x3: " << it->points[2].x << "\n";
            rois << "   y3: " << it->points[2].y << "\n";
            rois << "   x4: " << it->points[3].x << "\n";
            rois << "   y4: " << it->points[3].y << "\n";
            rois << "   id: " << it->id << "\n";
            // rois << "   remarks: " << it->remarks << "\n";
            if (it->remarks == "-")
            {
                rois << "   remarks: "
                     << "\n";
            }
            else
            {
                rois << "   remarks: " << it->remarks << "\n";
            }
        }
    }
    string roiStr = rois.str();

    string maskImage = "";
    if (maskingMapdatafile != "")
        maskImage = "\nmask_image: ";
    fprintf(yaml, "image: %s%s%s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: "
                  "0\noccupied_thresh: 0.65\nfree_thresh: 0.195\n\n%s\n%s",
            SLAMMapdatafile.c_str(), maskImage.c_str(),
            maskingMapdatafile.c_str(), SLAMMap->info.resolution,
            SLAMMap->info.origin.position.x, SLAMMap->info.origin.position.y, yaw,
            poiStr.c_str(), roiStr.c_str());
    fclose(yaml);

    ROS_INFO("Done\n");
}
