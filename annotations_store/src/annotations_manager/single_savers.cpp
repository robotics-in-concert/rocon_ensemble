/*
 * Copyright (c) 2013, Yujin Robot.
 *
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

#include "annotations_store/annotations_manager.hpp"

namespace yocs {

  bool AnnotationsManager::saveMarkers(annotations_store::SaveMarkers::Request &request,
                   annotations_store::SaveMarkers::Response &response)
  {
    ROS_INFO("Save markers for map '%s'", request.map_uuid.c_str());
  
    try
    {
      //remove from visualization tools and delete visualization markers
      clearVisuals();
  
      mr::Metadata metadata = mr::Metadata("map_name",   "Created from files",
                                           "map_uuid",   request.map_uuid,
                                           "session_id", "");
  
      markers_collection->removeMessages(mr::Query("map_uuid", request.map_uuid));
      markers_collection->insert(request.markers, metadata);
      response.found = true;
  
      ROS_INFO("Markers saved: %lu markers", request.markers.markers.size());
  
      // Check if we are modifying currently published annotations to republish them if so
      if (pub_map_id == request.map_uuid)
      {
        annotations_store::PublishAnnotations::Request pubReq;
        annotations_store::PublishAnnotations::Response pubRes;
        pubReq.map_uuid = request.map_uuid;
        if (publishAnnotations(pubReq, pubRes) == false)
          ROS_WARN("Republish modified annotations failed for map '%s'", request.map_uuid.c_str());
      }
      return true;
    }
    catch (mongo::DBException& e)
    {
      ROS_ERROR("Error during saving: %s", e.what());
      response.error_msg = e.what();
      return false;
    }
  }
  
  bool AnnotationsManager::saveTables(annotations_store::SaveTables::Request &request,
                  annotations_store::SaveTables::Response &response)
  {
    ROS_INFO("Save tables for map '%s'", request.map_uuid.c_str());
  
    try
    {
      //remove from visualization tools and delete visualization tables
      clearVisuals();
  
      mr::Metadata metadata = mr::Metadata("map_name",   "Created from files",
                                           "map_uuid",   request.map_uuid,
                                           "session_id", "");
  
      tables_collection->removeMessages(mr::Query("map_uuid", request.map_uuid));
      tables_collection->insert(request.tables, metadata);
      response.found = true;
  
      ROS_INFO("Tables saved: %lu tables", request.tables.tables.size());
  
      // Check if we are modifying currently published annotations to republish them if so
      if (pub_map_id == request.map_uuid)
      {
        annotations_store::PublishAnnotations::Request pubReq;
        annotations_store::PublishAnnotations::Response pubRes;
        pubReq.map_uuid = request.map_uuid;
        if (publishAnnotations(pubReq, pubRes) == false)
          ROS_WARN("Republish modified annotations failed for map '%s'", request.map_uuid.c_str());
      }
      return true;
    }
    catch (mongo::DBException& e)
    {
      ROS_ERROR("Error during saving: %s", e.what());
      response.error_msg = e.what();
      return false;
    }
  }
  
  bool AnnotationsManager::saveColumns(annotations_store::SaveColumns::Request &request,
                   annotations_store::SaveColumns::Response &response)
  {
    ROS_INFO("Save columns for map '%s'", request.map_uuid.c_str());
  
    try
    {
      //remove from visualization tools and delete visualization columns
      clearVisuals();
  
      mr::Metadata metadata = mr::Metadata("map_name",   "Created from files",
                                           "map_uuid",   request.map_uuid,
                                           "session_id", "");
  
      columns_collection->removeMessages(mr::Query("map_uuid", request.map_uuid));
      columns_collection->insert(request.columns, metadata);
      response.found = true;
  
      ROS_INFO("Columns saved: %lu columns", request.columns.obstacles.size());
  
      // Check if we are modifying currently published annotations to republish them if so
      if (pub_map_id == request.map_uuid)
      {
        annotations_store::PublishAnnotations::Request pubReq;
        annotations_store::PublishAnnotations::Response pubRes;
        pubReq.map_uuid = request.map_uuid;
        if (publishAnnotations(pubReq, pubRes) == false)
          ROS_WARN("Republish modified annotations failed for map '%s'", request.map_uuid.c_str());
      }
      return true;
    }
    catch (mongo::DBException& e)
    {
      ROS_ERROR("Error during saving: %s", e.what());
      response.error_msg = e.what();
      return false;
    }
  }
  
  bool AnnotationsManager::saveWalls(annotations_store::SaveWalls::Request &request,
                 annotations_store::SaveWalls::Response &response)
  {
    ROS_INFO("Save walls for map '%s'", request.map_uuid.c_str());
  
    try
    {
      //remove from visualization tools and delete visualization walls
      clearVisuals();
  
      mr::Metadata metadata = mr::Metadata("map_name",   "Created from files",
                                           "map_uuid",   request.map_uuid,
                                           "session_id", "");
  
      walls_collection->removeMessages(mr::Query("map_uuid", request.map_uuid));
      walls_collection->insert(request.walls, metadata);
  
      ROS_INFO("Walls saved: %lu walls", request.walls.obstacles.size());
  
      // Check if we are modifying currently published annotations to republish them if so
      if (pub_map_id == request.map_uuid)
      {
        annotations_store::PublishAnnotations::Request pubReq;
        annotations_store::PublishAnnotations::Response pubRes;
        pubReq.map_uuid = request.map_uuid;
        if (publishAnnotations(pubReq, pubRes) == false)
          ROS_WARN("Republish modified annotations failed for map '%s'", request.map_uuid.c_str());
      }
      return true;
    }
    catch (mongo::DBException& e)
    {
      ROS_ERROR("Error during saving: %s", e.what());
      response.error_msg = e.what();
      return false;
    }
  }
}
