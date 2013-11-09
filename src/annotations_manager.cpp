/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

/*
behavior:
 - sets up connection to warehouse
 - tells warehouse to publish latest map of any session (or default map?  or nothing?)
 - spins, handling service calls

service calls:
 - list_maps() returns list of map metadata: {id, name, timestamp, maybe thumbnail}
   - query for all maps.
 - delete_map(map id) returns void
   - Deletes the given map
 - rename_map[(map id, name) returns void
   - renames a given map
 - publish_map(map id) returns void
   - queries warehouse for map of given id
   - publishes the map on /map
   - sets dynamic map up to load it\
 - dynamic_map() returns nav_msgs/OccupancyGrid
   - returns the dynamic map
 */

#include <mongo_ros/message_collection.h>
#include <ros/ros.h>
#include <annotations_store/ListAnnotations.h>
#include <annotations_store/PublishAnnotations.h>
#include <annotations_store/DeleteAnnotations.h>
#include <annotations_store/RenameAnnotations.h>
#include <annotations_store/SaveAnnotations.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <yocs_msgs/ColumnList.h>
#include <yocs_msgs/WallList.h>
#include <yocs_msgs/TableList.h>

#include <string>
#include <sstream>
#include <exception>
namespace mr=mongo_ros;
namespace ar_msgs=ar_track_alvar;

mr::MessageCollection<ar_msgs::AlvarMarkers> *markers_collection;
mr::MessageCollection<yocs_msgs::ColumnList> *columns_collection;
mr::MessageCollection<yocs_msgs::WallList>   *walls_collection;
mr::MessageCollection<yocs_msgs::TableList>  *tables_collection;

ros::Publisher markers_pub;
ros::Publisher tables_pub;
ros::Publisher columns_pub;
ros::Publisher walls_pub;

std::string last_map;

ar_msgs::AlvarMarkers markers_msg;
yocs_msgs::ColumnList columns_msg;
yocs_msgs::WallList   walls_msg;
yocs_msgs::TableList  tables_msg;

typedef std::vector<mr::MessageWithMetadata<ar_msgs::AlvarMarkers>::ConstPtr> MarkersVector;
typedef std::vector<mr::MessageWithMetadata<yocs_msgs::ColumnList>::ConstPtr> ColumnsVector;
typedef std::vector<mr::MessageWithMetadata<yocs_msgs::WallList>::ConstPtr>   WallsVector;
typedef std::vector<mr::MessageWithMetadata<yocs_msgs::TableList>::ConstPtr>  TablesVector;


void onMarkersReceived(const ar_msgs::AlvarMarkers::ConstPtr& msg)
{
  markers_msg = *msg;
}

void onColumnsReceived(const yocs_msgs::ColumnList::ConstPtr& msg)
{
  columns_msg = *msg;
}

void onWallsReceived(const yocs_msgs::WallList::ConstPtr& msg)
{
  walls_msg = *msg;
}

void onTablesReceived(const yocs_msgs::TableList::ConstPtr& msg)
{
  tables_msg = *msg;
}


//bool listAnnotations(annotations_store::ListAnnotations::Request &request,
//                     annotations_store::ListAnnotations::Response &response)
//{
//  ROS_DEBUG("listAnnotations() service call");
//
//  MapVector all_maps;
//  all_maps = map_collection->pullAllResults( mr::Query(), true, "creation_time", false );
//
//  // Loop over all_maps to get the first of each session.
//  for(MapVector::const_iterator map_iter = all_maps.begin(); map_iter != all_maps.end(); map_iter++)
//  {
//    ROS_DEBUG("listAnnotations() reading a map");
//
//    ROS_DEBUG("listAnnotations() adding a map to the result list.");
//    ROS_DEBUG("listAnnotations() metadata is: '%s'", (*map_iter)->metadata.toString().c_str());
//
//    // Add the map info to our result list.
//    annotations_store::MapListEntry new_entry;
//    new_entry.name = (*map_iter)->lookupString("name");
//    new_entry.date = (int64_t)(*map_iter)->lookupDouble("creation_time");
//    new_entry.session_id = (*map_iter)->lookupString("session_id");
//    new_entry.map_id = (*map_iter)->lookupString("uuid");
//
//    response.map_list.push_back(new_entry);
//  }
//
//  ROS_DEBUG("listAnnotations() service call done");
//  return true;
//}

//bool lookupMap(std::string name, nav_msgs::OccupancyGridConstPtr &ptr) {
//  MapVector matching_maps;
//  try
//  {
//    matching_maps = map_collection->pullAllResults(mr::Query("uuid", name), false );
//  } catch(const std::exception &e) {
//    ROS_ERROR("Error during query: %s", e.what());
//    return false;
//  }
//
//  if (matching_maps.size() != 1)
//  {
//    ROS_ERROR("publishAnnotations() found %d matching maps instead of 1.  Failing.", (int) matching_maps.size());
//    return false;
//  }
//  ptr = nav_msgs::OccupancyGridConstPtr( matching_maps[0] );
//  return true;
//}

bool publishAnnotations(annotations_store::PublishAnnotations::Request &request,
                        annotations_store::PublishAnnotations::Response &response)
{
  ROS_INFO("Publish annotations for map '%s'", request.map_uuid.c_str());

//  last_map = request.map_id;
//  ros::NodeHandle nh;
//  nh.setParam("last_map_id", last_map);
//  ar_msgs::AlvarMarkers markers;
//  yocs_msgs::ColumnList columns;
//  yocs_msgs::WallList   walls;
//  yocs_msgs::TableList  tables;

  try
  {
    MarkersVector matching_markers = markers_collection -> pullAllResults(mr::Query("map_uuid", request.map_uuid));
    ColumnsVector matching_columns = columns_collection -> pullAllResults(mr::Query("map_uuid", request.map_uuid));
    WallsVector   matching_walls   = walls_collection   -> pullAllResults(mr::Query("map_uuid", request.map_uuid));
    TablesVector  matching_tables  = tables_collection  -> pullAllResults(mr::Query("map_uuid", request.map_uuid));

    // TODO publish visualization markers
    if (matching_markers.size() > 0) markers_pub.publish(ar_msgs::AlvarMarkersConstPtr (matching_markers [0]));
    if (matching_columns.size() > 0) columns_pub.publish(yocs_msgs::ColumnListConstPtr (matching_columns [0]));
    if (matching_walls.size()   > 0)   walls_pub.publish(yocs_msgs::WallListConstPtr   (matching_walls   [0]));
    if (matching_tables.size()  > 0)  tables_pub.publish(yocs_msgs::TableListConstPtr  (matching_tables  [0]));

    ROS_INFO("Annotations fetched: %lu markers, %lu columns, %lu walls, %lu tables",
             matching_markers.size(), matching_columns.size(), matching_walls.size(),matching_tables.size());
  }
  catch(const std::exception &e) {
    ROS_ERROR("Error during query: %s", e.what());
    return false;
  }

  return true;
}

bool deleteAnnotations(annotations_store::DeleteAnnotations::Request &request,
	               annotations_store::DeleteAnnotations::Response &response)
{
//  ros::NodeHandle nh;
//  std::string param;
//  if (nh.getParam("last_map_id", param))
//  {
//    if (param == request.map_id)
//    {
//      nh.deleteParam("last_map_id");
//    }
//  }
//  if (last_map == request.map_id)
//  {
//    last_map = "";
//  }
  ROS_INFO("Delete annotations for map '%s'", request.map_uuid.c_str());
  int removed = 0;
  removed += markers_collection -> removeMessages(mr::Query("map_uuid", request.map_uuid));
  removed += columns_collection -> removeMessages(mr::Query("map_uuid", request.map_uuid));
  removed += walls_collection   -> removeMessages(mr::Query("map_uuid", request.map_uuid));
  removed += tables_collection  -> removeMessages(mr::Query("map_uuid", request.map_uuid));

  if (removed == 0)
    ROS_WARN("No annotations found for map '%s')", request.map_uuid.c_str());
  else if (removed != 4)
    ROS_WARN("Inconsistent number of annotations types (%d)", removed);

  return (removed > 0);
}
//
//bool renameAnnotations(annotations_store::RenameAnnotations::Request &request,
//                       annotations_store::RenameAnnotations::Response &response)
//{
//  map_collection->modifyMetadata(mr::Query("uuid", request.map_id), mr::Metadata("name", request.new_name));
//  return true;
//}
//
//
//bool dynamicMap(nav_msgs::GetMap::Request &request,
//		nav_msgs::GetMap::Response &response) {
//  if (last_map == "") {
//    return false;
//  }
//  nav_msgs::OccupancyGridConstPtr map;
//  if (lookupMap(last_map, map))
//  {
//    response.map = *map;
//  }
//  else
//  {
//    return false;
//  }
//  return true;
//}

bool saveAnnotations(annotations_store::SaveAnnotations::Request &request,
                     annotations_store::SaveAnnotations::Response &response)
{
// TODO verify that the map exists
//  nav_msgs::GetMap srv;
//  if (!dynamic_map_service_client.call(srv)) {
//    ROS_ERROR("Dynamic map getter service call failed");
//    return false;
//  }

//  std::string uuid_string = uuidGenerate();
  mr::Metadata metadata = mr::Metadata("map_name",   request.map_name, // TODO try renaming to map_uuid and map_name if all ok
                                       "map_uuid",   request.map_uuid,
                                       "session_id", request.session_id);

  ROS_INFO("Saving annotations for map %s with uuid %s", request.map_name.c_str(), request.map_uuid.c_str());
  try
  {
    markers_collection -> removeMessages(mr::Query("map_uuid", request.map_uuid));
    columns_collection -> removeMessages(mr::Query("map_uuid", request.map_uuid));
    walls_collection   -> removeMessages(mr::Query("map_uuid", request.map_uuid));
    tables_collection  -> removeMessages(mr::Query("map_uuid", request.map_uuid));

    markers_collection -> insert(markers_msg, metadata);
    columns_collection -> insert(columns_msg, metadata);
    walls_collection   -> insert(walls_msg,   metadata);
    tables_collection  -> insert(tables_msg,  metadata);

    ROS_INFO("Annotations saved: %lu markers, %lu columns, %lu walls, %lu tables",
             markers_msg.markers.size(), columns_msg.obstacles.size(),
             walls_msg.obstacles.size(), tables_msg.tables.size());

    return true;
  }
  catch (mongo::DBException& e)
  {
    ROS_ERROR("Error during saving: %s", e.what());
    response.error_msg = e.what();
    return false;
  }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "annotations_manager");
  ros::NodeHandle nh;

//  map_collection = new mr::MessageCollection<nav_msgs::OccupancyGrid>("annotations_store", "maps");
//  map_collection->ensureIndex("uuid");

  markers_collection = new mr::MessageCollection<ar_msgs::AlvarMarkers> ("annotations_store", "markers");
  columns_collection = new mr::MessageCollection<yocs_msgs::ColumnList> ("annotations_store", "columns");
  walls_collection   = new mr::MessageCollection<yocs_msgs::WallList>   ("annotations_store", "walls");
  tables_collection  = new mr::MessageCollection<yocs_msgs::TableList>  ("annotations_store", "tables");

  markers_collection -> ensureIndex("map_uuid");
  columns_collection -> ensureIndex("map_uuid");
  walls_collection   -> ensureIndex("map_uuid");
  tables_collection  -> ensureIndex("map_uuid");


//  if (!nh.getParam("last_map_id", last_map))   TODO I don't think it make sense to pub last annotations
//  {
//    last_map = "";
//  }

  markers_pub = nh.advertise<ar_msgs::AlvarMarkers> ("annotations/markers", 1, true);
  columns_pub = nh.advertise<yocs_msgs::ColumnList> ("annotations/columns", 1, true);
  walls_pub   = nh.advertise<yocs_msgs::WallList>   ("annotations/walls",   1, true);
  tables_pub  = nh.advertise<yocs_msgs::TableList>  ("annotations/tables",  1, true);
//  if (last_map != "")
//  {
//    nav_msgs::OccupancyGridConstPtr map;
//    if (lookupMap(last_map, map))
//    {
//      try {
//	map_publisher.publish(map);
//      } catch(...) {
//	ROS_ERROR("Error publishing map");
//      }
//    }
//    else
//    {
//      ROS_ERROR("Invalid last_map_id");
//    }
//  }

  ros::Subscriber markers_sub = nh.subscribe("annotations/markers", 1, onMarkersReceived);
  ros::Subscriber tables_sub  = nh.subscribe("annotations/tables",  1, onTablesReceived);
  ros::Subscriber columns_sub = nh.subscribe("annotations/columns", 1, onColumnsReceived);
  ros::Subscriber walls_sub   = nh.subscribe("annotations/walls",   1, onWallsReceived);

//  ros::ServiceServer list_annotations_srv    = nh.advertiseService("list_annotations",    listAnnotations);
  ros::ServiceServer publish_annotations_srv = nh.advertiseService("publish_annotations", publishAnnotations);
  ros::ServiceServer delete_annotations_srv  = nh.advertiseService("delete_annotations",  deleteAnnotations);
//  ros::ServiceServer rename_annotations_srv  = nh.advertiseService("rename_annotations",  renameAnnotations);
  ros::ServiceServer save_annotations_srv    = nh.advertiseService("save_annotations",    saveAnnotations);
//  ros::ServiceServer dynamic_annotations_srv = nh.advertiseService("dynamic_annotations", dynamicMap);

  ROS_DEBUG("Annotations manager running");

  ros::spin();

  delete markers_collection;

  return 0;
}
