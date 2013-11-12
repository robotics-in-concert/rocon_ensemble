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

/*
 * Based on map_store, but addapted to publish/store semantic information (annotations)
behavior:
 - sets up connection to warehouse
 - tells warehouse to publish latest annotations of any session (this is commented by now; can be misleading)
 - spins, handling service calls

service calls:
 - publish_annotations(map uuid) returns true if annotations were found for the given map
   - queries warehouse for annotations associated to the given map
   - publishes the annotations on markers, tables, columns, walls topics
   - publishes visualization markers for all the annotations
   - sets map uuid as the current map, so we can update published annotations if needed
 - rename_annotations(map uuid, new name) returns void
   - renames the associated map identified by map_uuid on annotations database
 - delete_annotations(map uuid) returns true if annotations were found for the given map
   - deletes the annotations associated to the given map
   - if current map is set, calls publish_annotations to reflect changes
 - save_annotations(map uuid, map name, session id) returns error message if any
   - saves currently published annotations as associated to the given map
   - if current map is set, calls publish_annotations to reflect changes

 NOT IMPLEMENTED, and not useful by now
 - list_maps() returns list of map metadata: {id, name, timestamp, maybe thumbnail}
   - query for all annotations.
 - dynamic_map() returns nav_msgs/OccupancyGrid
   - returns the dynamic map
 */

#include <mongo_ros/message_collection.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
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
ros::Publisher visuals_pub;

std::string last_map;
std::string pub_map_id;

visualization_msgs::MarkerArray visuals_array;

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

void clearVisuals()
{
  for (int i = 0; i < visuals_array.markers.size(); ++i)
  {
    visuals_array.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  if (visuals_array.markers.size() > 0)
  {
    visuals_pub.publish(visuals_array);
    visuals_array.markers.clear();
  }
}

visualization_msgs::Marker makeVisual(const std::string& frame, const std::string& name, int id,
                                      int type, double length, double width, double height,
                                      const std_msgs::ColorRGBA& color, const geometry_msgs::Pose& pose)
{
  visualization_msgs::Marker visual, label;

  visual.header.frame_id = frame;
  visual.header.stamp = ros::Time::now();
  visual.scale.x = width;  // scale in meters
  visual.scale.y = length;
  visual.scale.z = height;
  visual.color = color;
  visual.ns = name;
  visual.id = id;
  visual.pose = pose;
  visual.type = type;
  visual.action = visualization_msgs::Marker::ADD;

  // Make z-coordinate the lowest point of visuals, so they appear to lay in the floor if they
  // have zero z. This is wrong for AR markers, but doesn't matter as they are rather small.
  visual.pose.position.z += visual.scale.z/2.0;

  return visual;
}

visualization_msgs::Marker makeLabel(const visualization_msgs::Marker& visual)
{
  visualization_msgs::Marker label = visual;
  label.id = visual.id + 1000000;  // visual id must be unique
  label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  label.pose.position.z = visual.pose.position.z + visual.scale.z/2.0 + 0.1; // just above the visual
  label.text = visual.ns;
  label.scale.x = label.scale.y = label.scale.z = 0.12;
  // label.color.r = label.color.g = label.color.b = 0.0; label.color.a = 1.0; // make solid black

  return label;
}

bool publishAnnotations(annotations_store::PublishAnnotations::Request &request,
                        annotations_store::PublishAnnotations::Response &response)
{
  ROS_INFO("Publish annotations for map '%s'", request.map_uuid.c_str());

//  last_map = request.map_id;
//  ros::NodeHandle nh;
//  nh.setParam("last_map_id", last_map);

  try
  {
    //remove from visualization tools and delete visualization markers
    clearVisuals();

    MarkersVector matching_markers = markers_collection -> pullAllResults(mr::Query("map_uuid", request.map_uuid));
    ColumnsVector matching_columns = columns_collection -> pullAllResults(mr::Query("map_uuid", request.map_uuid));
    WallsVector   matching_walls   = walls_collection   -> pullAllResults(mr::Query("map_uuid", request.map_uuid));
    TablesVector  matching_tables  = tables_collection  -> pullAllResults(mr::Query("map_uuid", request.map_uuid));

    if ((matching_markers.size() == 0) && (matching_tables.size() == 0) &&
        (matching_columns.size() == 0) && (matching_walls.size() == 0))
    {
      ROS_WARN("No annotations found for map '%s'; we don't consider this an error", request.map_uuid.c_str());
      response.found = false;
      return true;  // we don't consider this an error
    }
    response.found = true;

    if (matching_markers.size() > 0)
    {
      std_msgs::ColorRGBA color; color.r = color.b = color.g = color.a = 1.0;
      for (int i = 0; i < matching_markers[0]->markers.size(); ++i)
      {
        ar_msgs::AlvarMarker ann = matching_markers[0]->markers[i];
        std::stringstream name; name << ann.id;
        visuals_array.markers.push_back(makeVisual(ann.pose.header.frame_id, name.str(), i,
                                                   visualization_msgs::Marker::CUBE,
                                                   0.18, 0.18, 0.01,
                                                   color, ann.pose.pose));
        visuals_array.markers.push_back(makeLabel(visuals_array.markers.back()));
      }
      markers_pub.publish(ar_msgs::AlvarMarkersConstPtr(matching_markers [0]));
    }
    if (matching_columns.size() > 0)
    {
      std_msgs::ColorRGBA color; color.b = 1.0; color.a = 0.5;
      for (int i = 0; i < matching_columns[0]->obstacles.size(); ++i)
      {
        yocs_msgs::Column ann = matching_columns[0]->obstacles[i];
        visuals_array.markers.push_back(makeVisual(ann.pose.header.frame_id, ann.name, i,
                                                   visualization_msgs::Marker::CYLINDER,
                                                   ann.radius*2, ann.radius*2, ann.height,
                                                   color, ann.pose.pose.pose));
        visuals_array.markers.push_back(makeLabel(visuals_array.markers.back()));
      }
      columns_pub.publish(yocs_msgs::ColumnListConstPtr(matching_columns [0]));
    }

    if (matching_walls.size() > 0)
    {
      std_msgs::ColorRGBA color; color.r = 0.2; color.g = 0.4; color.b = 0.4; color.a = 0.5;
      for (int i = 0; i < matching_walls[0]->obstacles.size(); ++i)
      {
        yocs_msgs::Wall ann = matching_walls[0]->obstacles[i];
        visuals_array.markers.push_back(makeVisual(ann.pose.header.frame_id, ann.name, i,
                                                   visualization_msgs::Marker::CUBE,
                                                   ann.length, ann.width, ann.height,
                                                   color, ann.pose.pose.pose));
        visuals_array.markers.push_back(makeLabel(visuals_array.markers.back()));
      }
      walls_pub.publish(yocs_msgs::WallListConstPtr(matching_walls[0]));
    }

    if (matching_tables.size() > 0)
    {
      std_msgs::ColorRGBA color; color.r = 0.45; color.g = 0.2; color.a = 0.7;
      for (int i = 0; i < matching_tables[0]->tables.size(); ++i)
      {
        yocs_msgs::Table ann = matching_tables[0]->tables[i];
        visuals_array.markers.push_back(makeVisual(ann.pose.header.frame_id, ann.name, i,
                                                   visualization_msgs::Marker::CYLINDER,
                                                   ann.radius*2, ann.radius*2, ann.height,
                                                   color, ann.pose.pose.pose));
        visuals_array.markers.push_back(makeLabel(visuals_array.markers.back()));
      }
      tables_pub.publish(yocs_msgs::TableListConstPtr(matching_tables[0]));
    }

    // publish visualization markers
    if (visuals_array.markers.size() > 0)
      visuals_pub.publish(visuals_array);

    // Extra sanity checking
    if ((matching_markers.size() == 1) && (matching_tables.size() == 1) &&
        (matching_columns.size() == 1) && (matching_walls.size() == 1))
    {
      ROS_INFO("Annotations fetched: %lu markers, %lu columns, %lu walls, %lu tables",
               matching_markers[0]->markers.size(), matching_columns[0]->obstacles.size(),
               matching_walls[0]->obstacles.size(),matching_tables[0]->tables.size());
    }
    else
    {
      ROS_WARN("Incoherent annotation entries: %lu markers, %lu columns, %lu walls, %lu tables",
               matching_markers.size(), matching_columns.size(), matching_walls.size(),matching_tables.size());
    }

    // Keep track of currently published annotations to republish if we receive updated data
    pub_map_id = request.map_uuid;
    return true;
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
  {
    ROS_WARN("No annotations found for map '%s'; we don't consider this an error", request.map_uuid.c_str());
    response.found = false;
  }
  else
  {
    if (removed != 4)  // this should not happen
      ROS_WARN("Inconsistent number of annotations types (%d)", removed);

    // Check if we are removing currently published annotations to "unpublish" them if so
    if (pub_map_id == request.map_uuid)
    {
      annotations_store::PublishAnnotations::Request pubReq;
      annotations_store::PublishAnnotations::Response pubRes;
      pubReq.map_uuid = request.map_uuid;
      if (publishAnnotations(pubReq, pubRes) == false)
        ROS_WARN("Unpublish removed annotations failed for map '%s'", request.map_uuid.c_str());
    }
  }

  // TODO catch db exception
  return true;
}

bool renameAnnotations(annotations_store::RenameAnnotations::Request &request,
                       annotations_store::RenameAnnotations::Response &response)
{
  markers_collection -> modifyMetadata(mr::Query("map_uuid", request.map_uuid),
                                       mr::Metadata("map_name", request.new_name));
  columns_collection -> modifyMetadata(mr::Query("map_uuid", request.map_uuid),
                                       mr::Metadata("map_name", request.new_name));
  walls_collection   -> modifyMetadata(mr::Query("map_uuid", request.map_uuid),
                                       mr::Metadata("map_name", request.new_name));
  tables_collection  -> modifyMetadata(mr::Query("map_uuid", request.map_uuid),
                                       mr::Metadata("map_name", request.new_name));
  return true;
}


bool saveAnnotations(annotations_store::SaveAnnotations::Request &request,
                     annotations_store::SaveAnnotations::Response &response)
{
// TODO verify that the map exists; I need a version of the removed "lookMap"
//  nav_msgs::GetMap srv;
//  if (!dynamic_map_service_client.call(srv)) {
//    ROS_ERROR("Dynamic map getter service call failed");
//    return false;
//  }

  mr::Metadata metadata = mr::Metadata("map_name",   request.map_name,
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

int main (int argc, char** argv)
{
  ros::init(argc, argv, "annotations_manager");
  ros::NodeHandle nh;

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

  markers_pub = nh.advertise<ar_msgs::AlvarMarkers> ("markers", 1, true);
  columns_pub = nh.advertise<yocs_msgs::ColumnList> ("columns", 1, true);
  walls_pub   = nh.advertise<yocs_msgs::WallList>   ("walls",   1, true);
  tables_pub  = nh.advertise<yocs_msgs::TableList>  ("tables",  1, true);

  visuals_pub = nh.advertise<visualization_msgs::MarkerArray>  ("visual_markers", 1, true);
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

  ros::Subscriber markers_sub = nh.subscribe("markers", 1, onMarkersReceived);
  ros::Subscriber tables_sub  = nh.subscribe("tables",  1, onTablesReceived);
  ros::Subscriber columns_sub = nh.subscribe("columns", 1, onColumnsReceived);
  ros::Subscriber walls_sub   = nh.subscribe("walls",   1, onWallsReceived);

//  ros::ServiceServer list_annotations_srv    = nh.advertiseService("list_annotations",    listAnnotations);
  ros::ServiceServer publish_annotations_srv = nh.advertiseService("publish_annotations", publishAnnotations);
  ros::ServiceServer delete_annotations_srv  = nh.advertiseService("delete_annotations",  deleteAnnotations);
  ros::ServiceServer rename_annotations_srv  = nh.advertiseService("rename_annotations",  renameAnnotations);
  ros::ServiceServer save_annotations_srv    = nh.advertiseService("save_annotations",    saveAnnotations);
//  ros::ServiceServer dynamic_annotations_srv = nh.advertiseService("dynamic_annotations", dynamicMap);

  ROS_DEBUG("Annotations manager running");

  ros::spin();

  delete markers_collection;
  delete columns_collection;
  delete walls_collection;
  delete tables_collection;

  return 0;
}
