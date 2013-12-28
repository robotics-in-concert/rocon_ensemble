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
#ifndef _ANNOTATIONS_MANAGER_HPP_
#define _ANNOTATIONS_MANAGER_HPP_

#include <ros/ros.h>
#include <mongo_ros/message_collection.h>
#include <visualization_msgs/MarkerArray.h>
#include <annotations_store/ListAnnotations.h>
#include <annotations_store/PublishAnnotations.h>
#include <annotations_store/DeleteAnnotations.h>
#include <annotations_store/RenameAnnotations.h>
#include <annotations_store/SaveAnnotations.h>
#include <annotations_store/SaveMarkers.h>
#include <annotations_store/SaveTables.h>
#include <annotations_store/SaveColumns.h>
#include <annotations_store/SaveWalls.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <yocs_msgs/ColumnList.h>
#include <yocs_msgs/WallList.h>
#include <yocs_msgs/TableList.h>

#include <string>
#include <sstream>
#include <exception>

namespace mr=mongo_ros;
namespace ar_msgs=ar_track_alvar;

namespace yocs {

class AnnotationsManager {
  typedef std::vector<mr::MessageWithMetadata<ar_msgs::AlvarMarkers>::ConstPtr> MarkersVector;
  typedef std::vector<mr::MessageWithMetadata<yocs_msgs::ColumnList>::ConstPtr> ColumnsVector;
  typedef std::vector<mr::MessageWithMetadata<yocs_msgs::WallList>::ConstPtr>   WallsVector;
  typedef std::vector<mr::MessageWithMetadata<yocs_msgs::TableList>::ConstPtr>  TablesVector;

  public:
    AnnotationsManager();
    ~AnnotationsManager();
  protected:
    // annotation subscribers
    void onMarkersReceived(const ar_msgs::AlvarMarkers::ConstPtr& msg);
    void onColumnsReceived(const yocs_msgs::ColumnList::ConstPtr& msg);
    void onWallsReceived(const yocs_msgs::WallList::ConstPtr& msg);
    void onTablesReceived(const yocs_msgs::TableList::ConstPtr& msg);

    visualization_msgs::Marker makeVisual(const std::string& frame, const std::string& name, int id, int type, double length, double width, double height, const std_msgs::ColorRGBA& color, const geometry_msgs::Pose& pose);
    visualization_msgs::Marker makeLabel(const visualization_msgs::Marker& visual);
    void clearVisuals();

    // annotation handle services
    bool publishAnnotations(annotations_store::PublishAnnotations::Request &request, annotations_store::PublishAnnotations::Response &response);
    bool deleteAnnotations(annotations_store::DeleteAnnotations::Request &request, annotations_store::DeleteAnnotations::Response &response);
    bool renameAnnotations(annotations_store::RenameAnnotations::Request &request, annotations_store::RenameAnnotations::Response &response);
    bool saveAnnotations(annotations_store::SaveAnnotations::Request &request, annotations_store::SaveAnnotations::Response &response);

    // single saves
    bool saveMarkers(annotations_store::SaveMarkers::Request &request, annotations_store::SaveMarkers::Response &response);
    bool saveTables(annotations_store::SaveTables::Request &request, annotations_store::SaveTables::Response &response);
    bool saveColumns(annotations_store::SaveColumns::Request &request, annotations_store::SaveColumns::Response &response);
    bool saveWalls(annotations_store::SaveWalls::Request &request, annotations_store::SaveWalls::Response &response);

  private:
    ros::NodeHandle nh;

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

    ros::ServiceServer save_markers_srv;
    ros::ServiceServer save_tables_srv ;
    ros::ServiceServer save_columns_srv;
    ros::ServiceServer save_walls_srv  ;
    ros::ServiceServer publish_annotations_srv; 
    ros::ServiceServer delete_annotations_srv;
    ros::ServiceServer rename_annotations_srv;  
    ros::ServiceServer save_annotations_srv;    

    ros::Subscriber markers_sub;
    ros::Subscriber tables_sub;
    ros::Subscriber columns_sub;
    ros::Subscriber walls_sub;



};


}

#endif // _ANNOTATIONS_MANAGER_HPP_
