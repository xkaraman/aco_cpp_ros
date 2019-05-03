/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <random>

#include <QLabel>
#include <QVBoxLayout>
#include <QListWidget>
#include <QPushButton>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/service.h>

#include <nav_msgs/GetPlan.h>


#include <visualization_msgs/MarkerArray.h>

#include "myviz.h"
#include "ACO.cpp"

namespace turtlebot_move{
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : rviz::Panel( parent )
  //, nh()
{
  mWaypoints.reserve(30);
  // Construct and lay out labels, lists and buttons
 // TODO Add Buttons (ADD,REMOVE,EDIT)
  QVBoxLayout *topic_layout = new QVBoxLayout;
  topic_layout->addWidget(new QLabel("Current Points"));
  dropdown_list = new QListWidget;
  topic_layout->addWidget(dropdown_list);

  mAddPushButton = new QPushButton("&Add",this);
  mRemovePushButton = new QPushButton("&Remove",this);
  mCalculatePathsPushButton = new QPushButton("Calculate Paths", this);
  mEditACOParamButton = new QPushButton("Edit ACO",this);
  mRunACOButton = new QPushButton("Run ACO",this);

  // Connect Remove to callback slot
  connect(mRemovePushButton,&QPushButton::clicked,this,&MyViz::removePoint );
  connect(mCalculatePathsPushButton,&QPushButton::clicked,this,&MyViz::calculatePaths);
  connect(mEditACOParamButton,&QPushButton::clicked,this,&MyViz::editACOParam);
  connect(mRunACOButton,&QPushButton::clicked,this,&MyViz::runACO);

  // mAddPushButton = new QPushButton("Add",this);
  QHBoxLayout *edit_points_layout = new QHBoxLayout;
  edit_points_layout->addWidget(mAddPushButton);
  edit_points_layout->addWidget(mRemovePushButton);

  QHBoxLayout *aco_edit_layout = new QHBoxLayout;
  aco_edit_layout->addWidget(mEditACOParamButton);
  aco_edit_layout->addWidget(mRunACOButton);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(edit_points_layout);
  layout->addWidget(mCalculatePathsPushButton);
  layout->addLayout(aco_edit_layout);
  setLayout(layout);

  mPointSub = nh.subscribe("clicked_point",100,&MyViz::pointReceived,this);
  mPathPub = nh.advertise<visualization_msgs::MarkerArray>("/turtlebot_move/all_paths",10);
  mBestPathPub = nh.advertise<visualization_msgs::MarkerArray>("/turtlebot_move/best_path",10);

}

// Destructor.
MyViz::~MyViz()
{
  // delete manager_;
}

void MyViz::pointReceived(const geometry_msgs::PointStamped &msg){
  ROS_INFO("I heard");
  std::string name;
  name = "Point at (x,y) = (" + std::to_string(msg.point.x) + "," + std::to_string(msg.point.y) + ")";
  dropdown_list->addItem(QString(name.c_str()));
  mWaypoints.push_back(msg);

  // std::cout << "### Points in List ###" << '\n';
  // for (size_t i = 0; i < mWaypoints.size(); i++) {
  //   std::cout << mWaypoints[i];
  //   }
}

void MyViz::removePoint(){
  int index = dropdown_list->currentRow();
  std::cout << "Removing Point at : " << index << '\n';
  if(!mWaypoints.empty() && index!=-1){
    dropdown_list->takeItem(index);
    mWaypoints.erase(mWaypoints.begin() + index);
  }
  std::cout << "### Removed ###" << '\n';
  // std::cout << "### Points in List ###" << '\n';
  // for (size_t i = 0; i < mWaypoints.size(); i++) {
  //   std::cout << mWaypoints[i];
  // }
}

void MyViz::calculatePaths(){
  ros::service::waitForService("/move_base/make_plan");
  std::cout << "### Calculating Paths Cost ###" << '\n';
  mPaths.clear();
  mPathsCost.clear();
  mPathsCost.reserve(mWaypoints.size());

  for (size_t i = 0; i < mWaypoints.size(); i++) {
    mPathsCost.push_back({});
    mPathsCost[i].reserve(mWaypoints.size());
    for (size_t j = 0; j < mWaypoints.size(); j++) {
      if(j>i){
        std_msgs::Header hdr;
        hdr.stamp = ros::Time::now();
        hdr.frame_id = "map";

        geometry_msgs::PoseStamped start,stop;
        geometry_msgs::Pose p;
        geometry_msgs::Quaternion q;
        start.header = hdr;
        p.position = mWaypoints[i].point;
        p.orientation = q;
        start.pose = p;

        stop.header = hdr;
        p.position = mWaypoints[j].point;
        stop.pose = p;

        nav_msgs::GetPlan srv;
        srv.request.start = start;
        srv.request.goal = stop;
        srv.request.tolerance = 0.2;
        // std::cout << "Calck path i to j " << i << " "<< j << '\n';
        if (ros::service::call("/move_base/make_plan",srv) ){
            double cost;
            cost = calc_path_cost(srv.response.plan.poses);
            // std::cout << "Pushback" << '\n';
            mPaths.push_back(srv.response.plan);
            mPathsCost[i].push_back(cost);
          }
      } else {
          mPathsCost[i].push_back(0.0);
      }
    }
  }

  std::cout << "### Done. Total Paths : " << mPaths.size() << " ###" << '\n';

  visualization_msgs::MarkerArray all_paths;
  visualization_msgs::Marker pathMarker;
  all_paths.markers.clear();
  mPathPub.publish(all_paths);
  pathMarker.header.frame_id = "map";
  pathMarker.header.stamp = ros::Time::now();
  pathMarker.ns = "lines";
  pathMarker.action = visualization_msgs::Marker::ADD;
  pathMarker.pose.orientation.w = 1.0;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(0, 1);
  for (size_t i = 0; i < mPaths.size(); i++) {
    pathMarker.points.clear();
    pathMarker.id = i;
    pathMarker.type = visualization_msgs::Marker::LINE_STRIP;
    pathMarker.scale.x = 0.05;
    pathMarker.color.r = dis(gen);
    pathMarker.color.g = dis(gen);
    pathMarker.color.b = dis(gen);
    pathMarker.color.a = 1.0;
    for (size_t j = 0; j < mPaths[i].poses.size(); j++) {
      pathMarker.points.push_back(mPaths[i].poses[j].pose.position);
    }
    all_paths.markers.push_back(pathMarker);
  }


  mPathPub.publish(all_paths);
  for (size_t i = 0; i < mPathsCost.size(); i++) {
    for (size_t j = 0; j < mPathsCost[i].size(); j++) {
      std::cout << mPathsCost[i][j] << " ";
    }
    std::cout << '\n';
  }
}

double MyViz::calc_path_cost(const std::vector< geometry_msgs::PoseStamped > &poses){
  double sum = 0;

  for (size_t i = 0; i < poses.size() - 1; i++) {
    sum += std::sqrt(std::pow((poses[i+1].pose.position.x - poses[i].pose.position.x),2) + std::pow((poses[i+1].pose.position.y - poses[i].pose.position.y), 2));
  }
  return sum;
}

void MyViz::editACOParam(){

}

void MyViz::runACO(){
  ACOAlgorithm aco(mPathsCost);
  aco.RunACS("test.txt");
  std::vector<int> bestPathNodes;
  double bestLength;
  bestPathNodes = aco.getBestPath();
  bestLength = aco.getBestLength();

  // Print Best Path on Console
  std::cout << "Best Path: ";
  for (size_t i = 0; i < bestPathNodes.size(); i++) {
    std::cout << bestPathNodes[i] << " ";
  }
  std::cout << "Best Length: " << bestLength << '\n';

  // Visualize best path on RViz
  visualization_msgs::MarkerArray best_path;
  visualization_msgs::Marker pathMarker;
  best_path.markers.clear();
  mBestPathPub.publish(best_path);
  pathMarker.header.frame_id = "map";
  pathMarker.header.stamp = ros::Time::now();
  pathMarker.ns = "best_path";
  pathMarker.action = visualization_msgs::Marker::ADD;
  pathMarker.pose.orientation.w = 1.0;

  for (size_t x = 0; x < bestPathNodes.size() - 1; x++) {
    // i,j are the 2 nodes to connect
    int i = bestPathNodes[x];
    int j = bestPathNodes[x+1];

    if ( i > j) {
      std::swap(i,j);
    }
    int n = mPathsCost.size();
    // Find linear index in upper diagonal to get the correct pathMarker
    int k = (n*(n-1)/2) - (n-i)*((n-i)-1)/2 + j - i - 1;
    // std::cout << "Connecting [i,j]. Linear Upper triangular Index [k] " << i << j << k << '\n';


    pathMarker.points.clear();
    pathMarker.id = k;
    pathMarker.type = visualization_msgs::Marker::LINE_STRIP;
    pathMarker.scale.x = 0.08;
    pathMarker.color.r = 1.0;
    // pathMarker.color.g = dis(gen);
    // pathMarker.color.b = dis(gen);
    pathMarker.color.a = 1.0;
    // std::cout << "No of Points in path" << mPaths[k].poses.size()<< '\n';
    for (size_t j = 0; j < mPaths[k].poses.size(); j++) {
      pathMarker.points.push_back(mPaths[k].poses[j].pose.position);
    }
    best_path.markers.push_back(pathMarker);
  }

  std::cout << "No of Paths " <<best_path.markers.size()<< '\n';
  mBestPathPub.publish(best_path);

}

} // end of namespace
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(turtlebot_move::MyViz,rviz::Panel )
