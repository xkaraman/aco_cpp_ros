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

#include <QLabel>
#include <QVBoxLayout>
#include <QListWidget>
#include <QPushButton>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/service.h>

#include <nav_msgs/GetPlan.h>

#include "myviz.h"

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

  // Connect Remove to callback slot
  connect(mRemovePushButton,&QPushButton::clicked,this,&MyViz::removePoint );
  connect(mCalculatePathsPushButton,&QPushButton::clicked,this,&MyViz::calculatePaths);

  // mAddPushButton = new QPushButton("Add",this);
  QHBoxLayout *edit_points_layout = new QHBoxLayout;
  edit_points_layout->addWidget(mAddPushButton);
  edit_points_layout->addWidget(mRemovePushButton);


  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(edit_points_layout);
  layout->addWidget(mCalculatePathsPushButton);
  setLayout(layout);

  mPointSub = nh.subscribe("clicked_point",100,&MyViz::pointReceived,this);
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
<<<<<<< HEAD
  std::cout << "### Removed ###" << '\n';
=======
>>>>>>> fa1058ac57f6a565841e89141b3540cc720dd060
  std::cout << "### Points in List ###" << '\n';
  for (size_t i = 0; i < mWaypoints.size(); i++) {
    std::cout << mWaypoints[i];
  }
<<<<<<< HEAD
=======
  std::cout << "### Removed ###" << '\n';
>>>>>>> fa1058ac57f6a565841e89141b3540cc720dd060
}

void MyViz::calculatePaths(){
  ros::service::waitForService("/move_base/make_plan");
  std::cout << "### Calculating Paths Cost ###" << '\n';
  mPathsCost.clear();
  mPathsCost.reserve(mWaypoints.size());
  // for (size_t i = 0; i < mPathsCost.size(); i++) {
  //   mPathsCost[i].reserve(mWaypoints.size());
  //   mPathsCost[i].clear();
  // }
  for (size_t i = 0; i < mWaypoints.size(); i++) {
    mPathsCost.push_back({});
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
            mPathsCost[i].push_back(cost);
        }

      } else{
          mPathsCost[i].push_back(0.0);
      }
    }
  }

  std::cout << "### Done ###" << '\n';
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

} // end of namespace
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(turtlebot_move::MyViz,rviz::Panel )
