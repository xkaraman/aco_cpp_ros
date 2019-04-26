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
  // mAddPushButton = new QPushButton("Add",this);
  QHBoxLayout *edit_points_layout = new QHBoxLayout;
  edit_points_layout->addWidget(mAddPushButton);
  edit_points_layout->addWidget(mRemovePushButton);

  mCalculatePathsPushButton = new QPushButton("Calculate Paths", this);
  
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

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(turtlebot_move::MyViz,rviz::Panel )
