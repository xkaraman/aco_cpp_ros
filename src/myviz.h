#ifndef MYVIZ_H
#define MYVIZ_H


#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#endif

class QListWidget;
class QPushButton;

namespace turtlebot_move{
// Class "MyViz" implements the top level widget for this example.
// TODO Refactor MyViz to something meaningfull
class MyViz: public rviz::Panel
{
Q_OBJECT
public:
  MyViz( QWidget* parent = 0 );
  virtual ~MyViz();

  void pointReceived(const  geometry_msgs::PointStamped &msg);
  // virtual void load(const rviz::Config &config);
  // virtual void save( rviz::Config config) const;


// private Q_SLOTS:
  void removePoint();
  void calculatePaths();
  void editACOParam();
  void runACO();

//   void setCellSize( int cell_size_percent );
private:
  double calc_path_cost(const std::vector< geometry_msgs::PoseStamped > &poses);

private:
  QListWidget *dropdown_list;
  QPushButton *mAddPushButton;
  QPushButton *mRemovePushButton;
  QPushButton *mCalculatePathsPushButton;
  QPushButton *mEditACOParamButton;
  QPushButton *mRunACOButton;

  std::vector< std::vector< double > > mPathsCost;
  std::vector< geometry_msgs::PointStamped > mWaypoints;
  std::vector< nav_msgs::Path > mPaths;

  ros::Publisher mPathPub;
  ros::Publisher mBestPathPub;
  ros::Subscriber mPointSub;
  ros::NodeHandle nh;
};

} // end of namespace
#endif // MYVIZ_H
