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
class QLabel;

namespace turtlebot_move{
// Class "Waypoints" implements the top level widget for this example.
class Waypoints: public rviz::Panel
{
Q_OBJECT
public:
  Waypoints( QWidget* parent = 0 );
  virtual ~Waypoints();

  void pointReceived(const  geometry_msgs::PointStamped &msg);
  // virtual void load(const rviz::Config &config);
  // virtual void save( rviz::Config config) const;


public Q_SLOTS:
  void addPoint();
  void removePoint();
  void calculatePaths();
  void viewFromToMatrix();
  void editACOParam();
  void runACO();
  void moveToWaypoints();
  void moveToWaypointsThread();


//   void setCellSize( int cell_size_percent );
private:
  double calc_path_cost(const std::vector< geometry_msgs::PoseStamped > &poses);
  void publishMarkerPoints();
  void publishPaths();
  void publishBestPath();

private:
  QLabel      *mCurrentSizeLabel;
  QLabel      *mBestPathLabel;
  QListWidget *mDropdownList;
  QPushButton *mAddPushButton;
  QPushButton *mRemovePushButton;
  QPushButton *mCalculatePathsPushButton;
  QPushButton *mViewFromToMatrixPushButton;
  QPushButton *mEditACOParamButton;
  QPushButton *mRunACOButton;
  QPushButton *mMoveToWaypointsButton;

  std::vector< std::vector< double > > mPathsCost;
  std::vector< geometry_msgs::PointStamped > mWaypoints;
  std::vector< nav_msgs::Path > mPaths;
  std::vector<int> mBestPathNodes;
  double mBestLength;

  ros::Publisher mPathPub;
  ros::Publisher mBestPathPub;
  ros::Publisher mPointsMarkerPub;
  ros::Subscriber mPointSub;
  ros::NodeHandle nh;
};

} // end of namespace
#endif // MYVIZ_H
