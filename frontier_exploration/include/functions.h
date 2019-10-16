#ifndef functions_H
#define functions_H
#include "ros/ros.h"
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3.h"
#include <tf/tf.h>
#include"rrt_exploration/FrontierTF.h"
#include <tuple>
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
// rdm class, for gentaring random flot numbers
class rdm{
int i;
public:
rdm();
float randomize();
};

//Norm function prototype
float Norm( std::vector<float> , std::vector<float> );

//sign function prototype
float sign(float );
//init map for exploration
void initMap(nav_msgs::OccupancyGrid&,float&, float&,float&, float&,visualization_msgs::Marker&);
//Nearest function prototype
std::tuple<std::vector<float>, int> Nearest(  std::vector< std::vector<float>  > , std::vector<float> );
std::tuple<std::vector<float>,int> NearestSameId(int&,std::vector<rrt_exploration::FrontierTF>&,rrt_exploration::FrontierTF,cartographer_ros_msgs::SubmapList);

std::vector<float> TF2XY(cartographer_ros_msgs::SubmapList,rrt_exploration::FrontierTF);
//Steer function prototype
std::vector<float> Steer(  std::vector<float>, std::vector<float>, float );

//gridValue function prototype
int gridValue(nav_msgs::OccupancyGrid &,std::vector<float>);

char checkNeighbours(nav_msgs::OccupancyGrid &,std::vector<float>);

//ObstacleFree function prototype
char ObstacleFree(std::vector<float> , std::vector<float> & , nav_msgs::OccupancyGrid);
rrt_exploration::FrontierTF Point2Tf(cartographer_ros_msgs::SubmapList,std::vector<float>);

std::vector< std::vector<float>> refreshTree(cartographer_ros_msgs::SubmapList,std::vector<rrt_exploration::FrontierTF>);
void DrawRRT(visualization_msgs::Marker&,std::vector<std::tuple<int,int>>,std::vector< std::vector<float>>);
#endif
