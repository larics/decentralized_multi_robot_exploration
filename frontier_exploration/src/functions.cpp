#include "functions.h"

using Rigid3d=cartographer::transform::Rigid3d;

// rdm class, for gentaring random flot numbers
rdm::rdm() {i=time(0);}
float rdm::randomize() { i=i+1;  srand (i);  return float(rand())/float(RAND_MAX);}



//Norm function 
float Norm(std::vector<float> x1,std::vector<float> x2)
{
return pow(	(pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2))	,0.5);
}


//sign function
float sign(float n)
{
if (n<0.0){return -1.0;}
else{return 1.0;}
}

void initMap(nav_msgs::OccupancyGrid& mapData,float& init_map_x, float& init_map_y,float& Xstartx, float& Xstarty,visualization_msgs::Marker& points){

    geometry_msgs::Pose OccupancyOrigin=mapData.info.origin;
    const Rigid3d rigidTransf=cartographer_ros::ToRigid3d(OccupancyOrigin);
    Rigid3d TFpoint=rigidTransf*Rigid3d::Translation(
            Rigid3d::Vector(0.0,mapData.info.height*mapData.info.resolution,0.0));
    points.points.push_back(cartographer_ros::ToGeometryMsgPose(TFpoint).position);
    TFpoint=rigidTransf*Rigid3d::Translation(
            Rigid3d::Vector(0.0,0.0,0.0));
    points.points.push_back(cartographer_ros::ToGeometryMsgPose(TFpoint).position);
    TFpoint=rigidTransf*Rigid3d::Translation(
            Rigid3d::Vector(mapData.info.width*mapData.info.resolution,0.0,0.0));
    points.points.push_back(cartographer_ros::ToGeometryMsgPose(TFpoint).position);
    TFpoint=rigidTransf*Rigid3d::Translation(
            Rigid3d::Vector(mapData.info.width*mapData.info.resolution,mapData.info.height*mapData.info.resolution,0.0));
    points.points.push_back(cartographer_ros::ToGeometryMsgPose(TFpoint).position);

    std::vector<float> temp1;
    temp1.push_back(points.points[0].x);
    temp1.push_back(points.points[0].y);

    std::vector<float> temp2;

    temp2.push_back(points.points[2].x);
    temp2.push_back(points.points[0].y);


    init_map_x=Norm(temp1,temp2);
    temp1.clear();		temp2.clear();

    temp1.push_back(points.points[0].x);
    temp1.push_back(points.points[0].y);

    temp2.push_back(points.points[0].x);
    temp2.push_back(points.points[2].y);

    init_map_y=Norm(temp1,temp2);
    temp1.clear();		temp2.clear();

    Xstartx=(points.points[0].x+points.points[2].x)*.5;
    Xstarty=(points.points[0].y+points.points[2].y)*.5;


}
//Nearest function
std::tuple<std::vector<float>,int> Nearest(  std::vector< std::vector<float>  > V, std::vector<float>  x){

    float min=Norm(V[0],x);
    int min_index;
    float temp;

    for (int j=0;j<V.size();j++)
    {
        temp=Norm(V[j],x);
        if (temp<=min){
            min=temp;
            min_index=j;}

    }

    return std::make_tuple(V[min_index],min_index) ;
}
std::tuple<std::vector<float>,int> NearestSameId(int& init_id,std::vector<rrt_exploration::FrontierTF>& pointsTF,rrt_exploration::FrontierTF point,cartographer_ros_msgs::SubmapList SubmapList_){
    int i=0;
    float min = std::numeric_limits<float>::max();;
    int min_index;
    float temp;
    std::vector<float> best;
    std::vector<float> XYpoint1,XYpoint2;

    XYpoint1=TF2XY(SubmapList_,point);
    for (auto& TFpoint : pointsTF) {
        if(TFpoint.trajectory_id==point.trajectory_id){
            XYpoint2=TF2XY(SubmapList_,TFpoint);
            temp=Norm(XYpoint1,XYpoint2);
            if (temp<=min){
                min=temp;
                std::cout<<min<<std::endl;
                min_index=i;
                best=XYpoint2;
            }
            init_id=1;
        }
        i++;
    }
    if (init_id==0){
        rrt_exploration::FrontierTF added;
        for (auto& submap_msg : SubmapList_.submap) {
            if (submap_msg.trajectory_id==point.trajectory_id){
                added.trajectory_id=submap_msg.trajectory_id;
                added.submapIndex=submap_msg.submap_index;
                geometry_msgs::Vector3 vec;
                vec.x=0;
                vec.y=0;
                vec.z=0;
                added.transform=vec;
                break;
            }
        }

        pointsTF.push_back(added);
        std::cout<<"dodaj"<<std::endl;
        //return dummy values
        min_index=i;
        best=XYpoint2;

    }
    return std::make_tuple(best,min_index) ;

}

std::vector<float> TF2XY(cartographer_ros_msgs::SubmapList SubmapList_,rrt_exploration::FrontierTF pointTF){
    std::vector<float> PointXY;
    for (auto& submap_msg : SubmapList_.submap) {
        if((submap_msg.trajectory_id==pointTF.trajectory_id) && (submap_msg.submap_index==pointTF.submapIndex)){
            PointXY.push_back(pointTF.transform.x+submap_msg.pose.position.x);
            PointXY.push_back(pointTF.transform.y+submap_msg.pose.position.y);
            break;
        }
    }
    return PointXY;
}

//Steer function
std::vector<float> Steer(  std::vector<float> x_nearest , std::vector<float> x_rand, float eta){
std::vector<float> x_new;

if (Norm(x_nearest,x_rand)<=eta){
x_new=x_rand;
}
else{


float m=(x_rand[1]-x_nearest[1])/(x_rand[0]-x_nearest[0]);

x_new.push_back(  (sign(x_rand[0]-x_nearest[0]))* (   sqrt( (pow(eta,2)) / ((pow(m,2))+1) )   )+x_nearest[0] );
x_new.push_back(  m*(x_new[0]-x_nearest[0])+x_nearest[1] );

if(x_rand[0]==x_nearest[0]){
x_new[0]=x_nearest[0];
x_new[1]=x_nearest[1]+eta;
}



}
return x_new;
}


//gridValue function
int gridValue(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp){

float resolution=mapData.info.resolution;
float Xstartx=mapData.info.origin.position.x;
float Xstarty=mapData.info.origin.position.y;

float width=mapData.info.width;
std::vector<signed char> Data=mapData.data;

//returns grid value at "Xp" location
//map data:  100 occupied      -1 unknown       0 free
float indx=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );
int out;
//std::cout<<"Dobio "<<Xp[0]<<" "<<Xp[1]<<std::endl;

//std::cout<<"Ne volim index "<<indx<<std::endl;

out=Data[int(indx)];
std::cout<<"GridValue output "<<out<<std::endl;
return out;
}

inline float getCellIndex(float cell_x_new,float cell_y_new, float width) {return ( cell_y_new*width)+cell_x_new;}
//search for outliers by checking neighbours of free space cells
void checkNeighbours(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp, char& obs){


    //std::vector<float> neigbours;
    float resolution=mapData.info.resolution;//m/cell
    float Xstartx=mapData.info.origin.position.x;
    float Xstarty=mapData.info.origin.position.y;

    float width=mapData.info.width;
    float height=mapData.info.height;
    std::vector<signed char> Data=mapData.data;

    float cell_y=floor((Xp[1]-Xstarty)/resolution);
    float cell_x=floor((Xp[0]-Xstartx)/resolution);

    for (int i=-1;i<=1;i++){
        for(int j=-1;j<=1;j++){
            if (Data[int(getCellIndex(cell_x+i,cell_y+j,width))]>30) obs = 1;
        }
    }
}

// ObstacleFree function-------------------------------------

char ObstacleFree(std::vector<float> xnear, std::vector<float> &xnew, nav_msgs::OccupancyGrid mapsub)
{
    float rez=float(mapsub.info.resolution)*.2;
    float stepz=int(ceil(Norm(xnew,xnear))/rez); 
    std::vector<float> xi=xnear;
    char  obs=0; char unk=0; char free=0;
    std::cout<<"enter ObstacelFree"<<std::endl;

    geometry_msgs::Point p;
    for (int c=0;c<stepz;c++){
      xi=Steer(xi,xnew,rez);

        std::cout<<"check grid value"<<std::endl;

        if (gridValue(mapsub,xi) >=30) {     obs=1; }

        if (gridValue(mapsub,xi) ==-1){      unk=1;	break;}

        if (gridValue(mapsub,xi)<30) {free=1;

        }
        std::cout<<"pass grid val check"<<std::endl;

      }
        std::cout<<"calc out"<<std::endl;

    checkNeighbours(mapsub,xi,obs);
    char out=0;
     xnew=xi;
     if (unk==1){  out=-1;}
     	
     if (obs==1){  out=0;}
     		
     if (unk!=1 && free==1 && obs!=1 ){   out=1;}

     return out;
}

rrt_exploration::FrontierTF Point2Tf(cartographer_ros_msgs::SubmapList SubmapList_,std::vector<float> point) {
    bool init=true;
    float min_dist;
    float dist;
    cartographer_ros_msgs::SubmapEntry best_submap;
    rrt_exploration::FrontierTF newTFPoint;
    for (auto& submap_msg : SubmapList_.submap) {
        std::vector<float> submap_point;
        submap_point.push_back(submap_msg.pose.position.x);
        submap_point.push_back(submap_msg.pose.position.y);
        dist=Norm(submap_point,point);
        if(init==true){
            min_dist=dist;
            best_submap=submap_msg;
            init=false;
        }
        else {
            if(min_dist>dist){
            min_dist=dist;
            best_submap=submap_msg;
            }
        }
    }
    std::cout<<"Robotu "<<best_submap.trajectory_id<<" najbliza submapa "<<best_submap.submap_index<<std::endl;
    const Rigid3d rigidTransf=cartographer_ros::ToRigid3d(best_submap.pose);
    const Rigid3d TFpoint=rigidTransf.inverse()*Rigid3d::Translation(
            Rigid3d::Vector(point[0],point[1],0.0));
    newTFPoint.trajectory_id=best_submap.trajectory_id;
    newTFPoint.submapIndex=best_submap.submap_index;
    newTFPoint.transform.x=cartographer_ros::ToGeometryMsgPose(TFpoint).position.x;
    newTFPoint.transform.y=cartographer_ros::ToGeometryMsgPose(TFpoint).position.y;
    newTFPoint.transform.z=0.0;


    return newTFPoint;

}
std::vector< std::vector<float>> refreshTree(cartographer_ros_msgs::SubmapList SubmapList_,std::vector<rrt_exploration::FrontierTF> pointsTF){
    std::vector< std::vector<float>> RRT;
    for(auto point:pointsTF){
        for (auto& submap_msg : SubmapList_.submap) {
            if((submap_msg.trajectory_id==point.trajectory_id) && (submap_msg.submap_index==point.submapIndex)){
                const Rigid3d rigidTransf=cartographer_ros::ToRigid3d(submap_msg.pose);
                const Rigid3d TFpoint=rigidTransf*Rigid3d::Translation(
                        Rigid3d::Vector(point.transform.x,point.transform.y,point.transform.z));

                RRT.push_back({TFpoint.translation().x(),
                               TFpoint.translation().y()});
                break;
            }
        }
    }
    return RRT;
}
void DrawRRT(visualization_msgs::Marker& line,std::vector<std::tuple<int,int>>indexes,std::vector< std::vector<float>> points){
    line.points.clear();
    int one,two;
    geometry_msgs::Point p;
    std::vector<float> point;
    for (auto ind:indexes){
        std::tie(one,two)=ind;
        point=points.at(one);
        p.x=point[0];
        p.y=point[1];
        p.z=0.0;
        line.points.push_back(p);
        point=points.at(two);
        p.x=point[0];
        p.y=point[1];
        p.z=0.0;
        line.points.push_back(p);
    }
}

 


     
   


  
 
 
 
 





























