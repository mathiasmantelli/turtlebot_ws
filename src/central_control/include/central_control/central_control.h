#include <central_control/thinning.h>

enum CellType{kUnknown, kObstacle, kFree, kExtraExpansion, kNeighbor, kNeighborExpansion};

struct CentralControlParameters{
//   float OBSTACLE_EXPANSION = 0.32; 
//   float VORONOI_FILTER = 0.07; 
//   float SQUARE_AROUND_POINT = 0.4; 
};

struct Cell{
  int LAST_TIME_ANALYSED = 0; 
  // bool ROBOT_VISITED = false;
  // bool IS_POINT = false;
  int8_t SLAM_OCC;
  int8_t ALL_OBST;
  int8_t ORIGINAL_MAP;
  CellType CELL_TYPE;
  // bool IS_ACCESSIBLE = false;
  // double ENERGY = 0;
  // bool VISITED = false;
  double POTENTIAL = 1.0;
  bool BOUNDARY_CONDITION = true;
};  

struct OldCell{
  int map_index;
  Cell cell;
};  

inline geometry_msgs::Quaternion computeQuaternionFromYaw(const double yaw){
  geometry_msgs::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = sin(yaw * 0.5);
  q.w = cos(yaw * 0.5);
  return q;
}

//// fonte: http://docs.ros.org/en/jade/api/tf2/html/impl_2utils_8h_source.html
inline
double getYaw(const geometry_msgs::Quaternion& q)
{
  double yaw;

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x * q.x;
  sqy = q.y * q.y;
  sqz = q.z * q.z;
  sqw = q.w * q.w;

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (q.x*q.z - q.w*q.y) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

  if (sarg <= -0.99999) {
    yaw   = -2 * atan2(q.y, q.x);
  } else if (sarg >= 0.99999) {
    yaw   = 2 * atan2(q.y, q.x);
  } else {
    yaw   = atan2(2 * (q.x*q.y + q.w*q.z), sqw + sqx - sqy - sqz);
  }
  return yaw;
};

inline double ComputeYaw(const geometry_msgs::Quaternion& q_gm) {
    return getYaw(q_gm);
}