#include "triangle_area_normal.h"
#include <Eigen/Geometry>

Eigen::RowVector3d triangle_area_normal(
  const Eigen::RowVector3d & a, 
  const Eigen::RowVector3d & b, 
  const Eigen::RowVector3d & c)
{
  auto dir_1 = b - a, dir_2 = c - a;
  auto cross = dir_1.cross(dir_2);
  Eigen::RowVector3d n = cross.normalized();
  // the length of the cross product is the size of the parallelogram spanned 
  // by the two vectors, thus half of that is the area.
  double parallelogram_area = cross.norm();
  double triangle_area = 0.5 * parallelogram_area;
  return triangle_area * n;
}
