#include "per_face_normals.h"
#include "triangle_area_normal.h"
#include <Eigen/Geometry>

void per_face_normals(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & N)
{
  N = Eigen::MatrixXd::Zero(F.rows(), 3);
  for (int face_index = 0; face_index < F.rows(); ++face_index) {
    Eigen::RowVector3d vertex1 = V.row(F(face_index, 0));
    Eigen::RowVector3d vertex2 = V.row(F(face_index, 1));
    Eigen::RowVector3d vertex3 = V.row(F(face_index, 2));
    N.row(face_index) = triangle_area_normal(vertex1, vertex2, vertex3).normalized();
  }
}
