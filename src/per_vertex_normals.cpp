#include "per_vertex_normals.h"
#include "triangle_area_normal.h"
#include <Eigen/Geometry>

void per_vertex_normals(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & N)
{
  N = Eigen::MatrixXd::Zero(V.rows(), 3);
  std::unordered_map<int, Eigen::RowVector3d> face_index_to_normal;
  std::unordered_map<int, std::vector<int>> vertex_index_to_adjacent_faces_index;
  
  // calculate normals for each face and populate adjacent face data
  for (int face_index = 0; face_index < F.rows(); face_index++) {
    Eigen::RowVector3d vertex1 = V.row(F(face_index, 0));
    Eigen::RowVector3d vertex2 = V.row(F(face_index, 1));
    Eigen::RowVector3d vertex3 = V.row(F(face_index, 2));
    
    face_index_to_normal[face_index] = triangle_area_normal(vertex1, vertex2, vertex3);
    
    // update the list of adjacent faces for each vertex in the current face
    // ie, for for each vertex the current face is adjacent
    for (int vertex_index = 0; vertex_index < F.cols(); vertex_index++) {
      int vertex_id = F(face_index, vertex_index);
      vertex_index_to_adjacent_faces_index[vertex_id].push_back(face_index);
    }
  }

  // by this time the vertex to adjacent faces map is complete
  // so we can calculate the weighted average normal for each vertex
  for (int vertex_index = 0; vertex_index < V.rows(); vertex_index++) {
    Eigen::RowVector3d accumulated_normal(0, 0, 0);
    
    // sum the normals of all faces adjacent to the current vertex
    for (int adjacent_face : vertex_index_to_adjacent_faces_index[vertex_index]) {
      accumulated_normal += face_index_to_normal[adjacent_face];
    }
    
    // normalize the accumulated normal and assign it to the output matrix
    N.row(vertex_index) = accumulated_normal.normalized();
  }
}
