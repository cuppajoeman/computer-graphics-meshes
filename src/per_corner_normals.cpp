#include "per_corner_normals.h"
#include "triangle_area_normal.h"
// Hint:
#include "vertex_triangle_adjacency.h"
#include <iostream>

#include "per_face_normals.h"
#include <Eigen/Geometry>

void per_corner_normals(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  const double corner_threshold,
  Eigen::MatrixXd & N)
{
  N = Eigen::MatrixXd::Zero(F.rows() * 3, 3);
  
  // prepare to gather adjacent faces for each vertex
  std::vector<std::vector<int>> adjacent_faces;
  vertex_triangle_adjacency(F, V.rows(), adjacent_faces);
  
  // calculate normals for each face
  Eigen::MatrixXd face_normals;
  per_face_normals(V, F, face_normals);
  
  // Compute the area-weighted average of normals for each vertex
  for (int face_index = 0; face_index < F.rows(); face_index++) {
    for (int i = 0; i < 3; i++) {
      Eigen::RowVector3d accumulated_normal(0, 0, 0);
      
      int idx_of_vertex_on_current_face = F(face_index, i);
      auto curr_face_normal = face_normals.row(face_index);

      // iterate over adjacent faces for the current vertex
      for (int adjacent_face_idx : adjacent_faces[idx_of_vertex_on_current_face]) {
        // calculate the threshold for normal similarity
        double threshold = cos(corner_threshold * M_PI / 180.0);

        auto adjacent_face_normal = face_normals.row(adjacent_face_idx);
        // this is really the cos of the angle between them, which describes why
        // the cos is used in the threshold, rather than just the angle itself.
        double similiarity = adjacent_face_normal.dot(curr_face_normal);

        // if this is not met, the angle between the two vectors is near 90 degrees.
        // this results in vectors that are very different not being used 
        // which casts out "outliers" as well as things on other sides of an edge
        // giving smoothed out lighting and crisp edges.
        if (similiarity >= threshold) { 
          // Accumulate the normal for the adjacent face
          accumulated_normal += triangle_area_normal(
            V.row(F(adjacent_face_idx, 0)), 
            V.row(F(adjacent_face_idx, 1)), 
            V.row(F(adjacent_face_idx, 2))
          );
        }
      }
      
      // normalize the accumulated normal and assign it to the output matrix
      N.row(face_index * 3 + i) = accumulated_normal.normalized();
    }
  }
}
