#include "vertex_triangle_adjacency.h"
#include <Eigen/Geometry>

void vertex_triangle_adjacency(
  const Eigen::MatrixXi & F,
  const int num_vertices,
  std::vector<std::vector<int> > & VF)
{
  VF.resize(num_vertices);

  // for each face go through the id's of each vertex forming the face
  // and then make a map which maps the vertex id to the faces adjacent
  // kinda just like inverting the map in a certain way
  for (int face_index = 0; face_index < F.rows(); face_index++) {
    for (int vertex_index = 0; vertex_index < 3; vertex_index++) {
      // get the vertex id from the face matrix, working with tri's now so 
      // only needs 3 to determine a face
      int vertex_id = F(face_index, vertex_index);
      // add the current face index to the vertex's adjacency list
      VF[vertex_id].push_back(face_index);
    }
  }
}

