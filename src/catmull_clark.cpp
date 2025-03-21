#include "catmull_clark.h"
#include <unordered_map>
#include <utility>
#include <functional>
#include <Eigen/Geometry>

Eigen::RowVector3d compute_edge_point(
  const Eigen::MatrixXd &V,
  std::unordered_map<int, Eigen::RowVector3d> &face_points,
  std::unordered_map<std::string, std::vector<int>> &directed_edge_to_adjacent_faces, 
  int vertex_idx_a, 
  int vertex_idx_b)
{
  auto vertex_a = V.row(vertex_idx_a);
  auto vertex_b = V.row(vertex_idx_b);
  std::string edge_from_a_to_b_index_key = std::to_string(vertex_idx_a) + " " + std::to_string(vertex_idx_b);
  Eigen::RowVector3d sum(0, 0, 0);

  // this is safe because every edge will have two adjacent faces unless the mesh is bad (assuming not bad :) )
  int idx_of_neighboring_face_point_1 = directed_edge_to_adjacent_faces[edge_from_a_to_b_index_key][0];
  int idx_of_neighboring_face_point_2 = directed_edge_to_adjacent_faces[edge_from_a_to_b_index_key][1];

  auto neighboring_face_point_1 = face_points[idx_of_neighboring_face_point_1];
  auto neighboring_face_point_2 = face_points[idx_of_neighboring_face_point_2];

  // edge point is the average of these four points
  sum += neighboring_face_point_1;
  sum += neighboring_face_point_2;
  sum += vertex_a;
  sum += vertex_b;
  return sum / 4.0;
}

Eigen::RowVector3d get_modified_vertex_position(
  const Eigen::MatrixXd &V,
  std::unordered_map<int, Eigen::RowVector3d> &face_points,
  std::unordered_map<int, std::vector<int>> &vertex_adjacent_faces, 
  std::unordered_map<int, std::vector<int>> &vertex_idx_to_neighbors,
  std::unordered_map<std::string, std::vector<int>> &directed_edge_to_adjacent_faces,
  int vertex_idx)
{
  auto adjacent_faces = vertex_adjacent_faces[vertex_idx];
  int num_adjacent_faces = adjacent_faces.size();

  Eigen::RowVector3d average_of_adjacent_face_pts(0, 0, 0);
  for (auto &face_idx: adjacent_faces) {
    average_of_adjacent_face_pts += face_points[face_idx];   
  }  
  average_of_adjacent_face_pts /= num_adjacent_faces;

  Eigen::RowVector3d average_of_adjacent_edge_points(0, 0, 0);
  int num_adjacent_edges = 0;
  for (auto &neighbor_vertex: vertex_idx_to_neighbors[vertex_idx]) {
    Eigen::RowVector3d edge_point = compute_edge_point(V, face_points, directed_edge_to_adjacent_faces, vertex_idx, neighbor_vertex);
    average_of_adjacent_edge_points += edge_point;
    num_adjacent_edges++;
  }
  average_of_adjacent_edge_points /= num_adjacent_edges;

  Eigen::RowVector3d new_vertex = (average_of_adjacent_face_pts + 2 * average_of_adjacent_edge_points + (num_adjacent_faces - 3) * V.row(vertex_idx)) / num_adjacent_faces;
  return new_vertex;
};


void catmull_clark(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  const int num_iters,
  Eigen::MatrixXd & SV,
  Eigen::MatrixXi & SF)
{
  if (num_iters <= 0) {
    return;
  }
  std::unordered_map<int, Eigen::RowVector3d> face_point_averages;
  std::unordered_map<int, std::vector<int>> vertex_to_adjacent_faces;
  std::unordered_map<std::string, std::vector<int>> directed_edge_to_face_idx;
  std::unordered_map<int, std::vector<int>> vertex_idx_to_neighbors;

  for (int row_face_idx = 0; row_face_idx < F.rows(); row_face_idx++) {

    Eigen::RowVector3d sum_of_all_points_on_face(0, 0, 0);

    for (int col_idx = 0; col_idx < F.cols(); col_idx++) {

      int next_vertex_index = (col_idx + 1) % F.cols();
      // adding F.cols() so that the mod is positive
      int previous_vertex_index = ((col_idx - 1) + F.cols()) % F.cols();

      int v_curr_idx = F(row_face_idx, col_idx);
      int v_next_idx = F(row_face_idx, next_vertex_index);
      int v_prev_idx = F(row_face_idx, previous_vertex_index);

      sum_of_all_points_on_face += V.row(v_curr_idx);

      // for each vertex on a face, that face is adjacent to that vertex
      vertex_to_adjacent_faces[v_curr_idx].push_back(row_face_idx);

      // 
      // find faces for each egde. key is v1-v2 and v2-v1; value is list of faces (two faces)
      // we do the following due to the following phenomenon
      //                                                                                      
      //                      va                                                              
      //      |----------------|----------------|
      //      |  ^---------->  |  ^---------->  |
      //      |  -          -  |  -          -  |
      //      |  -          -  |  -          -  |
      //      |  -          -  |  -          -  |
      //      |  <----------v  |  <----------v  |
      //      |----------------|----------------|
      //                      vb                                                              
      // notice that when iterating over the vertex (indices) of the left square we would do va -> vb
      // but on the second (rightmost) square we would do vb -> va, due to this we will store the 
      // forward and backward edge key into edge faces, that way when iterating over the vertices 
      // when we try to find the faces for a given edge the order will be correct.
      // another benefit is that we can index with either the forward edge or backwards edge and it won't matter
      // we'll still get back the same results
      
      // we add both here because later on we will use the directed_edge_to_face_idx and need to not 
      // care about what order edges are specified in.
      std::string forward_edge_key = std::to_string(v_curr_idx) + " " + std::to_string(v_next_idx);
      std::string backward_edge_key = std::to_string(v_next_idx) + " " + std::to_string(v_curr_idx);

      directed_edge_to_face_idx[forward_edge_key].push_back(row_face_idx);
      directed_edge_to_face_idx[backward_edge_key].push_back(row_face_idx); 

      bool v_curr_doesnt_have_next_as_neighbor = std::find(vertex_idx_to_neighbors[v_curr_idx].begin(), vertex_idx_to_neighbors[v_curr_idx].end(), v_next_idx) == vertex_idx_to_neighbors[v_curr_idx].end();
      if (v_curr_doesnt_have_next_as_neighbor) {
        vertex_idx_to_neighbors[v_curr_idx].push_back(v_next_idx);
      }
      
      bool v_curr_doesnt_have_prev_as_neighbor = std::find(vertex_idx_to_neighbors[v_curr_idx].begin(), vertex_idx_to_neighbors[v_curr_idx].end(), v_prev_idx) == vertex_idx_to_neighbors[v_curr_idx].end();
      if (v_curr_doesnt_have_prev_as_neighbor) {
        vertex_idx_to_neighbors[v_curr_idx].push_back(v_prev_idx);
      }

    }
    face_point_averages[row_face_idx] = sum_of_all_points_on_face / 4;
  }

  // resize so that they can fit in the vertices and indices as a matrix
  // we need four because we are storing quads.
  SV.resize(0, 3);
  SF.resize(0, 4);

  for (int row_face_idx = 0; row_face_idx < F.rows(); row_face_idx++) {
    for (int j = 0; j < F.cols(); j++) {
      // increase size to fit in new patch
      SF.conservativeResize(SF.rows()+1, 4);

      int v_curr_idx = F(row_face_idx, j);
      int v_next_idx = F(row_face_idx, (j + 1) % F.cols());
      int v_prev_idx = F(row_face_idx, ((j - 1) + F.cols()) % F.cols());

      std::unordered_map<int, Eigen::RowVector3d> new_catmull_points_around_curr_vertex;

      Eigen::RowVector3d edge_point_between_curr_and_next = compute_edge_point(V, face_point_averages, directed_edge_to_face_idx, v_curr_idx, v_next_idx);
      Eigen::RowVector3d edge_point_between_curr_and_prev = compute_edge_point(V, face_point_averages, directed_edge_to_face_idx, v_curr_idx, v_prev_idx);
      Eigen::RowVector3d face_point = face_point_averages[row_face_idx];  
      Eigen::RowVector3d modified_vertex = get_modified_vertex_position(V, face_point_averages, vertex_to_adjacent_faces, vertex_idx_to_neighbors, directed_edge_to_face_idx, v_curr_idx);

      new_catmull_points_around_curr_vertex[0] = modified_vertex;
      new_catmull_points_around_curr_vertex[1] = edge_point_between_curr_and_next;
      new_catmull_points_around_curr_vertex[2] = face_point;
      new_catmull_points_around_curr_vertex[3] = edge_point_between_curr_and_prev;            
      
      // add in the new points to the algorithm
      for (int i = 0; i < 4; i++) {
        bool point_already_exists = false;
        // go through and see if we already have the vertex, if we do re-use the index
        for (int constructed_vertex_idx = 0; constructed_vertex_idx < SV.rows(); constructed_vertex_idx++) {
          auto already_constructed_vertex = SV.row(constructed_vertex_idx);
          point_already_exists = new_catmull_points_around_curr_vertex[i].isApprox(already_constructed_vertex);

          if (point_already_exists) {
            // then re-use the same vertex index for this place.
            SF(SF.rows() - 1, i) = constructed_vertex_idx;
            break;
          }
        }

        if (not point_already_exists) {
          // get space for the new vertex
          SV.conservativeResize(SV.rows()+1, 3);
          // store the vertex
          SV.row(SV.rows()-1) = new_catmull_points_around_curr_vertex[i];
          // store the index for it
          SF(SF.rows()-1, i) = SV.rows()-1;
        } 
      }
    }  
  }
  // recurse
  catmull_clark(Eigen::MatrixXd(SV), Eigen::MatrixXi(SF), num_iters - 1, SV, SF);
}
