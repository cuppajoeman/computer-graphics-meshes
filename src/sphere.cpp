#include "sphere.h"
#include <iostream>
#include <Eigen/Geometry>

void sphere(
  const int num_faces_u,
  const int num_faces_v,
  Eigen::MatrixXd & V,
  Eigen::MatrixXi & F,
  Eigen::MatrixXd & UV,
  Eigen::MatrixXi & UF,
  Eigen::MatrixXd & NV,
  Eigen::MatrixXi & NF)
{
  const int num_vertices_u = num_faces_u + 1;
  const int num_vertices_v = num_faces_v + 1;
  const int num_vertices = num_vertices_u * num_vertices_v;
  const int num_faces = num_faces_u * num_faces_v;

  // resize output matrices
  V.resize(num_vertices, 3);
  F.resize(num_faces, 4);
  UV.resize(num_vertices, 2);
  UF.resize(num_faces, 4);
  NV.resize(num_vertices, 3);
  NF.resize(num_faces, 4);

  // generate vertices and uv coordinates
  for (int v = 0; v < num_vertices_v; v++) {
    double phi = M_PI * double(v) / double(num_faces_v);  // latitude/pitch (0 to pi)
    for (int u = 0; u < num_vertices_u; ++u) {
      double theta = 2 * M_PI * double(u) / double(num_faces_u);  // longitude/yaw (0 to 2pi)

      int idx = v * num_vertices_u + u;

      // this looks "messed up" but in opengl and general graphics
      // z is forward and y is up, so this is actually correct
      double x = std::sin(phi) * std::cos(theta);
      double y = std::cos(phi);  // latitude
      double z = std::sin(phi) * std::sin(theta);

      V(idx, 0) = x;
      V(idx, 1) = y;
      V(idx, 2) = z;

      // uv coordinates (latitude-longitude mercator projection)
      // note that I had to flip this, this is probably because the image loading library is loading
      // in the image upside down, which I dislike
      UV(idx, 0) = 1.0 - double(u) / double(num_faces_u);  // u (longitude)
      UV(idx, 1) = 1.0 - double(v) / double(num_faces_v);  // v (latitude)

      // normal vectors (same as vertex positions, since the sphere is centered at origin)
      NV(idx, 0) = x;
      NV(idx, 1) = y;
      NV(idx, 2) = z;
    }
  }

  // generate face indices
  // for this to make sense realize that in the upper loop we mainly iterate over 
  // the yaw, therefore +1 takes us from left to right, we have to go a whole row 
  // over to get to go down, top to bottom
  for (int v = 0; v < num_faces_v; v++) {
    for (int u = 0; u < num_faces_u; u++) {
      int idx = v * num_faces_u + u;

      int top_left = v * num_vertices_u + u;
      int top_right = top_left + 1;
      int bottom_left = (v + 1) * num_vertices_u + u;
      int bottom_right = bottom_left + 1;

      F(idx, 0) = top_left;
      F(idx, 1) = top_right;
      F(idx, 2) = bottom_right;
      F(idx, 3) = bottom_left;

      UF(idx, 0) = top_left;
      UF(idx, 1) = top_right;
      UF(idx, 2) = bottom_right;
      UF(idx, 3) = bottom_left;

      NF(idx, 0) = top_left;
      NF(idx, 1) = top_right;
      NF(idx, 2) = bottom_right;
      NF(idx, 3) = bottom_left;
    }
  }
}
