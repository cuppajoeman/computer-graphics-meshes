#include "cube.h"
#include <Eigen/Geometry>

void cube(
  Eigen::MatrixXd & V,
  Eigen::MatrixXi & F,
  Eigen::MatrixXd & UV,
  Eigen::MatrixXi & UF,
  Eigen::MatrixXd & NV,
  Eigen::MatrixXi & NF)
{
  ////////////////////////////////////////////////////////////////////////////
  // resize the matrices to fit the number of elements:
  // - 8 vertices for the cube (v)
  // - 6 faces, each with 4 vertices (f)
  // - 14 uv coordinates for texture mapping (uv)
  // - 6 faces with 4 uv indices each (uf)
  // - 6 normal vectors for the 6 cube faces (nv)
  // - 6 faces with 4 normal indices each (nf)
  ////////////////////////////////////////////////////////////////////////////
  V.resize(8,3);
  F.resize(6,4);
  UV.resize(14,2);
  UF.resize(6,4);
  NV.resize(6,3);
  NF.resize(6,4);

  ////////////////////////////////////////////////////////////////////////////
  // 8 vertex positions of the unit cube.
  // the cube spans from (0, 0, 0) to (1, 1, 1).
  // each row corresponds to a vertex's (x, y, z) position.
  ////////////////////////////////////////////////////////////////////////////
  V << 0, 0, 0,  // vertex 0: bottom-left-back
       1, 0, 0,  // vertex 1: bottom-right-back
       1, 0, 1,  // vertex 2: bottom-right-front
       0, 0, 1,  // vertex 3: bottom-left-front
       0, 1, 0,  // vertex 4: top-left-back
       1, 1, 0,  // vertex 5: top-right-back
       1, 1, 1,  // vertex 6: top-right-front
       0, 1, 1;  // vertex 7: top-left-front
  
  ////////////////////////////////////////////////////////////////////////////
  // 6 faces of the cube using vertex indices.
  // each face is a quadrilateral defined by 4 vertices.
  // the indices refer to rows in the vertex matrix (v) we created above
  ////////////////////////////////////////////////////////////////////////////
  F << 3, 2, 1, 0,  // bottom face
       3, 0, 4, 7,  // left face
       0, 1, 5, 4,  // back face
       1, 2, 6, 5,  // right face
       4, 5, 6, 7,  // top face
       7, 6, 2, 3;  // front face
  
  ////////////////////////////////////////////////////////////////////////////
  // the 14 texture coordinates (uv mapping).
  // see rubicks-cube.png for details as to why they are set as they are
  // each row corresponds to a (u, v) coordinate for texture mapping.
  // these coordinates map the texture to the cube's faces.
  ////////////////////////////////////////////////////////////////////////////
  UV << 0.25, 0,      // uv 0: bottom-left of white texture
        0.5, 0,       // uv 1: bottom-right of white texture
        0, 0.25,      // uv 2: bottom-left of red texture
        0.25, 0.25,   // uv 3: bottom-right of green texture
        0.5, 0.25,    // uv 4: bottom-left of blue texture
        0.75, 0.25,   // uv 5: bottom-right of blue texture
        1, 0.25,      // uv 6: bottom-right of orange texture
        0, 0.5,       // uv 7: top-left of red texture
        0.25, 0.5,    // uv 8: top-left of green texture
        0.5, 0.5,     // uv 9: top-left of blue texture
        0.75, 0.5,    // uv 10: top-left of orange texture
        1, 0.5,       // uv 11: top-right of orange texture
        0.25, 0.75,   // uv 12: top-left of yellow texture
        0.5, 0.75;    // uv 13: top-right of yellow texture

  ////////////////////////////////////////////////////////////////////////////
  // the uv indices for each face (uf).
  // each face has 4 uv indices corresponding to the texture coordinates.
  // the indices refer to rows in the uv matrix and are zero-based.
  ////////////////////////////////////////////////////////////////////////////
  UF << 7, 2, 3, 8,   // RED
        12, 8, 9, 13, // YELLOW
        8, 3, 4, 9,   // GREEN
        3, 0, 1, 4,   // WHITE
        9, 4, 5, 10,  // BLUE
        10, 5, 6, 11; // ORANGE

  ////////////////////////////////////////////////////////////////////////////
  // the normal vectors for each face.
  // each normal vector is a unit vector indicating the direction the face points in.
  // these normals are used for lighting and shading calculations.
  ////////////////////////////////////////////////////////////////////////////
  NV << 0, 0, -1,   // normal for front face (negative z direction)
        1, 0, 0,    // normal for right face (positive x direction)
        0, 0, 1,    // normal for back face (positive z direction)
        -1, 0, 0,   // normal for left face (negative x direction)
        0, -1, 0,   // normal for bottom face (negative y direction)
        0, 1, 0;    // normal for top face (positive y direction)

  ////////////////////////////////////////////////////////////////////////////
  // the normal indices for each face (nf).
  // each face has 4 normal indices, one for each vertex.
  // the indices refer to rows in the nv matrix and are zero-based.
  ////////////////////////////////////////////////////////////////////////////
  NF << 4, 4, 4, 4,  // bottom face
        3, 3, 3, 3,  // left face
        0, 0, 0, 0,  // front face
        1, 1, 1, 1,  // right face
        5, 5, 5, 5,  // top face
        2, 2, 2, 2;  // back face
}
