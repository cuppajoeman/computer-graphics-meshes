#include "write_obj.h"
#include <fstream>
#include <cassert>
#include <iostream>
#include <Eigen/Geometry>

bool write_obj(
  const std::string & filename,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  const Eigen::MatrixXd & UV,
  const Eigen::MatrixXi & UF,
  const Eigen::MatrixXd & NV,
  const Eigen::MatrixXi & NF)
{
  /*
  * the format for the obj file entries:
  * - vertex positions: `v x y z`
  * - texture coordinates: `vt u v`
  * - vertex normals: `vn nx ny nz`
  * - faces: `f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3`
  *   where `v` is the vertex index, `vt` is the texture coordinate index, and `vn` is the normal index.
  *   note: obj indices are 1-based, while eigen matrices use 0-based indexing, so we need to add 1 to the indices.
  */

  assert((F.size() == 0 || F.cols() == 3 || F.cols() == 4) && "F must have 3 or 4 columns");
  std::ofstream obj_file(filename);

  if (!obj_file) {
    return false;  
  } 

  // the try block ensures that any exceptions occurring during file writing 
  // (e.g., stream errors, disk failures, or invalid operations) are caught, so the program doesn't crash
  try {
    // write vertex positions
    for (int i = 0; i < V.rows(); ++i){
      obj_file << "v " << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << std::endl;
    }

    // write texture coordinates if uv is provided
    if (UV.size() > 0) {
      for (int i = 0; i < UV.rows(); ++i){
        obj_file << "vt " << UV(i, 0) << " " << UV(i, 1) << std::endl;
      }
    }

    // write vertex normals if nv is provided
    if (NV.size() > 0) {
      for (int i = 0; i < NV.rows(); ++i){
        obj_file << "vn " << NV(i, 0) << " " << NV(i, 1) << " " << NV(i, 2) << std::endl;
      }
    }

    // write face indices, see the spec for this
    for (int i = 0; i < F.rows(); ++i){
      obj_file << "f "; // start writing the face
      for (int j = 0; j < F.cols(); ++j){

        obj_file << F(i, j) + 1;  // write the vertex index

        if (UV.size() > 0 && UF.size() > 0) {
          obj_file << "/" << UF(i, j) + 1;  // texture coordinate index
        } else {
          obj_file << "/";
        }

        if (NV.size() > 0 && NF.size() > 0) {
          obj_file << "/" << NF(i, j) + 1;  // Normal index
        } else {
          obj_file << "/";
        }

        obj_file << " ";
      }
      obj_file << std::endl;
    }

    obj_file.close();
    return true;

  } catch (const std::exception &e) {
    obj_file.close();
    return false;
  }
}
