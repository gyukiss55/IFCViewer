#include <igl/readOFF.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include "tutorial_shared_path.h"

#include "tutorial_shared_path.h"

#include "IFCParser.h"

Eigen::MatrixXd V;
Eigen::MatrixXi F;




int main(int argc, char *argv[])
{

  Faces3D faces3D;
  std::vector<std::pair<DWORD, DWORD>> faceProductPairs ;
  ReadIFCFaces ("G:/Work/GitHub/_MyGit/Ifc2Brep/TestAC.ifc", faces3D, &faceProductPairs);

  // Plot the mesh
  igl::opengl::glfw::Viewer viewer;
/*
  // Load a mesh in OFF format
  igl::readOFF(TUTORIAL_SHARED_PATH "/bunny.off", V, F);
*/
  DWORD vertexNum (0);
  DWORD faceNum (0);
  for (const Face3D* face : faces3D.faces) {
	  switch (face->type) {
	  case Face3D::Triangle3DType:
		  break;
	  case Face3D::Quad3DType:
		  break;
	  case Face3D::Polygon3DType:
		  if (static_cast<const Polygon3D*>(face)->coords.size () == 3) {
			vertexNum += static_cast<const Polygon3D*>(face)->coords.size ();
			faceNum += 1;
		  }
		  break;
	  }
  }

  V.resize (vertexNum, 3);
  F.resize (faceNum, 3);

  DWORD vertexIndex (0);
  DWORD faceIndex (0);
  for (const Face3D* face : faces3D.faces) {
	  switch (face->type) {
	  case Face3D::Triangle3DType:
		  break;
	  case Face3D::Quad3DType:
		  break;
	  case Face3D::Polygon3DType:
		  {
			  const Polygon3D* poly = static_cast<const Polygon3D*>(face);
			  if (poly->coords.size () == 3) {
				  for (DWORD i = 0; i < 3; ++i) {
						V(vertexIndex + i, 0) = poly->coords[i][0];
						V(vertexIndex + i, 1) = poly->coords[i][1];
						V(vertexIndex + i, 2) = poly->coords[i][2];
				  }
				  F(faceIndex , 0) = vertexIndex;
				  F(faceIndex , 1) = vertexIndex + 1;
				  F(faceIndex , 2) = vertexIndex + 2;
				  faceIndex++;
				  vertexIndex += 3;
			  }

		  }
		  break;
	  }
  }

  const auto update_color = [&](const int fid, const int vid)
  {
	  Eigen::MatrixXd C (faceNum, 3);
	  int fidMin = 0;
	  int fidMax = faces3D.faces.size ();
	  for (int i = 0; i < faceProductPairs.size (); ++i) {
		  if (i < faceProductPairs.size () - 1)
			  fidMax = faceProductPairs[i + 1].first;
		  if (fid >= faceProductPairs[i].first && fid < fidMax) {
			  fidMin = faceProductPairs[i].first;
			  break;
		  }
	  }
	  printf_s ("Select fid: %d vid: %d minf: %d maxf: %d\n", fid, vid, fidMin, fidMax);
	  for (int j = 0; j < faceNum; j++) {
		  double r = 0.;
		  double g = 1.;
		  double b = 0.;
		  if (j >= fidMin && j < fidMax) {
			  r = 1.;
			  g = 0.;
			  b = 0.;
		  }
		  C (j, 0) = r;
		  C (j, 1) = g;
		  C (j, 2) = b;
	  }
 

	  viewer.data ().set_colors (C);
  };


  viewer.callback_mouse_down =
	  [&](igl::opengl::glfw::Viewer& viewer, int, int)->bool
  {
	  int fid;
	  Eigen::Vector3f bc;
	  // Cast a ray in the view direction starting from the mouse position
	  double x = viewer.current_mouse_x;
	  double y = viewer.core ().viewport (3) - viewer.current_mouse_y;
	  if (igl::unproject_onto_mesh (
		  Eigen::Vector2f (x, y),
		  viewer.core ().view,
		  viewer.core ().proj,
		  viewer.core ().viewport,
		  V,
		  F,
		  fid,
		  bc))
	  {
		  int max;
		  bc.maxCoeff (&max);
		  int vid = F (fid, max);
		  update_color (fid, vid);
		  return true;
	  }
	  return false;
  };

  viewer.data().set_mesh(V, F);
  viewer.launch();
}
