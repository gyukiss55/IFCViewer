#include <igl/readOFF.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/glfw/Viewer.h>

#include <map>

#include "tutorial_shared_path.h"

#include "tutorial_shared_path.h"

#include "IFCParser.h"
#include "IFCViewer.h"
#include "GeomUtils.h"
#include "LocalPlace.h"

Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::MatrixXd C;

static igl::opengl::glfw::Viewer * viewerPtr = nullptr;
static Eigen::Vector3d offset = {0.,0.,0.};

/*
Eigen::Vector3d m = V.colwise ().minCoeff ();
Eigen::Vector3d M = V.colwise ().maxCoeff ();

viewer.data ().add_edges (V + PD1 * avg, V - PD1 * avg, blue);

viewer.data ().add_points (V_box, Eigen::RowVector3d (1, 0, 0));
*/

void DebugPolygon (const std::vector<Eigen::Vector3d>& coords, const Eigen::Vector3d& offset)
{
	if (viewerPtr == nullptr)
		return;
	Eigen::MatrixXd VD1;
	Eigen::MatrixXd VD2;
	VD1.resize (coords.size (), 3);
	VD2.resize (coords.size (), 3);
	for (UInt32 i = 0; i < coords.size (); ++i) {
		UInt32 int j = i + 1;
		if (i == coords.size () - 1)
			j = 0;
		VD1 (i, 0) = coords[i].x () + offset.x ();
		VD1 (i, 1) = coords[i].y () + offset.y ();
		VD1 (i, 2) = coords[i].z () + offset.z ();
		VD2 (i, 0) = coords[j].x () + offset.x ();
		VD2 (i, 1) = coords[j].y () + offset.y ();
		VD2 (i, 2) = coords[j].z () + offset.z ();
	}
	viewerPtr->data ().add_edges (VD1, VD2, Eigen::RowVector3d (0.5, 0.5, 0));
}


void DebugPoints (const std::vector<Eigen::Vector3d>& coords, const std::map<UInt32, UInt32>& markedMap, const Eigen::Vector3d& offset)
{
	if (viewerPtr == nullptr)
		return;
	Eigen::MatrixXd VD;
	Eigen::MatrixXd CD;
	VD.resize (coords.size (), 3);
	CD.resize (coords.size (), 3);

	Eigen::Vector3d c1 (0., 0., 1.);
	Eigen::Vector3d c2 (0., 1., 0.);
	Eigen::Vector3d c3 (1., 0., 0.);
	Eigen::Vector3d c4 (1., 1., 0.);

	for (UInt32 i = 0; i < coords.size (); ++i) {
		if (i == 0) {
			if (markedMap.count (i) > 0) {
				CD (i, 0) = c4.x ();
				CD (i, 1) = c4.y ();
				CD (i, 2) = c4.z ();

			}
			else {
				CD (i, 0) = c1.x ();
				CD (i, 1) = c1.y ();
				CD (i, 2) = c1.z ();

			}
		}
		else if (markedMap.count (i) > 0) {
			CD (i, 0) = c3.x ();
			CD (i, 1) = c3.y ();
			CD (i, 2) = c3.z ();
		}
		else {
			CD (i, 0) = c2.x ();
			CD (i, 1) = c2.y ();
			CD (i, 2) = c2.z ();
		}

		VD (i, 0) = coords[i].x () + offset.x ();
		VD (i, 1) = coords[i].y () + offset.y ();
		VD (i, 2) = coords[i].z () + offset.z ();
	}
	viewerPtr->data ().add_points (VD, CD);
}


int IFCViewer (int argc, char* argv[])
{
	igl::opengl::glfw::Viewer viewer;
	viewerPtr = &viewer;

	Faces3D faces3D;
	std::vector<std::pair<DWORD, DWORD>> faceProductPairs;
	ReadIFCFaces ("G:/Work/GitHub/_MyGit/Ifc2Brep/primitivBREP.ifc", faces3D, &faceProductPairs);

	// Plot the mesh
	DWORD vertexNum (0);
	DWORD faceNum (0);
	for (const Face3D* face : faces3D.faces) {
		switch (face->type) {
		case Face3D::Triangle3DType:
			vertexNum += static_cast<const Triangle3D*>(face)->coords.size ();
			faceNum += 1;
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

	std::map<int, double> bodyVolumeMap;
	GeomUtils::CalculateVolume (faces3D, bodyVolumeMap);

	V.resize (vertexNum, 3);
	F.resize (faceNum, 3);
	C.resize (faceNum, 3);

	DWORD vertexIndex (0);
	DWORD faceIndex (0);
	for (const Face3D* face : faces3D.faces) {
		switch (face->type) {
		case Face3D::Triangle3DType:
			{
				const Triangle3D* poly = static_cast<const Triangle3D*>(face);
				if (poly->coords.size () == 3) {
					for (DWORD i = 0; i < 3; ++i) {
						if (poly->bodyIndex != 0 && bodyVolumeMap[poly->bodyIndex] < 0.) {
							V (vertexIndex + 2 - i, 0) = poly->coords[i][0];
							V (vertexIndex + 2 - i, 1) = poly->coords[i][1];
							V (vertexIndex + 2 - i, 2) = poly->coords[i][2];

						}
						else {
							V (vertexIndex + i, 0) = poly->coords[i][0];
							V (vertexIndex + i, 1) = poly->coords[i][1];
							V (vertexIndex + i, 2) = poly->coords[i][2];

						}
					}
					F (faceIndex, 0) = vertexIndex;
					F (faceIndex, 1) = vertexIndex + 1;
					F (faceIndex, 2) = vertexIndex + 2;
					C (faceIndex, 0) = poly->color.x ();
					C (faceIndex, 1) = poly->color.y ();
					C (faceIndex, 2) = poly->color.z ();
					faceIndex++;
					vertexIndex += 3;
				}

			}
			break;
		case Face3D::Quad3DType:
			break;
		case Face3D::Polygon3DType:
			{
				const Polygon3D* poly = static_cast<const Polygon3D*>(face);
				if (poly->coords.size () == 3) {
					for (DWORD i = 0; i < 3; ++i) {
						if (poly->bodyIndex != 0 && bodyVolumeMap[poly->bodyIndex] < 0.) {
							V (vertexIndex + 2 - i, 0) = poly->coords[i][0];
							V (vertexIndex + 2 - i, 1) = poly->coords[i][1];
							V (vertexIndex + 2 - i, 2) = poly->coords[i][2];

						}
						else {
							V (vertexIndex + i, 0) = poly->coords[i][0];
							V (vertexIndex + i, 1) = poly->coords[i][1];
							V (vertexIndex + i, 2) = poly->coords[i][2];

						}
					}
					F (faceIndex, 0) = vertexIndex;
					F (faceIndex, 1) = vertexIndex + 1;
					F (faceIndex, 2) = vertexIndex + 2;
					C (faceIndex, 0) = poly->color.x ();
					C (faceIndex, 1) = poly->color.y ();
					C (faceIndex, 2) = poly->color.z ();
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
			const Face3D* face = faces3D.faces[j];
			double r = face->color.x ();
			double g = face->color.y ();
			double b = face->color.z ();
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

	viewer.data ().set_mesh (V, F);
	viewer.data ().set_colors (C);
	viewer.launch ();

	return 0;
}
