#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include <map>
#include "LocalPlace.h"

int IFCViewer (int argc, char* argv[]);

void DebugPolygon (const std::vector<Eigen::Vector3d>& coords, const Eigen::Vector3d& offset);

void DebugPoints (const std::vector<Eigen::Vector3d>& coords, const std::map<UInt32, UInt32>& markedMap, const Eigen::Vector3d& offset);

