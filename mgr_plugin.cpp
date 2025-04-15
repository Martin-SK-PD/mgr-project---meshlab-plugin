#include "mgr_plugin.h"

#define VCG_USE_VERTEX_VFADJ
#define VCG_USE_FACE_VFADJ
#define VCG_USE_VERTEX_CURVATURE
#define VCG_USE_VERTEX_CURVATURE_DIR

#include <QFileDialog>
#include <QImage>
#include <cmath>
#include <common/plugins/interfaces/filter_plugin.h>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <math.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/curvature.h>
#include <vcg/complex/complex.h>
#include <vector>
#include <random>

using namespace std;
using namespace tri;
using namespace vcg;


Mgr_plugin::Mgr_plugin()
{
	typeList = {FP_FIRST, FP_SECOND, FP_THIRD};

	for (ActionIDType tt : types())
		actionList.push_back(new QAction(filterName(tt), this));
}



QString Mgr_plugin::pluginName() const
{
	return "Mgr project plugin";
}



QString Mgr_plugin::filterName(ActionIDType filterId) const
{
	switch (filterId) {
	case FP_FIRST: return QString("Mgr plugin - create mesh");
	case FP_SECOND: return QString("Mgr plugin - filter");
	case FP_THIRD: return QString("Mgr plugin - descriptor");
	default: assert(0); return QString();
	}
}



QString Mgr_plugin::pythonFilterName(ActionIDType f) const
{
	switch (f) {
	case FP_FIRST: return QString("");
	case FP_SECOND: return QString("");
	case FP_THIRD: return QString("");
	default: assert(0); return QString();
	}
}



QString Mgr_plugin::filterInfo(ActionIDType filterId) const
{
	switch (filterId) {
	case FP_FIRST:
		return tr(
			"This function allows users to create 3d model from depthmap."
			"<br>");
	case FP_SECOND:
		return tr("Filter / push vertices based on height from flat or curved reference.");
	case FP_THIRD: return tr("Export descriptor to file.");
	default: assert(0);
	}
	return "";
}



RichParameterList Mgr_plugin::initParameterList(const QAction* a, const MeshDocument& md)
{
	RichParameterList parlst;
	switch (ID(a)) {
	case FP_FIRST:
		parlst.addParam(RichFileOpen(
			"depthMap",
			"",
			QStringList() << "*.png " << "*.jpg",
			"Load depth map",
			"Choose a depth map image"));
		parlst.addParam(RichDynamicFloat(
			"scaleXY", 0.01f, 0.000001f, 0.3f, "scaleXY", "Scaling for X/Y coordinates"));
		parlst.addParam(
			RichDynamicFloat("scaleZ", 1.0f, 0.01f, 5.0f, "scaleZ", "Scaling for Z (depth)"));
		break;
	case FP_SECOND: {
		float avgZ = 0.0f;
		parlst.addParam(RichDynamicFloat(
			"thresholdZ",
			avgZ,
			globalMinZ,
			globalMaxZ,
			"Min height",
			"Minimum relative Z to keep"));

		QStringList modes;
		modes << "flat" << "curved" << "flatten edges";
		parlst.addParam(RichEnum("referencePlane", 0, modes, "Reference", "Base reference shape"));
		break;
	}
	case FP_THIRD: {
		
		break;
	}

	default: break;
	}
	return parlst;
}



FilterPlugin::FilterClass Mgr_plugin::getClass(const QAction* a) const
{
	switch (ID(a)) {
	case FP_FIRST: return FilterPlugin::Other;
	case FP_SECOND: return FilterPlugin::Other;
	case FP_THIRD: return FilterPlugin::Other;
	default: assert(0); return FilterPlugin::Generic;
	}
}



QString Mgr_plugin::filterScriptFunctionName(ActionIDType filterID)
{
	switch (filterID) {
	case FP_FIRST: return QString("First option");
	case FP_SECOND: return QString("Second option");
	case FP_THIRD: return QString("Third option");
	default: assert(0);
	}
	return NULL;
}



std::map<std::string, QVariant> Mgr_plugin::applyFilter(
	const QAction*           filter,
	const RichParameterList& par,
	MeshDocument&            md,
	unsigned int& /*postConditionMask*/,
	CallBackPos*)
{
	if (ID(filter) == FP_FIRST) {
		QString fileName = par.getString("depthMap");
		if (fileName.isEmpty())
			return {};

		float scaleXY = par.getDynamicFloat("scaleXY");
		float scaleZ  = par.getDynamicFloat("scaleZ");

		QImage img(fileName);
		if (img.isNull())
			return {};

		int width = img.width();
		int height = img.height();

		MeshModel* mm   = md.addNewMesh("MeshFromMap", "Generated from depth");
		CMeshO& mesh = mm->cm;

		std::vector<std::vector<int>> grid(height, std::vector<int>(width));
		int index = 0;

		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; ++x) {
				QColor color = img.pixelColor(x, y);
				double gray  = 0.299 * color.red() + 0.587 * color.green() + 0.114 * color.blue();
				float  z     = float(gray / 255.0f) * scaleZ;

				CVertexO v;
				v.P().X() = x * scaleXY;
				v.P().Y() = y * scaleXY;
				v.P().Z() = z;

				mesh.vert.push_back(v);
				grid[y][x] = index++;
			}
		}
		mesh.vn = mesh.vert.size();

		// Výpočet výšok
		float minZ_ = std::numeric_limits<float>::max();
		float maxZ_ = std::numeric_limits<float>::lowest();

		for (const auto& v : mesh.vert) {
			minZ_ = std::min(minZ_, v.P().Z());
			maxZ_ = std::max(maxZ_, v.P().Z());
		}


		// Centrovanie na stred (X, Y)
		float centerX = (width - 1) * 0.5f * scaleXY;
		float centerY = (height - 1) * 0.5f * scaleXY;
		for (auto& v : mesh.vert) {
			v.P().X() -= centerX;
			v.P().Y() -= centerY;
		}

		// Dynamický výber diagonály podľa dĺžky
		for (int y = 0; y < height - 1; ++y) {
			for (int x = 0; x < width - 1; ++x) {
				int i0 = grid[y][x];
				int i1 = grid[y][x + 1];
				int i2 = grid[y + 1][x];
				int i3 = grid[y + 1][x + 1];

				float d1 = Distance(mesh.vert[i0].P(), mesh.vert[i3].P()); // v0–v3
				float d2 = Distance(mesh.vert[i1].P(), mesh.vert[i2].P()); // v1–v2

				if (d1 < d2) {
					// diagonála v0–v3
					CFaceO f1, f2;
					f1.V(0) = &mesh.vert[i0];
					f1.V(1) = &mesh.vert[i1];
					f1.V(2) = &mesh.vert[i3];
					mesh.face.push_back(f1);
					mesh.fn++;

					f2.V(0) = &mesh.vert[i0];
					f2.V(1) = &mesh.vert[i3];
					f2.V(2) = &mesh.vert[i2];
					mesh.face.push_back(f2);
					mesh.fn++;
				}
				else {
					// diagonála v1–v2
					CFaceO f1, f2;
					f1.V(0) = &mesh.vert[i0];
					f1.V(1) = &mesh.vert[i1];
					f1.V(2) = &mesh.vert[i2];
					mesh.face.push_back(f1);
					mesh.fn++;

					f2.V(0) = &mesh.vert[i2];
					f2.V(1) = &mesh.vert[i1];
					f2.V(2) = &mesh.vert[i3];
					mesh.face.push_back(f2);
					mesh.fn++;
				}
			}
		}

		// Vytvorenie spodku modelu
		int topLeft     = grid[0][0];
		int topRight    = grid[0][width - 1];
		int bottomLeft  = grid[height - 1][0];
		int bottomRight = grid[height - 1][width - 1];

		float minZ = std::numeric_limits<float>::max();
		for (const auto& v : mesh.vert)
			minZ = std::min(minZ, v.P().Z());

		int i0 = mesh.vert.size();
		mesh.vert.emplace_back(mesh.vert[topLeft]);
		mesh.vert.back().P().Z() = minZ;
		int i1 = mesh.vert.size();
		mesh.vert.emplace_back(mesh.vert[topRight]);
		mesh.vert.back().P().Z() = minZ;
		int i2 = mesh.vert.size();
		mesh.vert.emplace_back(mesh.vert[bottomLeft]);
		mesh.vert.back().P().Z() = minZ;
		int i3  = mesh.vert.size();
		mesh.vert.emplace_back(mesh.vert[bottomRight]);
		mesh.vert.back().P().Z() = minZ;
		mesh.vn = mesh.vert.size();

		CFaceO fb1, fb2;
		fb1.V(0) = &mesh.vert[i0];
		fb1.V(1) = &mesh.vert[i1];
		fb1.V(2) = &mesh.vert[i2];
		mesh.face.push_back(fb1);

		fb2.V(0) = &mesh.vert[i2];
		fb2.V(1) = &mesh.vert[i1];
		fb2.V(2) = &mesh.vert[i3];
		mesh.face.push_back(fb2);
		mesh.fn += 2;
		
		// Normály a box
		UpdateBounding<CMeshO>::Box(mesh);
		UpdateNormal<CMeshO>::PerFaceNormalized(mesh);
		UpdateNormal<CMeshO>::PerVertexNormalized(mesh);
		// UpdateNormal<CMeshO>::PerFace(mesh);
		// UpdateNormal<CMeshO>::PerVertex(mesh);
		// UpdateNormal<CMeshO>::PerVertexNormalizedPerFaceNormalized(mesh);
	}

	else if (ID(filter) == FP_SECOND) {

		MeshModel* mm   = md.mm();
		CMeshO&    mesh = mm->cm;

		float thresholdZ = par.getDynamicFloat("thresholdZ");

		QStringList modes;
		modes << "flat" << "curved" << "flatten edges";
		QString mode = modes[par.getEnum("referencePlane")];

		UpdateBounding<CMeshO>::Box(mesh);

		int   movedCount = 0;
		float maxShift   = 0.f;

		//FILE* logFile = fopen("debug_output.txt", "w");
		FILE* logFile = NULL;
		if (!logFile) {
			qWarning("Unable to open debug_output.txt");
		}
		else {
			fprintf(logFile, "=== FP_SECOND DEBUG LOG ===\n");
			fprintf(logFile, "Mode: %s\n", mode.toUtf8().constData());
			fprintf(logFile, "ThresholdZ: %.4f\n", thresholdZ);
			fprintf(
				logFile,
				"Bounding box X range: %.2f .. %.2f (DimX=%.2f)\n",
				mesh.bbox.min.X(),
				mesh.bbox.max.X(),
				mesh.bbox.DimX());
		}

		if (mode == "flat") {
			for (auto& v : mesh.vert) {
				if (v.P().Z() < thresholdZ) {
					if (logFile)
						fprintf(logFile, "Vertex moved (flat): Z=%.4f -> Z=0.0\n", v.P().Z());
					maxShift  = std::max(maxShift, thresholdZ - v.P().Z());
					v.P().Z() = 0.f;
					movedCount++;
				}
			}
		}

		if (logFile) {
			fprintf(logFile, "Total moved vertices: %d\n", movedCount);
			fprintf(logFile, "Max shift: %.4f\n", maxShift);
			fclose(logFile);
		}

		UpdateNormal<CMeshO>::PerFaceNormalized(mesh);
		UpdateNormal<CMeshO>::PerVertexNormalized(mesh);
		UpdateBounding<CMeshO>::Box(mesh);

		qDebug("FP_SECOND: moved %d vertices (max shift %.4f). Log saved.", movedCount, maxShift);
		
	}

	else if (ID(filter) == FP_THIRD) {

	}

	else {
		wrongActionCalled(filter);
	}
	return {};
}



int Mgr_plugin::postCondition(const QAction* filter) const
{
	switch (ID(filter)) {
	case FP_FIRST: return MeshModel::MM_NONE;
	case FP_SECOND: return MeshModel::MM_VERTCOORD;
	case FP_THIRD: return MeshModel::MM_NONE;
	default: assert(0);
	}
	return MeshModel::MM_NONE;
}



int Mgr_plugin::getRequirements(const QAction* filter){
	switch (ID(filter)) {
	case FP_FIRST: return MeshModel::MM_VERTCOORD;
	case FP_SECOND: return MeshModel::MM_VERTCOORD | MeshModel::MM_FACEFACETOPO 
		| MeshModel::MM_FACENORMAL | MeshModel::MM_FACEFLAG;;
	case FP_THIRD: return;
	default: assert(0); return MeshModel::MM_NONE;
	}
}



int Mgr_plugin::getPreConditions(const QAction* filter) const{
	switch (ID(filter)) {
    case FP_FIRST:
	case FP_SECOND:
	case FP_THIRD: 
	default: return MeshModel::MM_NONE;
	}
}



MESHLAB_PLUGIN_NAME_EXPORTER(Mgr_plugin)
