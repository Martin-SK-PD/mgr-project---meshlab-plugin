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
		
		break;
	case FP_SECOND: {
		
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
		
	}

	else if (ID(filter) == FP_SECOND) {
		
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
	case FP_SECOND: return MeshModel::MM_NONE;
	case FP_THIRD: return MeshModel::MM_NONE;
	default: assert(0);
	}
	return MeshModel::MM_NONE;
}



int Mgr_plugin::getRequirements(const QAction* filter){
	switch (ID(filter)) {
	case FP_FIRST: return MeshModel::MM_VERTCOORD;
	case FP_SECOND: return;
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
