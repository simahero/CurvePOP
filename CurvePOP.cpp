/* Shared Use License: This file is owned by Derivative Inc. (Derivative)
 * and can only be used, and/or modified for use, in conjunction with
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement
 * (which also govern the use of this file). You may share or redistribute
 * a modified version of this file provided the following conditions are met:
 *
 * 1. The shared file or redistribution must retain the information set out
 * above and this list of conditions.
 * 2. Derivative's name (Derivative Inc.) or its trademarks may not be used
 * to endorse or promote products derived from this file without specific
 * prior written permission from Derivative.
 */

#include "CurvePOP.h"

#include <stdio.h>
#include <array>
#include <vector>
#include <string.h>
#include <math.h>
#include <assert.h>

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

	DLLEXPORT
	void
	FillPOPPluginInfo(POP_PluginInfo *info)
	{
		// Always set this to POPCPlusPlusAPIVersion
		if (!info->setAPIVersion(POPCPlusPlusAPIVersion))
			return;

		// The opType is the unique name for this TOP. It must start with a
		// capital A-Z character, and all the following characters must lower case
		// or numbers (a-z, 0-9)
		info->customOPInfo.opType->setString("CurvePOP");

		// The opLabel is the text that will show up in the OP Create Dialog
		info->customOPInfo.opLabel->setString("CurvePOP");

		// Will be turned into a 3 letter icon on the nodes
		info->customOPInfo.opIcon->setString("SSP");

		// Information about the author of this OP
		info->customOPInfo.authorName->setString("Deadwaves Studio");
		info->customOPInfo.authorEmail->setString("0@0.com");

		// This POP works with 2 inputs: particles to deform + curve control points
		info->customOPInfo.minInputs = 2;
		info->customOPInfo.maxInputs = 2;
	}

	DLLEXPORT
	POP_CPlusPlusBase *
	CreatePOPInstance(const OP_NodeInfo *info, POP_Context *context)
	{
		// Return a new instance of your class every time this is called.
		// It will be called once per POP that is using the .dll
		return new CurvePOP(info, context);
	}

	DLLEXPORT
	void
	DestroyPOPInstance(POP_CPlusPlusBase *instance)
	{
		// Delete the instance here, this will be called when
		// Touch is shutting down, when the POP using that instance is deleted, or
		// if the POP loads a different DLL
		delete (CurvePOP *)instance;
	}
};

CurvePOP::CurvePOP(const OP_NodeInfo *info, POP_Context *context)
	: myPOPContext(context)
{
}

CurvePOP::~CurvePOP()
{
}

void CurvePOP::getGeneralInfo(POP_GeneralInfo *ginfo, const OP_Inputs *inputs, void *reserved)
{
	// This will cause the node to cook every frame
	ginfo->cookEveryFrameIfAsked = false;
}

void CurvePOP::execute(POP_Output *output, const OP_Inputs *inputs, void *reserved)
{
	// Get inputs
	const OP_POPInput *particleInput = inputs->getInputPOP(0);
	const OP_POPInput *curveInput = inputs->getInputPOP(1);

	if (!particleInput || !curveInput)
		return;

	// Get parameters
	int curveType = inputs->getParInt("Curvetype");
	float strength = inputs->getParDouble("Strength");
	float falloffDist = inputs->getParDouble("Falloff");
	int deformMode = inputs->getParInt("Deformmode");
	bool closed = inputs->getParInt("Closed") != 0;
	float twistRate = inputs->getParDouble("Twistrate");
	int segments = inputs->getParInt("Segments");
	int deformAxis = inputs->getParInt("Deformaxis"); // 0=X, 1=Y, 2=Z
	float curveOffset = inputs->getParDouble("Curveoffset");
	bool keepScale = inputs->getParInt("Keepscale") != 0;

	// Build curve from second input
	CurveData curve;
	if (!buildCurveFromPOP(curveInput, curve, curveType, closed))
		return;

	// Get particle position attribute
	const POP_Attribute *posAttr = particleInput->getAttribute(POP_AttributeClass::Point, "P", nullptr);
	if (!posAttr)
		return;

	// Read position buffer
	POP_GetBufferInfo getInfo;
	auto posBuffer = posAttr->getBuffer(getInfo, nullptr);
	if (!posBuffer)
		return;

	Position *positions = (Position *)posBuffer->getData(nullptr);

	// Get point count
	POP_GetBufferInfo pointInfoGetInfo;
	auto pointInfoBuffer = particleInput->getPointInfo(pointInfoGetInfo, nullptr);
	if (!pointInfoBuffer)
		return;
	POP_PointInfo *pointInfo = (POP_PointInfo *)pointInfoBuffer->getData(nullptr);
	uint32_t numParticles = pointInfo->numPoints;

	// Calculate bounding box along deform axis
	float minAxis = FLT_MAX;
	float maxAxis = -FLT_MAX;
	Position center(0, 0, 0);
	for (uint32_t i = 0; i < numParticles; i++)
	{
		float axisVal = (deformAxis == 0) ? positions[i].x : (deformAxis == 1) ? positions[i].y
																			   : positions[i].z;
		if (axisVal < minAxis)
			minAxis = axisVal;
		if (axisVal > maxAxis)
			maxAxis = axisVal;
		center.x += positions[i].x;
		center.y += positions[i].y;
		center.z += positions[i].z;
	}
	center.x /= numParticles;
	center.y /= numParticles;
	center.z /= numParticles;

	float axisRange = maxAxis - minAxis;
	if (axisRange < 0.0001f)
		axisRange = 1.0f;

	// Create output buffer
	POP_BufferInfo outBufInfo;
	outBufInfo.size = numParticles * sizeof(Position);
	auto outBuffer = myPOPContext->createBuffer(outBufInfo, nullptr);

	Position *outPositions = (Position *)outBuffer->getData(nullptr);

	// Process each particle
	for (uint32_t i = 0; i < numParticles; i++)
	{
		Position particlePos = positions[i];

		// Map particle's axis position to curve parameter u [0,1]
		float axisVal = (deformAxis == 0) ? particlePos.x : (deformAxis == 1) ? particlePos.y
																			  : particlePos.z;
		float normalizedAxis = (axisVal - minAxis) / axisRange;

		float u;
		if (keepScale)
		{
			// Keep original scale - offset moves entire geometry along curve
			float centerAxis = (deformAxis == 0) ? center.x : (deformAxis == 1) ? center.y
																				: center.z;
			float centerU = ((centerAxis - minAxis) / axisRange) + curveOffset;
			float axisOffset = (axisVal - centerAxis) / curve.totalLength; // Scale by curve length
			u = centerU + axisOffset;
		}
		else
		{
			// Stretch to fit curve
			u = normalizedAxis + curveOffset;
		}

		// Handle wrapping for closed curves
		if (closed)
		{
			u = u - floorf(u); // Wrap to [0,1]
		}
		else
		{
			u = fmaxf(0.0f, fminf(1.0f, u)); // Clamp to [0,1]
		}

		// Get point on curve at this u parameter
		CurvePoint curvePoint = evaluateCurve(curve, u);

		// Calculate offset from axis
		float offsetX = (deformAxis == 0) ? 0.0f : particlePos.x - ((deformAxis == 0) ? 0.0f : center.x);
		float offsetY = (deformAxis == 1) ? 0.0f : particlePos.y - ((deformAxis == 1) ? 0.0f : center.y);
		float offsetZ = (deformAxis == 2) ? 0.0f : particlePos.z - ((deformAxis == 2) ? 0.0f : center.z);

		// Calculate falloff based on offset distance
		float offsetDist = sqrtf(offsetX * offsetX + offsetY * offsetY + offsetZ * offsetZ);
		float falloff = calculateFalloff(offsetDist, falloffDist);
		float finalStrength = strength * falloff;

		// Apply deformation based on mode
		Position newPos;
		switch (deformMode)
		{
		case 0: // Follow - geometry follows the curve
		{
			Vector binormal = calculateBinormal(curvePoint.tangent, curvePoint.normal);
			newPos.x = curvePoint.pos.x + curvePoint.normal.x * offsetX + binormal.x * offsetY + curvePoint.tangent.x * offsetZ;
			newPos.y = curvePoint.pos.y + curvePoint.normal.y * offsetX + binormal.y * offsetY + curvePoint.tangent.y * offsetZ;
			newPos.z = curvePoint.pos.z + curvePoint.normal.z * offsetX + binormal.z * offsetY + curvePoint.tangent.z * offsetZ;

			// Lerp with strength
			newPos.x = particlePos.x + finalStrength * (newPos.x - particlePos.x);
			newPos.y = particlePos.y + finalStrength * (newPos.y - particlePos.y);
			newPos.z = particlePos.z + finalStrength * (newPos.z - particlePos.z);
		}
		break;
		case 1: // Project - snap to nearest curve point
		{
			CurvePoint closestPoint;
			float distance = findClosestPointOnCurve(particlePos, curve, closestPoint);
			newPos = applyProjectDeformation(particlePos, closestPoint, finalStrength);
		}
		break;
		case 2: // Twist
			newPos = applyTwistDeformation(particlePos, curvePoint, finalStrength, twistRate);
			break;
		default:
			newPos = particlePos;
		}

		outPositions[i] = newPos;
	}

	// Set output attribute
	POP_AttributeInfo attrInfo;
	attrInfo.name = "P";
	attrInfo.numComponents = 3;
	attrInfo.numColumns = 1;
	attrInfo.type = POP_AttributeType::Float;
	attrInfo.attribClass = POP_AttributeClass::Point;

	POP_SetBufferInfo setInfo;

	output->setAttribute(&outBuffer, attrInfo, setInfo, nullptr);

	// Copy point info from input
	POP_GetBufferInfo infoGetInfo;
	auto inputPointInfoBuffer = particleInput->getPointInfo(infoGetInfo, nullptr);
	POP_PointInfo *inputPointInfo = (POP_PointInfo *)inputPointInfoBuffer->getData(nullptr);

	POP_BufferInfo pointInfoOutBufInfo;
	pointInfoOutBufInfo.size = sizeof(POP_PointInfo);
	auto pointInfoOutBuffer = myPOPContext->createBuffer(pointInfoOutBufInfo, nullptr);

	POP_PointInfo *outPointInfo = (POP_PointInfo *)pointInfoOutBuffer->getData(nullptr);
	*outPointInfo = *inputPointInfo; // Copy entire structure

	// Copy topology info from input
	auto inputTopoInfoBuffer = particleInput->getTopologyInfo(infoGetInfo, nullptr);
	POP_TopologyInfo *inputTopoInfo = (POP_TopologyInfo *)inputTopoInfoBuffer->getData(nullptr);

	POP_BufferInfo topoInfoBufInfo;
	topoInfoBufInfo.size = sizeof(POP_TopologyInfo);
	auto topoInfoBuffer = myPOPContext->createBuffer(topoInfoBufInfo, nullptr);

	POP_TopologyInfo *topoInfo = (POP_TopologyInfo *)topoInfoBuffer->getData(nullptr);
	*topoInfo = *inputTopoInfo; // Copy entire structure

	// Copy index buffer if it exists (defines faces/topology)
	const POP_IndexBuffer *inputIndexBuffer = particleInput->getIndexBuffer(nullptr);
	if (inputIndexBuffer)
	{
		POP_GetBufferInfo indexGetInfo;
		auto inputIdxBuf = inputIndexBuffer->getBuffer(indexGetInfo, nullptr);
		if (inputIdxBuf)
		{
			// Create output index buffer with same size
			POP_BufferInfo indexBufInfo;
			indexBufInfo.size = inputIdxBuf->info.size;
			auto outputIdxBuf = myPOPContext->createBuffer(indexBufInfo, nullptr);

			// Copy index data
			void *srcData = inputIdxBuf->getData(nullptr);
			void *dstData = outputIdxBuf->getData(nullptr);
			memcpy(dstData, srcData, indexBufInfo.size);

			// Set index buffer
			POP_IndexBufferInfo idxInfo;
			idxInfo.type = inputIndexBuffer->info.type;
			output->setIndexBuffer(&outputIdxBuf, idxInfo, setInfo, nullptr);
		}
	}

	// Set info buffers
	POP_InfoBuffers infoBuffers;
	infoBuffers.pointInfo = pointInfoOutBuffer;
	infoBuffers.topoInfo = topoInfoBuffer;

	output->setInfoBuffers(&infoBuffers, setInfo, nullptr);
}

//-----------------------------------------------------------------------------------------------------
//								CHOP, DAT, and custom parameters
//-----------------------------------------------------------------------------------------------------

int32_t
CurvePOP::getNumInfoCHOPChans(void *reserved)
{
	// We return the number of channel we want to output to any Info CHOP
	// connected to the CHOP. In this example we are just going to send 4 channels.
	return 0;
}

void CurvePOP::getInfoCHOPChan(int32_t index, OP_InfoCHOPChan *chan, void *reserved)
{
	// This function will be called once for each channel we said we'd want to return
	// In this example it'll only be called once.
}

bool CurvePOP::getInfoDATSize(OP_InfoDATSize *infoSize, void *reserved)
{
	infoSize->rows = 0;
	infoSize->cols = 0;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void CurvePOP::getInfoDATEntries(int32_t index, int32_t nEntries, OP_InfoDATEntries *entries, void *reserved)
{
}

void CurvePOP::setupParameters(OP_ParameterManager *manager, void *reserved)
{
	// Curve type
	{
		OP_StringParameter sp;
		sp.name = "Curvetype";
		sp.label = "Curve Type";
		sp.page = "Curve";
		sp.defaultValue = "linear";

		const char *names[] = {"linear", "catmullrom"};
		const char *labels[] = {"Linear", "Catmull-Rom"};
		manager->appendMenu(sp, 2, names, labels);
	}

	// Curve closed
	{
		OP_NumericParameter np;
		np.name = "Closed";
		np.label = "Closed Curve";
		np.page = "Curve";
		np.defaultValues[0] = 0.0;
		manager->appendToggle(np);
	}

	// Segments for curve tessellation
	{
		OP_NumericParameter np;
		np.name = "Segments";
		np.label = "Segments";
		np.page = "Curve";
		np.defaultValues[0] = 20.0;
		np.minSliders[0] = 2.0;
		np.maxSliders[0] = 100.0;
		np.clampMins[0] = true;
		np.minValues[0] = 2.0;
		manager->appendInt(np);
	}

	// Deformation mode
	{
		OP_StringParameter sp;
		sp.name = "Deformmode";
		sp.label = "Deform Mode";
		sp.page = "Deform";
		sp.defaultValue = "follow";

		const char *names[] = {"follow", "project", "twist"};
		const char *labels[] = {"Follow", "Project", "Twist"};
		manager->appendMenu(sp, 3, names, labels);
	}

	// Deform axis
	{
		OP_StringParameter sp;
		sp.name = "Deformaxis";
		sp.label = "Deform Axis";
		sp.page = "Deform";
		sp.defaultValue = "z";

		const char *names[] = {"x", "y", "z"};
		const char *labels[] = {"X", "Y", "Z"};
		manager->appendMenu(sp, 3, names, labels);
	}

	// Keep scale toggle
	{
		OP_NumericParameter np;
		np.name = "Keepscale";
		np.label = "Keep Scale";
		np.page = "Deform";
		np.defaultValues[0] = 1.0;
		manager->appendToggle(np);
	}

	// Curve offset
	{
		OP_NumericParameter np;
		np.name = "Curveoffset";
		np.label = "Curve Offset";
		np.page = "Deform";
		np.defaultValues[0] = 0.0;
		np.minSliders[0] = -1.0;
		np.maxSliders[0] = 1.0;
		manager->appendFloat(np);
	}

	// Strength
	{
		OP_NumericParameter np;
		np.name = "Strength";
		np.label = "Strength";
		np.page = "Deform";
		np.defaultValues[0] = 1.0;
		np.minSliders[0] = 0.0;
		np.maxSliders[0] = 1.0;
		manager->appendFloat(np);
	}

	// Falloff distance
	{
		OP_NumericParameter np;
		np.name = "Falloff";
		np.label = "Falloff Distance";
		np.page = "Deform";
		np.defaultValues[0] = 1.0;
		np.minSliders[0] = 0.0;
		np.maxSliders[0] = 10.0;
		np.minValues[0] = 0.0;
		np.clampMins[0] = true;
		manager->appendFloat(np);
	}

	// Twist rate (for twist mode)
	{
		OP_NumericParameter np;
		np.name = "Twistrate";
		np.label = "Twist Rate";
		np.page = "Deform";
		np.defaultValues[0] = 1.0;
		np.minSliders[0] = -5.0;
		np.maxSliders[0] = 5.0;
		manager->appendFloat(np);
	}
}

//-----------------------------------------------------------------------------------------------------
//								Curve evaluation implementations
//-----------------------------------------------------------------------------------------------------

bool CurvePOP::buildCurveFromPOP(const OP_POPInput *curveInput, CurveData &curve, int curveType, bool closed)
{
	curve.clear();
	curve.closed = closed;

	// Get position attribute from curve particles
	const POP_Attribute *posAttr = curveInput->getAttribute(POP_AttributeClass::Point, "P", nullptr);
	if (!posAttr)
		return false;

	POP_GetBufferInfo getInfo;
	auto posBuffer = posAttr->getBuffer(getInfo, nullptr);
	if (!posBuffer)
		return false;

	Position *positions = (Position *)posBuffer->getData(nullptr);

	// Get point count
	POP_GetBufferInfo pointInfoGetInfo;
	auto pointInfoBuffer = curveInput->getPointInfo(pointInfoGetInfo, nullptr);
	if (!pointInfoBuffer)
		return false;
	POP_PointInfo *pointInfo = (POP_PointInfo *)pointInfoBuffer->getData(nullptr);
	uint32_t numPoints = pointInfo->numPoints;

	if (numPoints < 2)
		return false;

	// Store control points
	curve.controlPoints.reserve(numPoints);
	for (uint32_t i = 0; i < numPoints; i++)
	{
		curve.controlPoints.push_back(positions[i]);
	}

	// Tessellate curve based on type
	if (curveType == 0) // Linear
	{
		tessellateLinear(curve);
	}
	else // Catmull-Rom
	{
		int segments = 20; // This will be passed as parameter
		tessellateCatmullRom(curve, segments);
	}

	// Calculate tangents and normals
	calculateTangentsAndNormals(curve);

	return true;
}

void CurvePOP::tessellateLinear(CurveData &curve)
{
	curve.samples.clear();

	uint32_t numPoints = (uint32_t)curve.controlPoints.size();
	uint32_t numSegments = curve.closed ? numPoints : numPoints - 1;

	curve.samples.reserve(numSegments + 1);

	float totalLength = 0.0f;

	for (uint32_t i = 0; i < numSegments; i++)
	{
		Position &p0 = curve.controlPoints[i];
		Position &p1 = curve.controlPoints[(i + 1) % numPoints];

		CurvePoint cp;
		cp.pos = p0;
		cp.u = (float)i / (float)numSegments;
		cp.arcLength = totalLength;
		curve.samples.push_back(cp);

		Vector seg = positionDiff(p1, p0);
		totalLength += seg.length();
	}

	// Add final point if not closed
	if (!curve.closed)
	{
		CurvePoint cp;
		cp.pos = curve.controlPoints[numPoints - 1];
		cp.u = 1.0f;
		cp.arcLength = totalLength;
		curve.samples.push_back(cp);
	}

	curve.totalLength = totalLength;
}

void CurvePOP::tessellateCatmullRom(CurveData &curve, int segments)
{
	curve.samples.clear();

	uint32_t numControlPoints = (uint32_t)curve.controlPoints.size();
	if (numControlPoints < 2)
		return;

	// For Catmull-Rom, we need at least 4 points
	if (numControlPoints < 4 && !curve.closed)
	{
		// Fall back to linear
		tessellateLinear(curve);
		return;
	}

	uint32_t numSegments = curve.closed ? numControlPoints : numControlPoints - 1;
	curve.samples.reserve(numSegments * segments + 1);

	float totalLength = 0.0f;
	Position lastPos = curve.controlPoints[0];

	for (uint32_t i = 0; i < numSegments; i++)
	{
		// Get 4 control points for Catmull-Rom
		Position p0, p1, p2, p3;

		if (curve.closed)
		{
			p0 = curve.controlPoints[(i - 1 + numControlPoints) % numControlPoints];
			p1 = curve.controlPoints[i];
			p2 = curve.controlPoints[(i + 1) % numControlPoints];
			p3 = curve.controlPoints[(i + 2) % numControlPoints];
		}
		else
		{
			// Handle endpoints
			if (i == 0)
				p0 = p1 = curve.controlPoints[0];
			else
				p0 = curve.controlPoints[i - 1];

			p1 = curve.controlPoints[i];
			p2 = curve.controlPoints[std::min(i + 1, numControlPoints - 1)];

			if (i >= numControlPoints - 2)
				p3 = p2;
			else
				p3 = curve.controlPoints[i + 2];
		}

		// Sample the curve segment
		for (int s = 0; s < segments; s++)
		{
			float t = (float)s / (float)segments;
			float t2 = t * t;
			float t3 = t2 * t;

			// Catmull-Rom matrix
			float q0 = -t3 + 2.0f * t2 - t;
			float q1 = 3.0f * t3 - 5.0f * t2 + 2.0f;
			float q2 = -3.0f * t3 + 4.0f * t2 + t;
			float q3 = t3 - t2;

			Position pos;
			pos.x = 0.5f * (p0.x * q0 + p1.x * q1 + p2.x * q2 + p3.x * q3);
			pos.y = 0.5f * (p0.y * q0 + p1.y * q1 + p2.y * q2 + p3.y * q3);
			pos.z = 0.5f * (p0.z * q0 + p1.z * q1 + p2.z * q2 + p3.z * q3);

			if (i > 0 || s > 0)
			{
				Vector seg = positionDiff(pos, lastPos);
				totalLength += seg.length();
			}

			CurvePoint cp;
			cp.pos = pos;
			cp.u = ((float)i + t) / (float)numSegments;
			cp.arcLength = totalLength;
			curve.samples.push_back(cp);

			lastPos = pos;
		}
	}

	// Add final point if not closed
	if (!curve.closed)
	{
		Position finalPos = curve.controlPoints[numControlPoints - 1];
		Vector seg = positionDiff(finalPos, lastPos);
		totalLength += seg.length();

		CurvePoint cp;
		cp.pos = finalPos;
		cp.u = 1.0f;
		cp.arcLength = totalLength;
		curve.samples.push_back(cp);
	}

	curve.totalLength = totalLength;
}

void CurvePOP::calculateTangentsAndNormals(CurveData &curve)
{
	uint32_t numSamples = (uint32_t)curve.samples.size();
	if (numSamples < 2)
		return;

	for (uint32_t i = 0; i < numSamples; i++)
	{
		CurvePoint &cp = curve.samples[i];

		// Calculate tangent
		if (i == numSamples - 1)
		{
			if (curve.closed && numSamples > 2)
			{
				Vector tangent = positionDiff(curve.samples[0].pos, curve.samples[i - 1].pos);
				cp.tangent = tangent;
				cp.tangent.normalize();
			}
			else
			{
				cp.tangent = positionDiff(curve.samples[i].pos, curve.samples[i - 1].pos);
				cp.tangent.normalize();
			}
		}
		else
		{
			Vector tangent = positionDiff(curve.samples[i + 1].pos, (i > 0 ? curve.samples[i - 1].pos : curve.samples[i].pos));
			cp.tangent = tangent;
			cp.tangent.normalize();
		}

		// Calculate normal (perpendicular to tangent)
		// Use a simple method: find the most perpendicular axis
		Vector up(0, 1, 0);
		if (fabs(cp.tangent.y) > 0.99f)
			up = Vector(1, 0, 0);

		cp.normal = crossProduct(cp.tangent, up);
		cp.normal.normalize();
		cp.normal = crossProduct(cp.normal, cp.tangent);
		cp.normal.normalize();
	}
}

float CurvePOP::findClosestPointOnCurve(const Position &pos, const CurveData &curve, CurvePoint &outPoint) const
{
	if (curve.samples.empty())
		return 0.0f;

	float minDist = FLT_MAX;
	uint32_t closestIdx = 0;

	// Brute force search for closest sample
	for (uint32_t i = 0; i < curve.samples.size(); i++)
	{
		Vector diff = positionDiff(pos, curve.samples[i].pos);
		float dist = diff.length();
		if (dist < minDist)
		{
			minDist = dist;
			closestIdx = i;
		}
	}

	outPoint = curve.samples[closestIdx];
	return minDist;
}

CurvePoint CurvePOP::evaluateCurve(const CurveData &curve, float u) const
{
	if (curve.samples.empty())
		return CurvePoint();

	if (u <= 0.0f)
		return curve.samples[0];
	if (u >= 1.0f)
		return curve.samples[curve.samples.size() - 1];

	// Find the two samples to interpolate between
	for (uint32_t i = 0; i < curve.samples.size() - 1; i++)
	{
		if (curve.samples[i].u <= u && curve.samples[i + 1].u >= u)
		{
			float t = (u - curve.samples[i].u) / (curve.samples[i + 1].u - curve.samples[i].u);

			CurvePoint result;
			result.u = u;
			result.pos.x = curve.samples[i].pos.x + t * (curve.samples[i + 1].pos.x - curve.samples[i].pos.x);
			result.pos.y = curve.samples[i].pos.y + t * (curve.samples[i + 1].pos.y - curve.samples[i].pos.y);
			result.pos.z = curve.samples[i].pos.z + t * (curve.samples[i + 1].pos.z - curve.samples[i].pos.z);

			Vector tangent;
			tangent.x = curve.samples[i].tangent.x + t * (curve.samples[i + 1].tangent.x - curve.samples[i].tangent.x);
			tangent.y = curve.samples[i].tangent.y + t * (curve.samples[i + 1].tangent.y - curve.samples[i].tangent.y);
			tangent.z = curve.samples[i].tangent.z + t * (curve.samples[i + 1].tangent.z - curve.samples[i].tangent.z);
			result.tangent = tangent;
			result.tangent.normalize();

			Vector normal;
			normal.x = curve.samples[i].normal.x + t * (curve.samples[i + 1].normal.x - curve.samples[i].normal.x);
			normal.y = curve.samples[i].normal.y + t * (curve.samples[i + 1].normal.y - curve.samples[i].normal.y);
			normal.z = curve.samples[i].normal.z + t * (curve.samples[i + 1].normal.z - curve.samples[i].normal.z);
			result.normal = normal;
			result.normal.normalize();

			return result;
		}
	}

	return curve.samples[curve.samples.size() - 1];
}

Position CurvePOP::applyProjectDeformation(const Position &particlePos, const CurvePoint &curvePoint, float strength) const
{
	// Lerp particle position towards curve position
	Position result;
	result.x = particlePos.x + strength * (curvePoint.pos.x - particlePos.x);
	result.y = particlePos.y + strength * (curvePoint.pos.y - particlePos.y);
	result.z = particlePos.z + strength * (curvePoint.pos.z - particlePos.z);
	return result;
}

Position CurvePOP::applyWrapDeformation(const Position &particlePos, const CurvePoint &curvePoint, float strength, float offset) const
{
	// Move particle to curve, maintaining offset distance in normal direction
	Vector binormal = calculateBinormal(curvePoint.tangent, curvePoint.normal);

	// Get offset direction
	Vector toParticle = positionDiff(particlePos, curvePoint.pos);
	float normalOffset = toParticle.dot(curvePoint.normal);
	float binormalOffset = toParticle.dot(binormal);

	// Project onto curve with maintained offset
	Position target;
	target.x = curvePoint.pos.x + normalOffset * curvePoint.normal.x + binormalOffset * binormal.x;
	target.y = curvePoint.pos.y + normalOffset * curvePoint.normal.y + binormalOffset * binormal.y;
	target.z = curvePoint.pos.z + normalOffset * curvePoint.normal.z + binormalOffset * binormal.z;

	// Lerp with strength
	Position result;
	result.x = particlePos.x + strength * (target.x - particlePos.x);
	result.y = particlePos.y + strength * (target.y - particlePos.y);
	result.z = particlePos.z + strength * (target.z - particlePos.z);
	return result;
}

Position CurvePOP::applyTwistDeformation(const Position &particlePos, const CurvePoint &curvePoint, float strength, float twistRate) const
{
	// Get vector from curve to particle
	Vector toParticle = positionDiff(particlePos, curvePoint.pos);

	// Project onto plane perpendicular to tangent
	float tangentDot = toParticle.dot(curvePoint.tangent);
	Vector tangentComponent(curvePoint.tangent.x * tangentDot, curvePoint.tangent.y * tangentDot, curvePoint.tangent.z * tangentDot);
	Vector radial = toParticle - tangentComponent;
	float radius = radial.length();

	if (radius < 0.0001f)
		return particlePos;

	// Calculate twist angle based on u parameter and twist rate
	float angle = curvePoint.u * twistRate * 3.14159f * 2.0f * strength;

	// Rotate radial vector around tangent
	Vector binormal = calculateBinormal(curvePoint.tangent, curvePoint.normal);

	float radialNormal = radial.dot(curvePoint.normal);
	float radialBinormal = radial.dot(binormal);

	float cosAngle = cosf(angle);
	float sinAngle = sinf(angle);

	float newRadialNormal = radialNormal * cosAngle - radialBinormal * sinAngle;
	float newRadialBinormal = radialNormal * sinAngle + radialBinormal * cosAngle;

	Position result;
	result.x = curvePoint.pos.x + curvePoint.tangent.x * tangentDot +
			   curvePoint.normal.x * newRadialNormal + binormal.x * newRadialBinormal;
	result.y = curvePoint.pos.y + curvePoint.tangent.y * tangentDot +
			   curvePoint.normal.y * newRadialNormal + binormal.y * newRadialBinormal;
	result.z = curvePoint.pos.z + curvePoint.tangent.z * tangentDot +
			   curvePoint.normal.z * newRadialNormal + binormal.z * newRadialBinormal;

	return result;
}

float CurvePOP::calculateFalloff(float distance, float falloffDist) const
{
	if (falloffDist <= 0.0001f)
		return 1.0f;

	if (distance >= falloffDist)
		return 0.0f;

	// Smooth falloff
	float t = distance / falloffDist;
	return 1.0f - t * t * (3.0f - 2.0f * t);
}

Vector CurvePOP::calculateBinormal(const Vector &tangent, const Vector &normal) const
{
	return crossProduct(tangent, normal);
}

Vector CurvePOP::positionToVector(const Position &p) const
{
	return Vector(p.x, p.y, p.z);
}

Vector CurvePOP::positionDiff(const Position &p1, const Position &p2) const
{
	return Vector(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

Vector CurvePOP::crossProduct(const Vector &v1, const Vector &v2) const
{
	return Vector(
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x);
}
