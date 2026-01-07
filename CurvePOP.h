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

#pragma once

#include "POP_CPlusPlusBase.h"
#include <string>
#include <vector>
#include <algorithm>
using namespace TD;

// Curve evaluation structures
struct CurvePoint
{
	Position pos;
	Vector tangent;
	Vector normal;
	float u; // parametric coordinate [0,1]
	float arcLength;
};

struct CurveData
{
	std::vector<Position> controlPoints;
	std::vector<CurvePoint> samples;
	float totalLength;
	bool closed;

	void clear()
	{
		controlPoints.clear();
		samples.clear();
		totalLength = 0.0f;
		closed = false;
	}
};

// To get more help about these functions, look at POP_CPlusPlusBase.h
class CurvePOP : public POP_CPlusPlusBase
{
public:
	CurvePOP(const OP_NodeInfo *info, POP_Context *context);

	virtual ~CurvePOP();

	virtual void getGeneralInfo(POP_GeneralInfo *, const OP_Inputs *, void *reserved1) override;

	virtual void execute(POP_Output *, const OP_Inputs *, void *reserved) override;

	virtual int32_t getNumInfoCHOPChans(void *reserved) override;

	virtual void getInfoCHOPChan(int index, OP_InfoCHOPChan *chan, void *reserved) override;

	virtual bool getInfoDATSize(OP_InfoDATSize *infoSize, void *reserved) override;

	virtual void getInfoDATEntries(int32_t index, int32_t nEntries, OP_InfoDATEntries *entries, void *reserved) override;

	virtual void setupParameters(OP_ParameterManager *manager, void *reserved) override;

private:
	// Curve evaluation methods
	bool buildCurveFromPOP(const OP_POPInput *curveInput, CurveData &curve, int curveType, bool closed);
	void tessellateLinear(CurveData &curve);
	void tessellateCatmullRom(CurveData &curve, int segments);
	void calculateTangentsAndNormals(CurveData &curve);

	// Curve query methods
	float findClosestPointOnCurve(const Position &pos, const CurveData &curve, CurvePoint &outPoint) const;
	CurvePoint evaluateCurve(const CurveData &curve, float u) const;

	// Deformation methods
	Position applyProjectDeformation(const Position &particlePos, const CurvePoint &curvePoint, float strength) const;
	Position applyWrapDeformation(const Position &particlePos, const CurvePoint &curvePoint, float strength, float offset) const;
	Position applyTwistDeformation(const Position &particlePos, const CurvePoint &curvePoint, float strength, float twistRate) const;

	// Helper methods
	float calculateFalloff(float distance, float falloffDist) const;
	Vector calculateBinormal(const Vector &tangent, const Vector &normal) const;
	Vector positionToVector(const Position &p) const;
	Vector positionDiff(const Position &p1, const Position &p2) const;
	Vector crossProduct(const Vector &v1, const Vector &v2) const;

	// Member variables
	POP_Context *myPOPContext;
};
