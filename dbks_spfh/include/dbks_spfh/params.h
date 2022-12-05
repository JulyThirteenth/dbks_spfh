#pragma once

#ifndef PARAMS_H
#define PARAMS_H

#include <gflags/gflags.h>
#include <math.h>

// DECLARE_double(GridMapResolution);
// DECLARE_int32(GridMapWidth);
// DECLARE_int32(GridMapHeight);

DECLARE_int32(Headings);
DECLARE_double(DeltaHeadingDeg);
DECLARE_double(DeltaHeadingRad);
DECLARE_double(DeltaHeadingNegRad);

DECLARE_double(ExpansionVelocity);
DECLARE_double(ExpansionDeviationAngle);

DECLARE_double(CarWheelbase);
DECLARE_double(CarFront);
DECLARE_double(CarBack);
DECLARE_double(CarLeft);
DECLARE_double(CarRight);

DECLARE_double(PenaltyTurning);   //转方向盘代价
DECLARE_double(PenaltyCOD);       //改变行驶方向代价
DECLARE_double(PenaltyReversing); //逆向行驶代价
DECLARE_double(PenaltyObsDis);    //障碍物项系数

DECLARE_bool(CarReverse);
DECLARE_double(CarMinTurningRadius);

DECLARE_double(RatioOfH);
DECLARE_double(RatioOfI);
DECLARE_double(ThresholdDistance);

enum SearchType {BothWay, ForwardWay, ReverseWay};
DECLARE_int32(SearchWay);
DECLARE_int32(MaxIterations);
DECLARE_int32(ShiftIterations);
DECLARE_double(MaxSwelling);
DECLARE_double(MinSwelling);
DECLARE_double(MaxShootingDistance);
DECLARE_bool(Visualization);

DECLARE_int32(MaxSmoothIterations);
DECLARE_double(RatioOfSmoothness);
DECLARE_double(RatioOfObstacle);
DECLARE_double(SmoothStepSize);

#endif