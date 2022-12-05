#include "params.h"

// DEFINE_double(GridMapResolution, 0.2,
//               "The resolution of GridMap. Units: m/pixel");
// DEFINE_int32(GridMapWidth, 100,
//              "The width of GridMap. Units: pixel");
// DEFINE_int32(GridMapHeight, 164,
//              "The heigth of GridMap. Units: pixel");

DEFINE_int32(Headings, 72, "The number of discretization.");
DEFINE_double(DeltaHeadingDeg, 360.0 / static_cast<double>(FLAGS_Headings),
              "The dalta heading degree of discretization. Units: degree");
DEFINE_double(DeltaHeadingRad, 2.0 * M_PI / static_cast<double>(FLAGS_Headings),
              "The delta heading radian of discretization. Units: radian");
DEFINE_double(DeltaHeadingNegRad, 2.0 * M_PI - FLAGS_DeltaHeadingRad,
              "2.0 * M_PI - FLAGS_deltaHeadingRad");

DEFINE_bool(CarReverse, true, "Reverse flag of car.");
DEFINE_double(CarMinTurningRadius, 6.2, "Min turning radius of car.");
DEFINE_double(ExpansionVelocity, 0.5, // m/s
              "The velocity of kinematic expansion.");
DEFINE_double(ExpansionDeviationAngle, 24.5, // degree
              "The deviation angle of kinmatic expansion.");

DEFINE_double(CarWheelbase, 2.56, "Wheelbase lenght of car. Units: meter");
DEFINE_double(CarFront, 3.48, "Front length of car. Units: meter");
DEFINE_double(CarBack, 0.88, "Back legnth of car. Units: meter");
DEFINE_double(CarLeft, 0.925, "Left length of car. Units: meter");
DEFINE_double(CarRight, 0.925, "Right length of car. Units: meter");

DEFINE_double(PenaltyTurning, 0.3, "The penalty of turning.");
DEFINE_double(PenaltyCOD, 0.5, "The penalty of change of direction.");
DEFINE_double(PenaltyReversing, 2.0, "The penalty of reversing.");
DEFINE_double(PenaltyObsDis, 1.0, "The penalty of obstacle.");

DEFINE_double(RatioOfH, 1.01, "Ratio of h heuristic.");
DEFINE_double(RatioOfI, 1.01, "Ratio of i heuristic.");
DEFINE_double(ThresholdDistance, 4., "The threshold of obstacle distance in I");

DEFINE_int32(SearchWay, BothWay, "The searching way of DBSK.");
DEFINE_int32(MaxIterations, 5000, "Max iterations of the DBSK.");
DEFINE_int32(ShiftIterations, 10, "Shift iterations of the DBSK.");
DEFINE_double(MaxSwelling, 0.1, "Max swelling size.");
DEFINE_double(MinSwelling, -0.2, "Min swelling size.");
DEFINE_double(MaxShootingDistance, 10.0, "Max distance of shooting.");
DEFINE_bool(Visualization, true, "Flag of alorithm visualization.");

DEFINE_int32(MaxSmoothIterations, 5000, "Max iterations of smoothness.");
DEFINE_double(RatioOfSmoothness, 1.0, "");
DEFINE_double(RatioOfObstacle, 10.0, "");
DEFINE_double(SmoothStepSize, 0.5, "");