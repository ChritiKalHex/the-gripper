#pragma once
#include <math.h>

// Everything for kinematics are based on what I can interpret from the drawing
// Good luck

namespace arm {

struct Vec {
  float x, y;
};

inline float toRad(float d) { return d * 0.01745329252f; }
inline float toDeg(float r) { return r * 57.29577951f; }

inline float wrapPi(float a) {
  while (a > 3.14159265f) a -= 6.28318531f;
  while (a <= -3.14159265f) a += 6.28318531f;
  return a;
}

//   A = shoulder pivot, B = elbow-actuator foot, C = shoulder-actuator
//   clevis, D = elbow pivot.
namespace dim {
const float AB = 269.34f;           // A to B, aligned
const float BD = 366.13f;           // B to D, aligned
const float BC = 184.27f;           // B to C, aligned
const float AD_X = 495.05f;         // A to D, horizontal in the plate frame
const float AC_X = 250.15f;         // A to C, horizontal in the plate frame
const float SHOULDER_TO_FOOT = 113.59f;  // A to the actuator foot on the base bracket
const float FOREARM_LEN = 533.40f;
const float FOREARM_WIDTH = 95.25f;      // also the elbow pivot to actuator clevis spacing

// Determined by me measuring the damn pdf and then scaling it
const float D_HEIGHT = 158.75f;     // D above the A-level line in the plate frame
const float FOOT_DROP = 101.0f;     // base-bracket foot below the shoulder pivot
}  // namespace dim

struct Linkage {
  float rFixed, aFixed; // fixed pin, polar about the joint axis
  float rMoving, aMoving; // moving pin at joint angle 0

  float lengthAt(float theta) const {
    float c = cosf(theta + aMoving - aFixed);
    float l2 = rMoving * rMoving + rFixed * rFixed - 2.0f * rMoving * rFixed * c;
    return sqrtf(l2 > 0.0f ? l2 : 0.0f);
  }

  // Picks the branch the reference pose lies on; the other branch is the
  // mirror-image mechanism configuration, which the hardware can't reach.
  float angleAt(float len) const {
    float x = (rMoving * rMoving + rFixed * rFixed - len * len) /
              (2.0f * rMoving * rFixed);
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;
    float delta = wrapPi(aMoving - aFixed);
    float s = (delta >= 0.0f) ? 1.0f : -1.0f;
    return s * acosf(x) - delta;
  }
};

struct Geometry {
  Vec B, C, D;       // link-1 holes (plate frame, A at origin)
  Vec foot;          // shoulder actuator foot on the base bracket
  Vec clevis;        // elbow actuator clevis on the forearm, relative to D
  Vec tip;           // forearm end relative to D, at the reference pose
  Linkage shoulder;  // base foot <-> C
  Linkage elbow;     // B <-> forearm clevis
};

inline Geometry buildGeometry() {
  Geometry g;

  // The five link-1 dimensions fix the plate's shape up to D's height, so
  // B and C are rebuilt from them rather than copied from the drawing by eye.
  g.D = {dim::AD_X, dim::D_HEIGHT};
  float d = sqrtf(g.D.x * g.D.x + g.D.y * g.D.y);
  float along = (dim::AB * dim::AB - dim::BD * dim::BD + d * d) / (2.0f * d);
  float off2 = dim::AB * dim::AB - along * along;
  float off = sqrtf(off2 > 0.0f ? off2 : 0.0f);
  float ux = g.D.x / d, uy = g.D.y / d;
  // B lies on the upper (counter-clockwise) side of the A-D line.
  g.B = {along * ux - off * uy, along * uy + off * ux};

  float dx = dim::AC_X - g.B.x;
  float cy2 = dim::BC * dim::BC - dx * dx;
  float cy = sqrtf(cy2 > 0.0f ? cy2 : 0.0f);
  g.C = {dim::AC_X, g.B.y - cy};  // C is the bump below B

  float footDx = sqrtf(dim::SHOULDER_TO_FOOT * dim::SHOULDER_TO_FOOT -
                       dim::FOOT_DROP * dim::FOOT_DROP);
  g.foot = {footDx, -dim::FOOT_DROP};

  g.clevis = {0.0f, dim::FOREARM_WIDTH};
  // The forearm's rounded top end is centred on the clevis, so the usable
  // length below the elbow pivot excludes the clevis spacing and that end.
  g.tip = {0.0f, -(dim::FOREARM_LEN - dim::FOREARM_WIDTH * 1.5f)};

  g.shoulder.rFixed = sqrtf(g.foot.x * g.foot.x + g.foot.y * g.foot.y);
  g.shoulder.aFixed = atan2f(g.foot.y, g.foot.x);
  g.shoulder.rMoving = sqrtf(g.C.x * g.C.x + g.C.y * g.C.y);
  g.shoulder.aMoving = atan2f(g.C.y, g.C.x);

  Vec bRel = {g.B.x - g.D.x, g.B.y - g.D.y};
  g.elbow.rFixed = sqrtf(bRel.x * bRel.x + bRel.y * bRel.y);
  g.elbow.aFixed = atan2f(bRel.y, bRel.x);
  g.elbow.rMoving = dim::FOREARM_WIDTH;
  g.elbow.aMoving = atan2f(g.clevis.y, g.clevis.x);

  return g;
}

inline Vec tipFromAngles(const Geometry &g, float th1, float th2) {
  float a = th1, b = th1 + th2;
  return {cosf(a) * g.D.x - sinf(a) * g.D.y + cosf(b) * g.tip.x - sinf(b) * g.tip.y,
          sinf(a) * g.D.x + cosf(a) * g.D.y + sinf(b) * g.tip.x + cosf(b) * g.tip.y};
}

// Keeps the elbow configuration of the drawn pose; the mirrored solution
// would need the forearm to pass through link 1.
inline bool anglesFromTip(const Geometry &g, float x, float y, float &th1,
                          float &th2) {
  float r1 = sqrtf(g.D.x * g.D.x + g.D.y * g.D.y);
  float phi1 = atan2f(g.D.y, g.D.x);
  float r2 = sqrtf(g.tip.x * g.tip.x + g.tip.y * g.tip.y);
  float psi = atan2f(g.tip.y, g.tip.x);

  float c = (x * x + y * y - r1 * r1 - r2 * r2) / (2.0f * r1 * r2);
  if (c > 1.0f || c < -1.0f) return false;

  float s = (wrapPi(psi - phi1) >= 0.0f) ? 1.0f : -1.0f;
  float delta = s * acosf(c);
  float a = atan2f(y, x) - atan2f(r2 * sinf(delta), r1 + r2 * cosf(delta));

  th1 = wrapPi(a - phi1);
  th2 = wrapPi(delta - psi + phi1);
  return true;
}

}  // namespace arm