#pragma once
#include <math.h>

// ArmKinematics arm({0, 106.934}, {0, 46.99}, {251.60, 184.76}, {237.86, 454.04})

struct Vec2 {float x, y;};

static inline Vec2 vsub(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
static inline Vec2 vadd(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
static inline float vnorm(Vec2 a) { return sqrtf(a.x * a.x + a.y * a.y); }
static inline float vcross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

// Rigid arm pivoting about P. Actuator of length l runs from fixed base
// point b to point y on the arm. x is another point on the arm.
// Reference pose: P, b, y0, x0 (positions of y and x at length l0 = |b - y0|).
class ArmKinematics {
public:
    ArmKinematics(Vec2 P, Vec2 b, Vec2 y0, Vec2 x0) : P_(P), x0_(x0) {
        Vec2 Pb = vsub(b, P), Py0 = vsub(y0, P);
        d_    = vnorm(Pb);
        ry_   = vnorm(Py0);
        l0_   = vnorm(vsub(b, y0));
        side_ = (vcross(Pb, Py0) >= 0.0f) ? 1.0f : -1.0f;
        phi0_ = angleAtP(l0_);
    }

    float referenceLength() const { return l0_; }

    // dl = change in actuator length from the reference length.
    // Writes the position of x into `out` and returns true, or returns
    // false if l = l0 + dl is geometrically impossible.
    bool xPosition(float dl, Vec2 &out) const {
        float l = l0_ + dl;
        float c = (d_ * d_ + ry_ * ry_ - l * l) / (2.0f * d_ * ry_);
        if (c < -1.0f || c > 1.0f) return false;

        float dtheta = side_ * (acosf(c) - phi0_);
        Vec2 r = vsub(x0_, P_);
        float ct = cosf(dtheta), st = sinf(dtheta);
        out = vadd(P_, Vec2{ct * r.x - st * r.y, st * r.x + ct * r.y});
        return true;
    }

private:
    float angleAtP(float l) const {
        float c = (d_ * d_ + ry_ * ry_ - l * l) / (2.0f * d_ * ry_);
        if (c > 1.0f) c = 1.0f;
        if (c < -1.0f) c = -1.0f;
        return acosf(c);
    }

    Vec2 P_, x0_;
    float d_, ry_, l0_, side_, phi0_;
};