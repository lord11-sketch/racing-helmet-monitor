#include "gravity.h"

#include <math.h>

namespace core {

namespace {

// Below this length a direction is too ill-defined to normalise.
const float kMinFrameLength = 0.2f;

AxisFrame sensorAxesFrame() {
    AxisFrame f;
    f.up      = {0.0f, 0.0f, 1.0f};
    f.forward = {0.0f, 1.0f, 0.0f};
    f.lateral = {1.0f, 0.0f, 0.0f};
    f.valid   = false;
    return f;
}

}  // namespace

float norm(const Vec3& v) { return sqrtf(dot(v, v)); }

GravityEstimator::GravityEstimator(bool adaptive, float tauS, float gateG)
    : adaptive_(adaptive), tauS_(tauS), gateG_(gateG), gravity_{0.0f, 0.0f, 1.0f}, gateOpen_(false) {}

void GravityEstimator::reset(const Vec3& gravity) { gravity_ = gravity; }

Vec3 GravityEstimator::update(const Vec3& accel, float dtS) {
    gateOpen_ = false;
    if (adaptive_ && dtS > 0.0f) {
        gateOpen_ = fabsf(norm(accel) - 1.0f) <= gateG_;
        if (gateOpen_) {
            // First-order low-pass; this form is stable for any dt.
            const float alpha = dtS / (tauS_ + dtS);
            gravity_ = gravity_ + (accel - gravity_) * alpha;
        }
    }
    return accel - gravity_;
}

AxisFrame makeAxisFrame(const Vec3& gravity, const Vec3& forwardHint) {
    const float gLen = norm(gravity);
    if (gLen < kMinFrameLength) return sensorAxesFrame();

    AxisFrame f;
    f.up = gravity * (1.0f / gLen);

    const Vec3  horizontal = forwardHint - f.up * dot(forwardHint, f.up);
    const float hLen       = norm(horizontal);
    if (hLen < kMinFrameLength) return sensorAxesFrame();

    f.forward = horizontal * (1.0f / hLen);
    f.lateral = cross(f.forward, f.up);
    f.valid   = true;
    return f;
}

AxisComponents projectOnFrame(const Vec3& v, const AxisFrame& frame) {
    AxisComponents c;
    c.lateral      = dot(v, frame.lateral);
    c.longitudinal = dot(v, frame.forward);
    c.vertical     = dot(v, frame.up);
    return c;
}

}  // namespace core
