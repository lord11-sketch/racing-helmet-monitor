// Gravity estimation and axis projection for a single 3-axis accelerometer.
// Hardware-independent: plain floats in, plain floats out. Units are g.
#pragma once

namespace core {

struct Vec3 {
    float x;
    float y;
    float z;
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& a, float k) { return {a.x * k, a.y * k, a.z * k}; }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
float norm(const Vec3& v);

// Tracks which part of the accelerometer reading is gravity.
//
// A stationary accelerometer reads 1 g pointing "up", in whatever orientation
// it happens to be. This class keeps a low-pass-filtered copy of the reading as
// the gravity estimate and returns (reading - estimate) as the dynamic
// acceleration. The estimate is only updated while the magnitude of the reading
// is within `gateG` of 1 g: during hard cornering or braking the magnitude is
// well above 1 g, the gate closes, and the load is not mistaken for a change
// in orientation.
//
// With adaptive = false the estimate never moves from what reset() was given,
// which reproduces a fixed "subtract 1 g from Z" compensation.
class GravityEstimator {
public:
    GravityEstimator(bool adaptive, float tauS, float gateG);

    // Set the estimate directly, e.g. to the mean reading from a stationary
    // calibration window. The initial estimate is (0, 0, 1).
    void reset(const Vec3& gravity);

    // Feed one accelerometer sample taken dtS seconds after the previous one.
    // Returns the dynamic (gravity-removed) acceleration.
    Vec3 update(const Vec3& accel, float dtS);

    const Vec3& gravity() const { return gravity_; }
    bool gateOpen() const { return gateOpen_; }

private:
    bool  adaptive_;
    float tauS_;
    float gateG_;
    Vec3  gravity_;
    bool  gateOpen_;
};

// Orthonormal frame derived from the gravity direction.
struct AxisFrame {
    Vec3 up;
    Vec3 forward;
    Vec3 lateral;
    bool valid;   // false: fell back to the raw sensor axes
};

// up      = direction of `gravity` (the reading at rest points up)
// forward = `forwardHint` with its vertical part removed
// lateral = forward x up
// If gravity is near zero or the hint is nearly vertical the frame cannot be
// built; the sensor's own axes (X lateral, Y forward, Z up) are returned with
// valid = false.
AxisFrame makeAxisFrame(const Vec3& gravity, const Vec3& forwardHint);

struct AxisComponents {
    float lateral;
    float longitudinal;
    float vertical;
};

AxisComponents projectOnFrame(const Vec3& v, const AxisFrame& frame);

}  // namespace core
