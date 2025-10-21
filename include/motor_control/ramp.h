enum class RampType { None, Linear, SCurve };

struct Ramp {
    RampType type = RampType::Linear;
    float time_ms = 0.2f; // in seconds, default

    // current value moves to target; returns new current value
    static float step(RampType type, float current, float target, float time_ms, float dt);

    
};