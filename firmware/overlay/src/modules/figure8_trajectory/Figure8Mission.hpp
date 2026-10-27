#pragma once

#include <cmath>
#include <cstdint>

namespace figure8
{

enum class Stage : uint8_t {
	Disabled = 0, Waiting, Ready, Takeoff, Hover, Figure8, ReturnHold, Landing, Touchdown, Complete, Aborted
};

enum class Failure : uint8_t { None = 0, Envelope, Tracking, HoverTimeout, LandTimeout, ContactLost };

struct Config {
	float height{3.f};
	float radius{3.f};
	float period{60.f};
	float takeoff_time{12.f};
	float hover_time{5.f};
	float land_time{18.f};
	float landing_depth{0.3f};
	float max_error{3.f};
	int laps{1};

	bool valid() const
	{
		return std::isfinite(height) && height >= 0.5f && height <= 10.f
		       && std::isfinite(radius) && radius >= 0.2f && radius <= 10.f
		       && std::isfinite(period) && period >= 20.f && period <= 300.f
		       && std::isfinite(takeoff_time) && takeoff_time >= 5.f && takeoff_time <= 60.f
		       && std::isfinite(hover_time) && hover_time >= 2.f && hover_time <= 60.f
		       && std::isfinite(land_time) && land_time >= 8.f && land_time <= 60.f
		       && std::isfinite(landing_depth) && landing_depth > 0.f && landing_depth <= 0.5f
		       && std::isfinite(max_error) && max_error >= 0.5f && max_error <= 10.f
		       && laps >= 1 && laps <= 20;
	}
};

struct Reference {
	float position[3] {};
	float velocity[3] {};
	float acceleration[3] {};
	float yaw{0.f};
};

struct Sample {
	float position[3] {};
	float velocity[3] {};
	bool landed{true};
};

inline void smooth_progress(float elapsed, float duration, float &s, float &ds, float &dds)
{
	const float u = fminf(1.f, fmaxf(0.f, elapsed / duration));
	s = u * u * u * (10.f + u * (-15.f + 6.f * u));
	ds = 30.f * u * u * (1.f - u) * (1.f - u) / duration;
	dds = 60.f * u * (1.f - u) * (1.f - 2.f * u) / (duration * duration);
}

inline Reference bernoulli_reference(const float origin[3], float down, float radius,
				     float theta, float omega, float alpha, float yaw)
{
	const float s = sinf(theta), c = cosf(theta);
	const float s2 = s * s, c2 = c * c;
	const float d = 1.f + s2, d2 = d * d, d3 = d2 * d;
	const float dx = -radius * s * (2.f + c2) / d2;
	const float dy = radius * (c2 - 2.f * s2) / d2;
	const float ddx = -radius * c * (s2 * s2 - 12.f * s2 + 3.f) / d3;
	const float ddy = radius * s * c * (6.f * s2 - 10.f) / d3;
	Reference ref{};
	ref.position[0] = origin[0] + radius * c / d;
	ref.position[1] = origin[1] + radius * s * c / d;
	ref.position[2] = down;
	ref.velocity[0] = dx * omega;
	ref.velocity[1] = dy * omega;
	// Chain rule includes phase acceleration for the smooth mission clock.
	ref.acceleration[0] = ddx * omega * omega + dx * alpha;
	ref.acceleration[1] = ddy * omega * omega + dy * alpha;
	ref.yaw = yaw;
	return ref;
}

inline Reference figure8_reference(const float origin[3], const Config &c, float elapsed, float yaw)
{
	float s, ds, dds;
	smooth_progress(elapsed, c.period, s, ds, dds);
	constexpr float two_pi = 6.28318530718f;
	return bernoulli_reference(origin, origin[2] - c.height, c.radius,
				   two_pi * s - 0.25f * two_pi, two_pi * ds, two_pi * dds, yaw);
}

class Mission
{
public:
	void start(const Config &config, const float origin[3], float yaw)
	{
		*this = Mission{};
		_config = config;

		for (int i = 0; i < 3; ++i) { _origin[i] = origin[i]; }

		_yaw = yaw;
		_stage = Stage::Takeoff;
	}

	void translate(const float delta[3], float delta_yaw)
	{
		for (int i = 0; i < 3; ++i) {
			_origin[i] += delta[i];
			_touchdown[i] += delta[i];
		}

		_yaw = atan2f(sinf(_yaw + delta_yaw), cosf(_yaw + delta_yaw));
	}

	Stage stage() const { return _stage; }
	Failure failure() const { return _failure; }
	bool airborne_seen() const { return _airborne_seen; }
	int lap() const { return _lap; }
	float stage_elapsed(float time) const { return time - _stage_start; }
	float height(const Sample &s) const { return _origin[2] - s.position[2]; }
	float error() const { return _error; }

	Reference step(float time, const Sample &sample)
	{
		const float elapsed = stage_elapsed(time);
		const float h = height(sample);
		const float speed = norm(sample.velocity);

		if (!sample.landed && h > 0.3f) { _airborne_seen = true; }

		Reference ref{};

		for (int i = 0; i < 3; ++i) { ref.position[i] = _origin[i]; }

		ref.position[2] -= _config.height;
		ref.yaw = _yaw;

		if (h > _config.height + 2.f || h < -1.f
		    || hypotf(sample.position[0] - _origin[0], sample.position[1] - _origin[1])
		    > _config.radius + _config.max_error) {
			abort(Failure::Envelope);
			return ref;
		}

		switch (_stage) {
		case Stage::Takeoff: {
				float s, ds, dds;
				smooth_progress(elapsed, _config.takeoff_time, s, ds, dds);
				ref.position[2] = _origin[2] - _config.height * s;
				ref.velocity[2] = -_config.height * ds;
				ref.acceleration[2] = -_config.height * dds;

				if (elapsed >= _config.takeoff_time) { transition(Stage::Hover, time); }

				break;
			}

		case Stage::Hover:
		case Stage::ReturnHold: {
				const float delta[3] {sample.position[0] - ref.position[0], sample.position[1] - ref.position[1],
						      sample.position[2] - ref.position[2]
						     };
				const bool stable = _airborne_seen && !sample.landed && norm(delta) <= 0.5f && speed <= 0.4f;

				if (stable) {
					if (_settled_since < 0.f) { _settled_since = time; }

				} else { _settled_since = -1.f; }

				if (elapsed >= _config.hover_time && _settled_since >= 0.f && time - _settled_since >= 2.f) {
					transition(_stage == Stage::Hover ? Stage::Figure8 : Stage::Landing, time);

				} else if (elapsed > _config.hover_time + 30.f) { abort(Failure::HoverTimeout); }

				break;
			}

		case Stage::Figure8:
			ref = figure8_reference(_origin, _config, elapsed, _yaw);

			if (elapsed >= _config.period) {
				if (_lap < _config.laps) { ++_lap; transition(Stage::Figure8, time); }

				else { transition(Stage::ReturnHold, time); }
			}

			break;

		case Stage::Landing: {
				// A downward reference through contact is needed by PX4's land detector.
				const float distance = _config.height + _config.landing_depth;
				const float terminal_speed = fmaxf(0.35f, 2.f * distance / _config.land_time);
				const float ramp = 2.f * distance / terminal_speed;
				const float u = fminf(1.f, fmaxf(0.f, elapsed / ramp));
				ref.position[2] += distance * (2.f * u * u * u - u * u * u * u)
						   + terminal_speed * fmaxf(0.f, elapsed - ramp);
				ref.velocity[2] = terminal_speed * (3.f * u * u - 2.f * u * u * u);
				ref.acceleration[2] = terminal_speed * 6.f * u * (1.f - u) / ramp;

				if (_airborne_seen && sample.landed && fabsf(h) < 0.5f && speed < 0.3f) {
					if (_landed_since < 0.f) { _landed_since = time; }

					if (time - _landed_since >= 1.f) {
						for (int i = 0; i < 3; ++i) { _touchdown[i] = sample.position[i]; }

						transition(Stage::Touchdown, time);
					}

				} else { _landed_since = -1.f; }

				if (elapsed > _config.land_time + 30.f) { abort(Failure::LandTimeout); }

				break;
			}

		case Stage::Touchdown:
			for (int i = 0; i < 3; ++i) { ref.position[i] = _touchdown[i]; }

			if (!sample.landed) { abort(Failure::ContactLost); }

			else if (elapsed > 10.f) { abort(Failure::LandTimeout); }

			break;

		default:
			break;
		}

		const float delta[3] {sample.position[0] - ref.position[0], sample.position[1] - ref.position[1],
				      sample.position[2] - ref.position[2]
				     };
		_error = norm(delta);

		if (_error > _config.max_error) { abort(Failure::Tracking); }

		return ref;
	}

private:
	static float norm(const float v[3]) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
	void abort(Failure failure) { _stage = Stage::Aborted; _failure = failure; }
	void transition(Stage stage, float time) { _stage = stage; _stage_start = time; _settled_since = -1.f; }
	Config _config{};
	Stage _stage{Stage::Waiting};
	Failure _failure{Failure::None};
	float _origin[3] {};
	float _touchdown[3] {};
	float _yaw{0.f};
	float _stage_start{0.f};
	float _settled_since{-1.f};
	float _landed_since{-1.f};
	float _error{0.f};
	int _lap{1};
	bool _airborne_seen{false};
};

} // namespace figure8
