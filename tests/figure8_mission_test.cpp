#include "Figure8Mission.hpp"
#include "px4_reference_generator/lemniscate_trajectory.hpp"

#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <limits>

using namespace figure8;

static void near(float actual, float expected, float tolerance = 1e-4f)
{
	if (fabsf(actual - expected) > tolerance) {
		fprintf(stderr, "actual %.9g expected %.9g tolerance %.9g\n", actual, expected, tolerance);
	}

	assert(fabsf(actual - expected) <= tolerance);
}

static void reference_tests()
{
	Config c;
	assert(c.valid());
	c.period = std::numeric_limits<float>::quiet_NaN();
	assert(!c.valid());
	c = Config{};
	c.laps = 0;
	assert(!c.valid());
	c = Config{};
	const float origin[3] {5.f, -3.f, 1.f};

	for (float time : {0.f, c.period, c.period + 10.f}) {
		const auto r = figure8_reference(origin, c, time, 0.7f);
		near(r.position[0], origin[0]); near(r.position[1], origin[1]);
		near(r.position[2], origin[2] - c.height);

		for (int i = 0; i < 3; ++i) { near(r.velocity[i], 0.f); near(r.acceleration[i], 0.f); }

		near(r.yaw, 0.7f);
	}

	for (float time = 1.f; time < c.period - 1.f; time += 0.37f) {
		constexpr float dt = 0.05f;
		const auto a = figure8_reference(origin, c, time - dt, 0.f);
		const auto b = figure8_reference(origin, c, time, 0.f);
		const auto d = figure8_reference(origin, c, time + dt, 0.f);

		for (int i = 0; i < 3; ++i) {
			near((d.position[i] - a.position[i]) / (2.f * dt), b.velocity[i], 0.001f);
			near((d.velocity[i] - a.velocity[i]) / (2.f * dt), b.acceleration[i], 0.001f);
		}
	}
}

static void full_mission_tests()
{
	for (int laps : {1, 2}) {
		Config config;
		config.laps = laps;
		Mission mission;
		const float origin[3] {};
		mission.start(config, origin, 0.4f);
		Sample sample;
		bool seen[11] {};
		bool touchdown = false;
		int highest_lap = 1;

		for (int step = 0; step < 15000; ++step) {
			const auto ref = mission.step(step * 0.02f, sample);
			assert(mission.failure() == Failure::None);
			seen[static_cast<int>(mission.stage())] = true;
			highest_lap = mission.lap() > highest_lap ? mission.lap() : highest_lap;

			if (mission.stage() == Stage::Touchdown) { touchdown = true; break; }

			for (int i = 0; i < 3; ++i) {
				sample.position[i] = ref.position[i]; sample.velocity[i] = ref.velocity[i];
			}

			if (sample.position[2] > 0.f) { sample.position[2] = 0.f; sample.velocity[2] = 0.f; }

			sample.landed = sample.position[2] >= -0.01f;
		}

		assert(touchdown && mission.airborne_seen() && highest_lap == laps);

		for (Stage stage : {Stage::Takeoff, Stage::Hover, Stage::Figure8, Stage::ReturnHold, Stage::Landing}) {
			assert(seen[static_cast<int>(stage)]);
		}
	}
}

static void bernoulli_release_tests()
{
	const float origin[3] {5.f, -3.f, 1.f};
	constexpr float radius = 3.f;
	constexpr float omega = 0.2f;
	px4_reference_generator::LemniscateTrajectory release(radius, omega, -2.f, origin[0], origin[1]);

	for (float time = 0.f; time < 90.f; time += 0.13f) {
		const auto expected = release.sample(time);
		const auto actual = bernoulli_reference(origin, -2.f, radius,
							omega * time - 1.570796326795f, omega, 0.f, 0.7f);
		near(actual.position[0], expected.x, 1e-5f);
		near(actual.position[1], expected.y, 1e-5f);
		near(actual.velocity[0], expected.vx, 1e-5f);
		near(actual.velocity[1], expected.vy, 1e-5f);
		near(actual.acceleration[0], expected.ax, 1e-5f);
		near(actual.acceleration[1], expected.ay, 1e-5f);
	}

	Config config;
	float xmax = 0.f, ymax = 0.f;

	for (int sample = 0; sample <= 12000; ++sample) {
		const auto ref = figure8_reference(origin, config, sample * 0.005f, 0.f);
		const float x = ref.position[0] - origin[0], y = ref.position[1] - origin[1];
		const float r2 = x * x + y * y;
		near(r2 * r2, radius * radius * (x * x - y * y), 0.002f);
		xmax = fmaxf(xmax, fabsf(x)); ymax = fmaxf(ymax, fabsf(y));
	}

	near(xmax, radius);
	near(ymax, radius / (2.f * sqrtf(2.f)));
}

static void failure_and_reset_tests()
{
	const float origin[3] {};
	Mission m;
	Sample s;
	m.start(Config{}, origin, 0.f);
	s.position[2] = -5.1f;
	m.step(1.f, s);
	assert(m.failure() == Failure::Envelope);

	m.start(Config{}, origin, 0.f);
	s = Sample{};
	s.position[0] = 3.1f;
	m.step(0.f, s);
	assert(m.failure() == Failure::Tracking);

	m.start(Config{}, origin, 0.f);
	Config config;
	config.max_error = 10.f;
	m.start(config, origin, 0.f);
	s = Sample{};
	m.step(12.f, s);
	assert(m.stage() == Stage::Hover);
	m.step(48.f, s);
	assert(m.failure() == Failure::HoverTimeout && !m.airborne_seen());

	m.start(Config{}, origin, 0.f);
	const float delta[3] {10.f, -5.f, 2.f};
	m.translate(delta, 0.3f);
	s = Sample{{10.f, -5.f, 2.f}, {}, true};
	const auto r = m.step(0.f, s);

	for (int i = 0; i < 3; ++i) { near(r.position[i], delta[i]); }

	near(r.yaw, 0.3f);
	assert(m.failure() == Failure::None);
}

int main()
{
	reference_tests();
	bernoulli_release_tests();
	full_mission_tests();
	failure_and_reset_tests();
	puts("Bernoulli mission: release parity, curve geometry, derivatives, boundaries, full flight, laps, aborts and EKF translation passed");
}
