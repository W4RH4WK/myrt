#ifndef RANDOM_HPP
#define RANDOM_HPP

#include <random>

namespace detail {
extern std::random_device g_random_device;
extern std::mt19937 g_generator;
extern std::uniform_real_distribution<float> g_uniform_distribution;
extern std::normal_distribution<float> g_normal_distribution;
} // namespace detail

static inline float randFloat()
{
	using namespace detail;
	return g_uniform_distribution(g_generator);
}

static inline float randFloatNormal()
{
	using namespace detail;
	return g_normal_distribution(g_generator);
}

#endif // RANDOM_HPP
