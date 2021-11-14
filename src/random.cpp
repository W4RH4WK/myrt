#include "random.hpp"

namespace detail {

std::random_device g_random_device;

std::mt19937 g_generator(g_random_device());

std::uniform_real_distribution<float> g_uniform_distribution;

std::normal_distribution<float> g_normal_distribution;

} // namespace detail
