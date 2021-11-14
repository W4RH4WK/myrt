#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>

#include "geometry.hpp"
#include "image.hpp"
#include "random.hpp"
#include "shapes.hpp"

using Clock = std::chrono::high_resolution_clock;

struct PointLight {
	Vec3 position;
	Vec3 color = Vec3::one;
	float brightness = 1.0f;
};

struct Camera {
	Vec3 position;
	Quat rotation;

	float fov = 60.0f;

	Ray spawnRay(size_t x, size_t y, size_t width, size_t height) const
	{
		Vec3 turn;
		turn.x = 2.0f * (float(x) / float(width)) - 1.0f;
		turn.y = -2.0f * (float(y) / float(height)) + 1.0f;

		turn = std::tan(fov / 2.0f * deg2rad) * turn;

		const auto aspect = float(width) / float(height);
		turn.x *= aspect;

		return {position, rotation * (Vec3::forward + turn)};
	}
};

//////////////////////////////////////////////////////////////////////

struct Scene {
	Camera camera;

	Vec3 clearColor = {0.0f, 0.1f, 0.1f};

	std::vector<std::unique_ptr<IShape>> shapes;
	std::vector<PointLight> lights;
};

//////////////////////////////////////////////////////////////////////

std::optional<Hit> intersect(const Scene &scene, const Ray &ray);

Vec3 illumination(const Scene &scene, const Ray &ray, const Hit &hit)
{
	// https://learnopengl.com/PBR/Lighting

	const auto N = hit.normal;
	const auto V = -ray.direction;

	Vec3 Lo;
	for (const auto &light : scene.lights) {
		const auto L = (light.position - hit.position).normalized();
		const auto H = (V + L).normalized();

		// shadow ray check
		{
			const auto shadowRay = Ray{hit.position + 0.01f * hit.normal, L};
			if (const auto shadowHit = intersect(scene, shadowRay)) {
				continue;
			}
		}

		const auto distance = (light.position - hit.position).length();
		const auto attenuation = 1.0f / (distance * distance);
		const auto radiance = attenuation * light.brightness * light.color;

		auto F0 = 0.04f * Vec3::one;
		F0 = mix(F0, hit.material.albedo, hit.material.metallic);
		const auto F = fresnelSchlick(std::max(dot(H, V), 0.0f), F0);

		const auto NDF = distributionGGX(N, H, hit.material.roughness);
		const auto G = geometrySmith(N, V, L, hit.material.roughness);

		const auto numerator = NDF * G * F;
		const auto denominator = 4.0f * std::max(dot(N, V), 0.0f) * std::max(dot(N, L), 0.0f);
		const auto specular = numerator / std::max(denominator, 0.001f);

		const auto kS = F;
		auto kD = Vec3::one - kS;
		kD *= 1.0f - hit.material.metallic;

		const auto NdotL = std::max(dot(N, L), 0.0f);
		Lo += NdotL * (kD * hit.material.albedo / PI + specular) * radiance;
	}

	const auto ambient = hit.material.ao * 0.03f * hit.material.albedo;

	auto color = ambient + Lo;

	// gamma correction
	color = color / (color + Vec3::one);
	color = pow(color, 1.0f / 2.2f);

	return color;
}

//////////////////////////////////////////////////////////////////////

std::optional<Hit> intersect(const Scene &scene, const Ray &ray)
{
	std::optional<Hit> closestHit;
	for (const auto &shape : scene.shapes) {
		closestHit = Hit::min(closestHit, shape->intersects(ray));
	}
	return closestHit;
}

const auto MAX_RECURSION_DEPTH = 4;
const auto REFRACTION_ALPHA = 0.5f;

Vec3 traceRay(const Scene &scene, const Ray &ray, int recursionDepth = 0)
{
	if (recursionDepth > MAX_RECURSION_DEPTH) {
		return Vec3::zero;
	}

	const auto closestHit = intersect(scene, ray);
	if (!closestHit) {
		return scene.clearColor;
	}

	auto baseColor = illumination(scene, ray, *closestHit);

	// reflection
	{
		for (auto i = 0; i < 4; i++) {
			const auto baseScatterFactor = 10.0f * deg2rad;
			const auto scatter = Quat::random(baseScatterFactor * closestHit->material.roughness);

			const auto reflectionRay = Ray{closestHit->position, scatter * reflect(ray.direction, closestHit->normal)};
			const auto reflectionColor = traceRay(scene, reflectionRay, recursionDepth + 1);
			baseColor = mix(baseColor, reflectionColor, closestHit->material.metallic);
		}
	}

	// // refraction
	// {
	// 	const auto frontFacing = dot(ray.direction, closestHit->normal) < 0.0f;

	// 	auto eta = closestHit->material.indexOfRefraction;
	// 	if (frontFacing) {
	// 		eta = 1.0f / eta;
	// 	}

	// 	const auto refractionRay = Ray{closestHit->position, refract(ray.direction, closestHit->normal, eta)};
	// 	const auto refractionColor = traceRay(scene, refractionRay, recursionDepth + 1);
	// 	baseColor = mix(baseColor, refractionColor, REFRACTION_ALPHA);
	// }

	return baseColor;
}

int main()
{
	Scene scene;
	scene.camera.position = Vec3{0.0f, 2.0f, -6.0f};

	for (auto x = -1; x < 1; x++) {
		for (auto z = -1; z < 1; z++) {
			const auto spacing = 4.0f;

			auto &light = scene.lights.emplace_back();
			light.position = {spacing * x, 5.0f, spacing * z};
			light.brightness = 1.5f * spacing;
		}
	}

	// scene.camera.rotation = Quat{45.0f * deg2rad, 0.0f, 0.0f};

	{
		auto floor = Mesh::plane(50.0f, 50.0f);
		floor->material.metallic = 0.0f;
		floor->material.roughness = 0.8f;
		floor->material.albedo = 0.5f * Vec3::one;
		scene.shapes.push_back(std::move(floor));
	}

	// Spheres
	for (auto z = 0.0f; z <= 3.0f; z++) {
		// for (auto y = -3.0f; y <= 3.0f; y++) {
		for (auto x = -3.0f; x <= 3.0f; x++) {
			auto sphere = std::make_unique<Sphere>();
			sphere->position = 4.5f * Vec3{x, 0.0f, z} + Vec3{0.0f, 1.0f, 0.0f};
			scene.shapes.push_back(std::move(sphere));
		}
		// }
	}

	// {
	// 	auto sphere = std::make_unique<Sphere>();
	// 	sphere->position = {0.0f, 0.0f, 0.0f};
	// 	scene.shapes.push_back(std::move(sphere));
	// }

	// for (auto i = 0; i < 1; i++) {
	// 	const auto a = Vec3{1.0f, 0.0f, 10.0f};
	// 	const auto b = Vec3{-1.0f, 0.0f, 10.0f};
	// 	const auto c = Vec3{0.0f, 1.0f, 10.0f};
	// 	auto triangle = std::make_unique<Triangle>(a, b, c);
	// 	triangle->color = {1.0f, 1.0f, 1.0f};
	// 	scene.shapes.push_back(std::move(triangle));
	// }

	// {
	// 	auto teapot = Mesh::loadObj("resources/teapot.obj");
	// 	teapot->position = {0.0f, 0.0f, 0.0f};
	// 	scene.shapes.push_back(std::move(teapot));
	// }

	auto image = Image(1600, 900);

	{
		using namespace std::chrono;

		const auto tstart = Clock::now();

#pragma omp parallel for collapse(2)
		for (auto y = 0; y < image.height; y++) {
			for (auto x = 0; x < image.width; x++) {
				auto cameraRay = scene.camera.spawnRay(x, y, image.width, image.height);
				image.pixel(x, y) = traceRay(scene, cameraRay);
			}
		}

		std::cout << "ray tracing: " << duration_cast<milliseconds>(Clock::now() - tstart).count() << " ms\n";
	}

	image.savePng("output.png");
}
