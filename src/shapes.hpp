#ifndef SHAPES_HPP
#define SHAPES_HPP

#include <cassert>
#include <memory>
#include <optional>

#include "tiny_obj_loader.h"

#include "geometry.hpp"

struct Material {
	Vec3 albedo = {0.0f, 1.0f, 0.5f};
	float metallic = 0.5f;
	float roughness = 0.5f;
	float ao = 0.1f;
	float indexOfRefraction = 1.52;
};

struct Hit {
	float distance = 0.0f;
	Vec3 position;
	Vec3 normal;
	Material material;

	static const std::optional<Hit> &min(const std::optional<Hit> &a, const std::optional<Hit> &b)
	{
		if (a && b) {
			return a->distance < b->distance ? a : b;
		} else {
			return a ? a : b;
		}
	}
};

//////////////////////////////////////////////////////////////////////

class IShape {
  public:
	virtual std::optional<Hit> intersects(const Ray &ray) const = 0;
	virtual ~IShape() {}
};

class Sphere : public IShape {
  public:
	Vec3 position;
	float radius = 1.0f;

	Material material;

	std::optional<Hit> intersects(const Ray &ray) const override
	{
		// https://viclw17.github.io/2018/07/16/raytracing-ray-sphere-intersection/

		constexpr auto EPSILON = 1e-3;

		const auto centerToOrigin = ray.origin - position;

		// quadratic equation for line-sphere intersection
		const auto a = ray.direction.lengthSquared();
		const auto b = 2.0f * dot(centerToOrigin, ray.direction);
		const auto c = centerToOrigin.lengthSquared() - radius * radius;

		const auto discriminant = b * b - 4.0f * a * c;
		if (discriminant < 0.0f) {
			return {}; // no intersection
		}

		// intersection closer to ray origin
		auto distance = (-b - std::sqrt(discriminant)) / 2.0f / a;

		if (distance < EPSILON) {
			// second intersection (further away)
			distance = (-b + std::sqrt(discriminant)) / 2.0f / a;
		}

		if (distance < EPSILON) {
			return {}; // both intersections behind ray origin
		}

		Hit hit;
		hit.distance = distance;
		hit.position = distance * ray.direction + ray.origin;
		hit.normal = (hit.position - position).normalized();
		hit.material = material;
		return hit;
	}
};

class Triangle : public IShape {
  public:
	Triangle(const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &na, const Vec3 &nb, const Vec3 &nc)
	    : a(a), b(b), c(c), na(na.normalized()), nb(nb.normalized()), nc(nc.normalized())
	{
	}

	const Vec3 a, b, c;
	const Vec3 na, nb, nc;
	Vec3 color = Vec3::one;

	std::optional<Hit> intersects(const Ray &ray) const override
	{
		// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
		// https://github.com/erich666/jgt-code/blob/master/Volume_02/Number_1/Moller1997a/raytri.c

		constexpr auto EPSILON = 1e-3;

		const auto edge1 = b - a;
		const auto edge2 = c - a;

		const auto pvec = cross(ray.direction, edge2);
		const auto det = dot(edge1, pvec);

		if (-EPSILON < det && det < EPSILON) {
			return {}; // ray parallel to triangle
		}

		const auto det_inv = 1.0f / det;
		const auto tvec = ray.origin - a;
		const auto u = det_inv * dot(tvec, pvec);
		if (u < 0.0f || u > 1.0f) {
			return {};
		}

		const auto qvec = cross(tvec, edge1);
		const auto v = det_inv * dot(ray.direction, qvec);
		if (v < 0.0f || u + v > 1.0f) {
			return {};
		}

		const auto t = det_inv * dot(edge2, qvec);
		if (t < EPSILON) {
			return {};
		}

		Hit hit;
		hit.distance = t;
		hit.position = t * ray.direction + ray.origin;
		hit.normal = u * nb + v * nc + (1.0f - u - v) * na;
		return hit;
	}
};

class Mesh : public IShape {
  public:
	Vec3 position;
	Material material;

	std::optional<Hit> intersects(const Ray &ray_) const override
	{
		auto ray = ray_;
		ray.origin -= position;

		if (!m_boundingSphere.intersects(ray)) {
			return {};
		}

		std::optional<Hit> closestHit;
		for (const auto &triangle : m_triangles) {
			closestHit = Hit::min(closestHit, triangle.intersects(ray));
		}

		if (closestHit) {
			closestHit->material = material;
		}

		return closestHit;
	}

	static std::unique_ptr<Mesh> loadObj(const std::string &filename)
	{
		tinyobj::attrib_t attrib;
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> materials;

		std::string error;
		const auto loaded = tinyobj::LoadObj(&attrib, &shapes, &materials, &error, filename.c_str());
		if (!error.empty()) {
			std::cerr << "tinyobjloader error: " << error << "\n";
		}
		if (!loaded) {
			return nullptr;
		}

		auto result = std::make_unique<Mesh>();

		const auto getVertex = [&](size_t index) -> Vec3 {
			return {attrib.vertices[3 * index + 0], attrib.vertices[3 * index + 1], attrib.vertices[3 * index + 2]};
		};
		const auto getNormal = [&](size_t index) -> Vec3 {
			return {attrib.normals[3 * index + 0], attrib.normals[3 * index + 1], attrib.normals[3 * index + 2]};
		};

		for (const auto &shape : shapes) {
			auto indexOffset = 0u;

			for (auto f = 0u; f < shape.mesh.num_face_vertices.size(); f++) {
				assert(shape.mesh.num_face_vertices[f] == 3);

				const auto v0 = getVertex(shape.mesh.indices[indexOffset + 0].vertex_index);
				const auto v1 = getVertex(shape.mesh.indices[indexOffset + 1].vertex_index);
				const auto v2 = getVertex(shape.mesh.indices[indexOffset + 2].vertex_index);

				const auto n0 = getNormal(shape.mesh.indices[indexOffset + 0].normal_index);
				const auto n1 = getNormal(shape.mesh.indices[indexOffset + 1].normal_index);
				const auto n2 = getNormal(shape.mesh.indices[indexOffset + 2].normal_index);

				result->m_triangles.emplace_back(v0, v1, v2, n0, n1, n2);

				indexOffset += 3;
			}
		}

		result->calculateBoundingSphere();
		return result;
	}

	static std::unique_ptr<Mesh> plane(float w, float h)
	{
		auto result = std::make_unique<Mesh>();

		result->m_triangles.emplace_back(Vec3{-w / 2.0f, 0.0f, -h / 2.0f}, //
		                                 Vec3{w / 2.0f, 0.0f, -h / 2.0f},  //
		                                 Vec3{-w / 2.0f, 0.0f, h / 2.0f},  //
		                                 Vec3::up, Vec3::up, Vec3::up);

		result->m_triangles.emplace_back(Vec3{w / 2.0f, 0.0f, -h / 2.0f}, //
		                                 Vec3{w / 2.0f, 0.0f, h / 2.0f},  //
		                                 Vec3{-w / 2.0f, 0.0f, h / 2.0f}, //
		                                 Vec3::up, Vec3::up, Vec3::up);

		result->calculateBoundingSphere();
		return result;
	}

  private:
	void calculateBoundingSphere()
	{
		Vec3 furthestVertex;
		for (const auto &triangle : m_triangles) {
			furthestVertex = max(furthestVertex, max(triangle.a, max(triangle.b, triangle.c)));
		}
		m_boundingSphere.radius = furthestVertex.length();
	}

	std::vector<Triangle> m_triangles;
	Sphere m_boundingSphere;
};

#endif // SHAPES_HPP
