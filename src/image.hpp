#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <string>
#include <vector>

#include "geometry.hpp"

class Image {
  public:
	Image(int width, int height) : width(width), height(height) { m_data.resize(width * height); }

	const int width, height;

	const Vec3 &pixel(int x, int y) const { return m_data[y * width + x]; }
	Vec3 &pixel(int x, int y) { return m_data[y * width + x]; }

	const std::vector<Vec3> &data() const { return m_data; }

	void savePng(const std::string &filename) const;

  private:
	std::vector<Vec3> m_data;
};

#endif // IMAGE_HPP
