#include "image.hpp"

#include <algorithm>
#include <cstdint>
#include <string>

#include "stb_image_write.h"

void Image::savePng(const std::string &filename) const
{
	const auto f2uc = [](float v) -> uint8_t { return std::clamp(v * 255.0f, 0.0f, 255.0f); };

	std::vector<uint8_t> pixelsRGB;
	pixelsRGB.reserve(width * height * 3);

	for (const auto p : m_data) {
		pixelsRGB.push_back(f2uc(p.x));
		pixelsRGB.push_back(f2uc(p.y));
		pixelsRGB.push_back(f2uc(p.z));
	}

	stbi_write_png(filename.c_str(), width, height, 3, pixelsRGB.data(), 0);
}
