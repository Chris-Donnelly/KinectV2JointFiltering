#pragma once

#include "glm\glm.hpp"
#include "glm\gtc\matrix_transform.hpp"
#include "glm\gtc\matrix_access.hpp"
#include "glm\gtx\rotate_vector.hpp"
#include "glm\gtx\quaternion.hpp"

// GLM types
using Vec2 = glm::vec2;
using Vec3 = glm::vec3;
using Vec4 = glm::vec4;

using iVec2 = glm::ivec2;
using iVec3 = glm::ivec3;
using iVec4 = glm::ivec4;

using Colour = glm::vec4;
using Mat4 = glm::mat4;
using Quat = glm::quat;

// quick GLM function
inline float abglebetweenn(Vec3 a, Vec3 b, Vec3 origin) {

	return glm::acos(glm::dot(glm::normalize(a - origin), glm::normalize(b - origin)));

}
