#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

// pair.first is min
// pair.second is max
using MinMax = std::pair<float, float>;

template<class Container>
MinMax projectBoxOnNormal(const Container& vertices, const glm::vec2& normal) {
	using float_prop = std::numeric_limits<float>;
	MinMax minMax = { glm::dot(vertices[0], normal), -float_prop::max() };

	for (size_t i = 1; i < vertices.size(); i++) {
		float dot = glm::dot(vertices[i], normal);

		if (dot < minMax.first) {
			minMax.first = dot;
		}
		else if (dot > minMax.second) {
			minMax.second = dot;
		}
	}

	return minMax;
}

template<class VertexContainer, class NormalContainer>
bool satHalfTest(const VertexContainer& vertices1, const VertexContainer& vertices2, const NormalContainer& normals, glm::vec2& oNormal, float& oDepth) {
	for (const glm::vec2& normal : normals) {
		MinMax proj1 = projectBoxOnNormal(vertices1, normal);
		MinMax proj2 = projectBoxOnNormal(vertices2, normal);

		if (!(proj1.second >= proj2.first && proj2.second >= proj1.first)) {
			// they are not colliding
			return false;
		}
		else {
			float newDepth = std::max(0.0f, std::min(proj1.second, proj2.second) - std::max(proj1.first, proj2.first));

			if (newDepth <= oDepth) {
				oNormal = normal;
				oDepth = newDepth;

				// we check to see if this value is on the left or right side of proj1
				float direction = proj1.second - proj2.second;
				if (direction > 0) {
					oNormal *= -1.0f;
				}
			}
		}
	}

	return true;
}

template<class T>
float cross(const glm::vec<2, T>& a, const glm::vec<2, T>& b) {
	// a.x * b.y - b.x * a.y
	return (a.x * b.y) - (a.y * b.x);
}

// describes a linear line (straight line) in general form i.e. Ax + By + C = 0
// can handle vertical lines, horizontal lines, etc. for intersection calculations
struct LinearGeneralForm {
	LinearGeneralForm(glm::vec2 p1, glm::vec2 p2)  {
		a = p2.y - p1.y;
		b = -(p2.x - p1.x);
		c = p1.y * p2.x - p1.x * p2.y;
	}

	bool intersects(const LinearGeneralForm& other, glm::vec2& intersectPoint) {
		float denom = a * other.b - other.a * b;
		if(denom == 0.0f) { // will only happen if the lines are parallel
			intersectPoint = {0.0f, 0.0f};
			return false;
		}

		intersectPoint.x = (b * other.c - other.b * c) / denom;
		intersectPoint.y = (c * other.a - other.c * a) / denom;

		return true;
	}

	float a;
	float b;
	float c;
};

// 0 on the line
// >0 on one side
// <0 on the other side
float sideOf(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
	return cross(a - b, p) + cross(b, a);
}

// Vertices are expected to be in CCW order
template<class Container, class OutputContainer = std::vector<glm::vec2>>
OutputContainer sutherlandHodgmanClip(const Container& subjectVertices, const Container& clipVertices) {
	OutputContainer outputVertices(subjectVertices.begin(), subjectVertices.end());
	OutputContainer inputVertices;

	glm::vec2 prevClipVertex = clipVertices.back();
	for(glm::vec2 curClipVertex : clipVertices) {
		LinearGeneralForm clipLine(prevClipVertex, curClipVertex);

		inputVertices = outputVertices;
		outputVertices.clear();

		if(inputVertices.empty())
			break;

		glm::vec2 prevVertex = inputVertices.back();
		for(glm::vec2 curVertex : inputVertices) {
			LinearGeneralForm subLine(prevVertex, curVertex);

			bool curVertexInside = sideOf(curVertex, prevClipVertex, curClipVertex) <= 0.0f;
			bool prevVertexInside = sideOf(prevVertex, prevClipVertex, curClipVertex) <= 0.0f;

			if(curVertexInside) {
				if(!prevVertexInside) {
					clipLine.intersects(subLine, outputVertices.emplace_back());
				}

				outputVertices.push_back(curVertex);
			} else if(prevVertexInside) {
				clipLine.intersects(subLine, outputVertices.emplace_back());
			}


			prevVertex = curVertex;
		}

		prevClipVertex = curClipVertex;
	}

	return outputVertices;
}