#pragma once
#include <cassert>
#include <fstream>
#include <xmmintrin.h>
#include "Math.h"
#include "DataTypes.h"

namespace dae
{
	namespace Utils
	{
		// Fast Sqrt	Source: https://geometrian.com/programming/tutorials/fastsqrt/index.php
		inline float FastSqrt(float arg) {
			return _mm_cvtss_f32(
				_mm_sqrt_ss(_mm_set_ps1(arg))
			);
		}
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof())
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if (isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			Vector3 toCenter = sphere.origin - ray.origin;
			float distance = Vector3::Dot(toCenter, ray.direction);
			float squaredDistance = toCenter.SqrMagnitude() - Square(distance);
			float squaredSpherePoint = Square(sphere.radius) - squaredDistance;
			if (squaredSpherePoint < 0.f)
			{
				return false;
			}
			float pointToHitPoint = Utils::FastSqrt(squaredSpherePoint);
			float t = distance - pointToHitPoint;
			
			if (t < ray.min || t > ray.max)
			{
				return false;
			}
			if (ignoreHitRecord)
			{
				return true;
			}

			hitRecord.didHit = true;
			if (hitRecord.t > t)
			{
				hitRecord.t = t;
				hitRecord.materialIndex = sphere.materialIndex;
				hitRecord.origin = ray.origin + (ray.direction * hitRecord.t);
				hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
			}
			

#pragma region Analytic
			//float a, b, c, d{};
			//a = Vector3::Dot(ray.direction, ray.direction);
			//b = Vector3::Dot(2 * ray.direction, ray.origin - sphere.origin);
			//c = Vector3::Dot(ray.origin - sphere.origin, ray.origin - sphere.origin) - (sphere.radius * sphere.radius);
			//d = (b * b) - (4 * a * c);
			//if (d >= 0.f)
			//{
			//	d = sqrtf(d);
			//
			//	float t1, t2;
			//	t1 = (-b + d) / (2 * a);
			//	t2 = (-b - d) / (2 * a);
			//
			//	if (t1 < 0 && t2 < 0)
			//	{
			//		return false;
			//	}
			//	if (hitRecord.t > t1)
			//	{
			//		if (t1 < 0)
			//		{
			//			return false;
			//		}
			//		if (t1 < ray.min || t1 > ray.max)
			//		{
			//			return false;
			//		}
			//		if (ignoreHitRecord)
			//		{
			//			return true;
			//		}
			//		hitRecord.t = t1;
			//		hitRecord.materialIndex = sphere.materialIndex;
			//		hitRecord.didHit = true;
			//		hitRecord.origin = ray.origin + (ray.direction * hitRecord.t);
			//		hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
			//	}
			//	if (hitRecord.t > t2)
			//	{
			//		if (t2 < 0)
			//		{
			//			return false;
			//		}
			//		if (t2 < ray.min || t2 > ray.max)
			//		{
			//			return false;
			//		}
			//		if (ignoreHitRecord)
			//		{
			//			return true;
			//		}
			//		hitRecord.t = t2;
			//		hitRecord.materialIndex = sphere.materialIndex;
			//		hitRecord.didHit = true;
			//		hitRecord.origin = ray.origin + (ray.direction * hitRecord.t);
			//		hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
			//	}
			//}
#pragma endregion
			return hitRecord.didHit;
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			float t = (Vector3::Dot((plane.origin - ray.origin), plane.normal) / Vector3::Dot(ray.direction, plane.normal));
			if (t > FLT_EPSILON)
			{
				if (t < 0)
				{
					return false;
				}
				if (t < ray.min || t > ray.max)
				{
					return false;
				}
				if (ignoreHitRecord)
				{
					return true;
				}
				if (hitRecord.t > t)
				{
					hitRecord.didHit = true;
					hitRecord.materialIndex = plane.materialIndex;
					hitRecord.origin = ray.origin + ray.direction * t;
					hitRecord.normal = plane.normal;
					hitRecord.t = t;
				}
			}
			return hitRecord.didHit;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			float normalDot = Vector3::Dot(ray.direction, triangle.normal);

			if (abs(normalDot) < FLT_EPSILON)
			{
				return false;
			}

			TriangleCullMode mode = triangle.cullMode;
			if (ignoreHitRecord && mode != TriangleCullMode::NoCulling)
			{
				if (mode == TriangleCullMode::FrontFaceCulling)
				{
					mode = TriangleCullMode::BackFaceCulling;
				}
				else
				{
					mode = TriangleCullMode::FrontFaceCulling;
				}
			}
			
			switch (mode)
			{
			case dae::TriangleCullMode::BackFaceCulling:
				if (normalDot > 0.f)
				{
					return false;
				}
				break;
			case dae::TriangleCullMode::FrontFaceCulling:
				if (normalDot < 0.f)
				{
					return false;
				}
				break;
			default:
				break;
			}

			//New code (https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm)
			Vector3 edge1, edge2, h, s, q;
			float f, u, v;

			edge1 = triangle.v1 - triangle.v0;
			edge2 = triangle.v2 - triangle.v0;
			h = Vector3::Cross(ray.direction, edge2);
			f = 1.0f / Vector3::Dot(edge1, h);
			s =  ray.origin - triangle.v0;
			u =  f * Vector3::Dot(s,h);
			if (u < 0.0f || u > 1.0f)
			{
				return false;
			}
			q =  Vector3::Cross(s, edge1);
			v =  f * Vector3::Dot(ray.direction, q);
			if (v < 0.0f || u + v > 1.0f)
			{
				return false;
			}
			float t =  f * Vector3::Dot(edge2, q);
			if (t < ray.min || t > ray.max)
			{
				return false;
			}
			if (ignoreHitRecord)
			{
				return true;
			}
			if (hitRecord.t > t)
			{
				hitRecord.didHit = true;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.origin = ray.origin + ray.direction * t;
				hitRecord.normal = triangle.normal;
				hitRecord.t = t;
			}

			return true;
#pragma region LessonCode
			//Vector3 origen = (triangle.v0 + triangle.v1 + triangle.v2) / 3.f;
			//Vector3 a = triangle.v0 - triangle.v1;
			//Vector3 b = triangle.v0 - triangle.v2;
			//Vector3 normal = Vector3::Cross(a, b);
			//
			//if (Vector3::Dot(normal, ray.direction) == 0.f)
			//{
			//	return false;
			//}
			//
			//TriangleCullMode mode = triangle.cullMode;
			//if (mode != TriangleCullMode::NoCulling && ignoreHitRecord == true)
			//{
			//	if (mode == TriangleCullMode::FrontFaceCulling)
			//	{
			//		mode = TriangleCullMode::BackFaceCulling;
			//	}
			//	else
			//	{
			//		mode = TriangleCullMode::FrontFaceCulling;
			//	}
			//}
			//
			//switch (mode)
			//{
			//case dae::TriangleCullMode::BackFaceCulling:
			//	if (Vector3::Dot(normal, ray.direction) > 0.f)
			//	{
			//		return false;
			//	}
			//	break;
			//case dae::TriangleCullMode::FrontFaceCulling:
			//	if (Vector3::Dot(normal, ray.direction) < 0.f)
			//	{
			//		return false;
			//	}
			//	break;
			//default:
			//	break;
			//}
			//
			//Vector3 L = origen - ray.origin;
			//float t = Vector3::Dot(L, normal) / Vector3::Dot(ray.direction, normal);
			//if (t < ray.min || t > ray.max)
			//{
			//	return false;
			//}
			//Vector3 p = ray.origin + t * ray.direction;
			//
			//Vector3 edge = triangle.v1 - triangle.v0;
			//Vector3 pointToSide = p - triangle.v0;
			//if (Vector3::Dot(normal, Vector3::Cross(edge, pointToSide)) < 0)
			//{
			//	return false;
			//}
			//edge = triangle.v2 - triangle.v1;
			//pointToSide = p - triangle.v1;
			//if (Vector3::Dot(normal, Vector3::Cross(edge, pointToSide)) < 0)
			//{
			//	return false;
			//}
			//edge = triangle.v0 - triangle.v2;
			//pointToSide = p - triangle.v2;
			//if (Vector3::Dot(normal, Vector3::Cross(edge, pointToSide)) < 0)
			//{
			//	return false;
			//}
			//
			//if (hitRecord.t > t)
			//{
			//	hitRecord.t = t;
			//	hitRecord.didHit = true;
			//	hitRecord.materialIndex = triangle.materialIndex;
			//	hitRecord.origin = p;
			//	hitRecord.normal = triangle.normal;
			//}
			//
			//return true;
#pragma endregion
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool SlabTest_TriangleMesh(const Ray& ray, const Vector3& minAABB, const Vector3& maxAABB)
		{
			float tx1 = (minAABB.x - ray.origin.x) * ray.invertedDirection.x;
			float tx2 = (maxAABB.x - ray.origin.x) * ray.invertedDirection.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			float ty1 = (minAABB.y - ray.origin.y) * ray.invertedDirection.y;
			float ty2 = (maxAABB.y - ray.origin.y) * ray.invertedDirection.y;

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			float tz1 = (minAABB.z - ray.origin.z) * ray.invertedDirection.z;
			float tz2 = (maxAABB.z - ray.origin.z) * ray.invertedDirection.z;

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;
		}
		inline void IntersectBVH(const TriangleMesh& mesh, const Ray& ray, Triangle& sharedTriangle, HitRecord& hitRecord, bool& hasHit, HitRecord& curClosestHit, bool ignoreHitRecord, unsigned int bvhNodeIdx)
		{
			BVHNode& node = mesh.pBvhNodes[bvhNodeIdx];

			if (!SlabTest_TriangleMesh(ray, node.aabbMin, node.aabbMax))
			{
				return;
			}
			
			if (!node.IsLeaf())
			{
				IntersectBVH(mesh, ray, sharedTriangle, hitRecord, hasHit, curClosestHit, ignoreHitRecord, node.leftChild);
				IntersectBVH(mesh, ray, sharedTriangle, hitRecord, hasHit, curClosestHit, ignoreHitRecord, node.leftChild + 1);
				return;
			}
			for (uint32_t i = 0; i < node.indicesCount; i+=3)
			{
				sharedTriangle.v0 = mesh.transformedPositions[mesh.indices[node.firstIndice + i]];
				sharedTriangle.v1 = mesh.transformedPositions[mesh.indices[node.firstIndice + i + 1]];
				sharedTriangle.v2 = mesh.transformedPositions[mesh.indices[node.firstIndice + i + 2]];
				sharedTriangle.normal = mesh.transformedNormals[(node.firstIndice + i) / 3];

				if (!HitTest_Triangle(sharedTriangle, ray, curClosestHit, ignoreHitRecord))
				{
					continue;
				}
				hasHit = true;

				if (ignoreHitRecord)
				{
					return;
				}

				if (hitRecord.t > curClosestHit.t)
				{
					hitRecord = curClosestHit;
				}
			}
		}
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			HitRecord tempHit{};
			bool hasHit = false;
			
			Triangle tempTriangle{};
			tempTriangle.cullMode = mesh.cullMode;
			tempTriangle.materialIndex = mesh.materialIndex;
			
			IntersectBVH(mesh, ray, tempTriangle, hitRecord, hasHit, tempHit, ignoreHitRecord, 0);

			//Lesson method
			//if (!SlabTest_TriangleMesh(ray, mesh.minAABB, mesh.maxAABB))
			//{
			//	return false;
			//}
			//
			//for (int i = 0; i < mesh.indices.size(); i+=3)
			//{
			//	tempTriangle.v0 = mesh.transformedPositions[mesh.indices[i]];
			//	tempTriangle.v1 = mesh.transformedPositions[mesh.indices[i + 1]];
			//	tempTriangle.v2 = mesh.transformedPositions[mesh.indices[i + 2]];
			//	tempTriangle.normal = mesh.transformedNormals[i / 3];
			//	if (!HitTest_Triangle(tempTriangle, ray, hitRecord, ignoreHitRecord))
			//	{
			//		continue;
			//	}
			//	hasHit = true;
			//	if (ignoreHitRecord)
			//	{
			//		return true;
			//	}
			//}
			return hasHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			return Vector3{ light.origin - origin };
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			return light.color * (light.intensity / (light.origin - target).SqrMagnitude());
		}
	}

	
}