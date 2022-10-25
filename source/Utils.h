#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			float a, b, c, d{};
			a = Vector3::Dot(ray.direction, ray.direction);
			b = Vector3::Dot(2 * ray.direction, ray.origin - sphere.origin);
			c = Vector3::Dot(ray.origin - sphere.origin, ray.origin - sphere.origin) - (sphere.radius * sphere.radius);
			d = (b * b) - (4 * a * c);
			if (d >= 0.f)
			{
				d = sqrtf(d);
				
				float t1, t2;
				t1 = (-b + d) / (2 * a);
				t2 = (-b - d) / (2 * a);

				if (t1 < 0 && t2 < 0)
				{
					return false;
				}
				if (hitRecord.t > t1)
				{
					if (t1 < 0)
					{
						return false;
					}
					if (t1 < ray.min || t1 > ray.max)
					{
						return false;
					}
					if (ignoreHitRecord)
					{
						return true;
					}
					hitRecord.t = t1;
					hitRecord.materialIndex = sphere.materialIndex;
					hitRecord.didHit = true;
					hitRecord.origin = ray.origin + (ray.direction * hitRecord.t);
					hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
				}
				if (hitRecord.t > t2)
				{
					if (t2 < 0)
					{
						return false;
					}
					if (t2 < ray.min || t2 > ray.max)
					{
						return false;
					}
					if (ignoreHitRecord)
					{
						return true;
					}
					hitRecord.t = t2;
					hitRecord.materialIndex = sphere.materialIndex;
					hitRecord.didHit = true;
					hitRecord.origin = ray.origin + (ray.direction * hitRecord.t);
					hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
				}
			}
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
			float t;
			t = (Vector3::Dot((plane.origin - ray.origin), plane.normal) / Vector3::Dot(ray.direction, plane.normal));
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
					hitRecord.t = t;
					hitRecord.didHit = true;
					hitRecord.materialIndex = plane.materialIndex;
					hitRecord.origin = ray.origin + (ray.direction * hitRecord.t);
					hitRecord.normal = plane.normal;
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
			Vector3 origen = (triangle.v0 + triangle.v1 + triangle.v2) / 3.f;
			Vector3 a = triangle.v0 - triangle.v1;
			Vector3 b = triangle.v0 - triangle.v2;
			Vector3 normal = Vector3::Cross(a, b);

			if (Vector3::Dot(normal, ray.direction) == 0.f)
			{
				return false;
			}

			TriangleCullMode mode = triangle.cullMode;
			if (mode != TriangleCullMode::NoCulling && ignoreHitRecord == true)
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
				if (Vector3::Dot(normal, ray.direction) > 0.f)
				{
					return false;
				}
				break;
			case dae::TriangleCullMode::FrontFaceCulling:
				if (Vector3::Dot(normal, ray.direction) < 0.f)
				{
					return false;
				}
				break;
			default:
				break;
			}

			Vector3 L = origen - ray.origin;
			float t = Vector3::Dot(L, normal) / Vector3::Dot(ray.direction, normal);
			if (t < ray.min || t > ray.max)
			{
				return false;
			}
			Vector3 p = ray.origin + t * ray.direction;

			Vector3 edge = triangle.v1 - triangle.v0;
			Vector3 pointToSide = p - triangle.v0;
			if (Vector3::Dot(normal, Vector3::Cross(edge, pointToSide)) < 0)
			{
				return false;
			}
			edge = triangle.v2 - triangle.v1;
			pointToSide = p - triangle.v1;
			if (Vector3::Dot(normal, Vector3::Cross(edge, pointToSide)) < 0)
			{
				return false;
			}
			edge = triangle.v0 - triangle.v2;
			pointToSide = p - triangle.v2;
			if (Vector3::Dot(normal, Vector3::Cross(edge, pointToSide)) < 0)
			{
				return false;
			}

			if (hitRecord.t > t)
			{
				hitRecord.t = t;
				hitRecord.didHit = true;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.origin = p;
				hitRecord.normal = triangle.normal;
			}

			return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			for (int i = 0; i < mesh.indices.size(); i+=3)
			{
				Triangle temp
				{
					mesh.transformedPositions[mesh.indices[i]],
					mesh.transformedPositions[mesh.indices[i + 1]],
					mesh.transformedPositions[mesh.indices[i + 2]],
					mesh.transformedNormals[i/3]
				};
				temp.materialIndex = mesh.materialIndex;
				temp.cullMode = mesh.cullMode;

				if (HitTest_Triangle(temp, ray, hitRecord, ignoreHitRecord))
				{
					return true;
				}
			}
			return false;
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

	namespace Utils
	{
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

				if(isnan(normal.x))
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
}