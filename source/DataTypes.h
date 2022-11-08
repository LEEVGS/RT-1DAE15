#pragma once
#include <cassert>

#include "Math.h"
#include "vector"

namespace dae
{
#pragma region GEOMETRY
	struct Sphere
	{
		Vector3 origin{};
		float radius{};

		unsigned char materialIndex{ 0 };
	};

	struct Plane
	{
		Vector3 origin{};
		Vector3 normal{};

		unsigned char materialIndex{ 0 };
	};

	enum class TriangleCullMode
	{
		FrontFaceCulling,
		BackFaceCulling,
		NoCulling
	};

	struct Triangle
	{
		Triangle() = default;
		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2, const Vector3& _normal):
			v0{_v0}, v1{_v1}, v2{_v2}, normal{_normal.Normalized()}{}

		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }
		{
			const Vector3 edgeV0V1 = v1 - v0;
			const Vector3 edgeV0V2 = v2 - v0;
			normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();
		}

		Vector3 v0{};
		Vector3 v1{};
		Vector3 v2{};

		Vector3 normal{};

		TriangleCullMode cullMode{};
		unsigned char materialIndex{};
	};
	struct BVHNode
	{
		Vector3 aabbMin{};
		Vector3 aabbMax{};
		uint32_t leftChild{};
		uint32_t firstIndice{};
		uint32_t indicesCount{};
		bool IsLeaf() { return indicesCount > 0; };
	};

	struct AABB
	{
		Vector3 min{ Vector3::One * FLT_MAX };
		Vector3 max{ Vector3::One * FLT_MIN };
		void Grow(const Vector3& point)
		{
			min = Vector3::Min(min, point);
			max = Vector3::Max(max, point);
		}
		void Grow(const AABB& bounds)
		{
			min = Vector3::Min(min, bounds.min);
			max = Vector3::Max(max, bounds.max);
		}
		float GetArea()
		{
			Vector3 boxSize{ max - min };
			return boxSize.x * boxSize.y + boxSize.y * boxSize.z + boxSize.z * boxSize.x;
		}
	};

	struct Bin
	{
		AABB bounds{};
		int indicesCount{};
	};
	struct TriangleMesh
	{
		TriangleMesh() = default;
		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, TriangleCullMode _cullMode):
		positions(_positions), indices(_indices), cullMode(_cullMode)
		{
			//Calculate Normals
			CalculateNormals();

			//Update Transforms
			UpdateTransforms();
		}

		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, const std::vector<Vector3>& _normals, TriangleCullMode _cullMode) :
			positions(_positions), indices(_indices), normals(_normals), cullMode(_cullMode)
		{
			UpdateTransforms();
		}

		~TriangleMesh()
		{
			delete[] pBvhNodes;
		}

		std::vector<Vector3> positions{};
		std::vector<Vector3> normals{};
		std::vector<int> indices{};
		unsigned char materialIndex{};

		TriangleCullMode cullMode{TriangleCullMode::BackFaceCulling};

		Matrix rotationTransform{};
		Matrix translationTransform{};
		Matrix scaleTransform{};

		Vector3 minAABB;
		Vector3 maxAABB;

		Vector3 transformedMinAABB;
		Vector3 transformedMaxAABB;

		std::vector<Vector3> transformedPositions{};
		std::vector<Vector3> transformedNormals{};

		BVHNode* pBvhNodes{};
		uint32_t startBvhNodeIndx{};
		uint32_t bvhNodesUsed{};

		void Translate(const Vector3& translation)
		{
			translationTransform = Matrix::CreateTranslation(translation);
		}

		void RotateY(float yaw)
		{
			rotationTransform = Matrix::CreateRotationY(yaw);
		}

		void Scale(const Vector3& scale)
		{
			scaleTransform = Matrix::CreateScale(scale);
		}

		void AppendTriangle(const Triangle& triangle, bool ignoreTransformUpdate = false)
		{
			int startIndex = static_cast<int>(positions.size());

			positions.push_back(triangle.v0);
			positions.push_back(triangle.v1);
			positions.push_back(triangle.v2);

			indices.push_back(startIndex);
			indices.push_back(++startIndex);
			indices.push_back(++startIndex);

			normals.push_back(triangle.normal);

			//Not ideal, but making sure all vertices are updated
			if(!ignoreTransformUpdate)
				UpdateTransforms();
		}

		void CalculateNormals()
		{
			normals.resize(indices.size() / 3);
			for (int i = 0; i < indices.size(); i+=3)
			{
				normals[i/3] = Vector3::Cross(positions[indices[i+1]] - positions[indices[i]], positions[indices[i+2]] - positions[indices[i]]).Normalized();
			}
		}
		void UpdateTransforms()
		{
			const Matrix finalTransformation{ scaleTransform * rotationTransform * translationTransform };

			transformedPositions.resize(positions.size());
			transformedNormals.resize(normals.size());

			for (int i = 0; i < positions.size(); i++)
			{
				transformedPositions[i] = finalTransformation.TransformPoint(positions[i]);
			}
			for (int i = 0; i < normals.size(); i++)
			{
				transformedNormals[i] = finalTransformation.TransformVector(normals[i]).Normalized();
			}
			UpdateTransformedAABB(finalTransformation);

			UpdateBVH();
		}
		void UpdateAABB()
		{
			if (positions.size() > 0)
			{
				minAABB = positions[0];
				maxAABB = positions[0];
				for (auto& p : positions)
				{
					minAABB = Vector3::Min(p, minAABB);
					maxAABB = Vector3::Max(p, maxAABB);
				}
			}
		}
		void UpdateTransformedAABB(const Matrix& finalTransform)
		{
			// AABB Update: be careful -> transform the 8 vertices of the aabb
			// and calculate the new min and max
			Vector3 tMinAABB = finalTransform.TransformPoint(minAABB);
			Vector3 tMaxAABB = tMinAABB;
			// (xmax, ymin, zmin)
			Vector3 tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmax, ymin, zmax)
			tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmin, ymin, zmax)
			tAABB = finalTransform.TransformPoint(minAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (minx, ymax, zmin)
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmax, ymax, zmin)
			tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmax, ymax, zmax)
			tAABB = finalTransform.TransformPoint(maxAABB);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmin, ymax, zmax)
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			transformedMinAABB = tMinAABB;
			transformedMaxAABB = tMaxAABB;
		}
		inline void UpdateBVH()
		{
			if (!pBvhNodes)
			{
				pBvhNodes = new BVHNode[indices.size()]{};
			}
			BVHNode& startNode = pBvhNodes[startBvhNodeIndx];
			bvhNodesUsed = 0;
			startNode.leftChild = 0;
			startNode.firstIndice = 0;
			startNode.indicesCount = indices.size();

			UpdateBVHNodeBounds(startBvhNodeIndx);
			Subdivide(startBvhNodeIndx);
		}
		inline void UpdateBVHNodeBounds(int nodeIndx)
		{
			BVHNode& node = pBvhNodes[nodeIndx];
			node.aabbMin = Vector3::One * FLT_MAX;
			node.aabbMax = Vector3::One * FLT_MIN;

			for (uint32_t i = node.firstIndice; i < node.firstIndice + node.indicesCount; i++)
			{
				Vector3& curVertex = transformedPositions[indices[i]];
				node.aabbMin = Vector3::Min(node.aabbMin, curVertex);
				node.aabbMax = Vector3::Max(node.aabbMax, curVertex);
			}
		}
		inline void Subdivide(int nodeIndx)
		{
			BVHNode& node = pBvhNodes[nodeIndx];
			if (node.indicesCount < 1)
			{
				return;
			}
			int axis;
			float splitPos;
			float cost = FindBestSplitPlane(node, axis, splitPos);
			float noSplitCost = CalculateNodeCost(node);
			if (cost >= noSplitCost)
			{
				return;
			}
			uint32_t i = node.firstIndice;
			uint32_t j = i+node.indicesCount-1;
			while (i <= j)
			{
				Vector3 centroid = (transformedPositions[indices[i]] + transformedPositions[indices[i + 1]] + transformedPositions[indices[i + 2]]) / 3.0f;
				if (centroid[axis] < splitPos)
				{
					i += 3;
				}
				else
				{
					std::swap(indices[i], indices[j - 2]);
					std::swap(indices[i + 1], indices[j - 1]);
					std::swap(indices[i + 2], indices[j]);
					std::swap(normals[i / 3], normals[(j - 2) / 3]);
					std::swap(transformedNormals[i / 3], transformedNormals[(j - 2) / 3]);
					j -= 3;
				}
			}
			uint32_t leftCount = i - node.firstIndice;
			if (leftCount == 0 || leftCount == node.indicesCount)
			{
				return;
			}
			uint32_t leftChildIndx = ++bvhNodesUsed;
			uint32_t rightChildIndx = ++bvhNodesUsed;
			node.leftChild = leftChildIndx;
			pBvhNodes[leftChildIndx].firstIndice = node.firstIndice;
			pBvhNodes[leftChildIndx].indicesCount = leftCount;
			pBvhNodes[rightChildIndx].firstIndice = i;
			pBvhNodes[rightChildIndx].indicesCount = node.indicesCount - leftCount;
			node.indicesCount = 0;

			UpdateBVHNodeBounds(leftChildIndx);
			UpdateBVHNodeBounds(rightChildIndx);

			Subdivide(leftChildIndx);
			Subdivide(rightChildIndx);
		}
		inline float FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos) const
		{
			float bestCost = FLT_MAX;
			for (int a = 0; a < 3; a++)
			{
				float boundsMin = FLT_MAX ;
				float boundsMax = FLT_MIN ;
				Vector3 v0, v1, v2, centroid;

				for (uint32_t i = 0; i < node.indicesCount; i+=3)
				{
					v0 = transformedPositions[indices[node.firstIndice + i]];
					v1 = transformedPositions[indices[node.firstIndice + i + 1]];
					v2 = transformedPositions[indices[node.firstIndice + i + 2]];

					centroid = (v0 + v1 + v2) / 3.f;
					boundsMin = std::min(centroid[a], boundsMin);
					boundsMax = std::max(centroid[a], boundsMax);
				}

				if (abs(boundsMin - boundsMax) < FLT_EPSILON)
				{
					continue;
				}

				const int nrBins = 8 ;
				Bin bins[nrBins];
				float scale = nrBins / (boundsMax - boundsMin);

				for (uint32_t i = 0; i < node.indicesCount; i += 3)
				{
					v0 = transformedPositions[indices[node.firstIndice + i]] ;
					v1 = transformedPositions[indices[node.firstIndice + i + 1]];
					v2 = transformedPositions[indices[node.firstIndice + i + 2]];
					centroid = (v0 + v1 + v2) / 3.0f;

					int binIdx{ std::min(nrBins - 1, static_cast<int>((centroid[a] - boundsMin) * scale)) };
					bins[binIdx].indicesCount += 3;
					bins[binIdx].bounds.Grow(v0);
					bins[binIdx].bounds.Grow(v1);
					bins[binIdx].bounds.Grow(v2);
				}

				float leftArea[nrBins - 1]{};
				float rightArea[nrBins - 1]{};
				float leftCount[nrBins - 1]{};
				float rightCount[nrBins - 1]{};

				AABB leftBox;
				AABB rightBox;
				float leftSum{};
				float rightSum{};
				for (uint32_t i = 0; i < nrBins - 1; ++i)
				{
					leftSum += bins[i].indicesCount;
					leftCount[i] = leftSum;
					leftBox.Grow(bins[i].bounds);
					leftArea[i] = leftBox.GetArea();

					rightSum += bins[nrBins - 1 - i].indicesCount;
					rightCount[nrBins - 2 - i] = rightSum;
					rightBox.Grow(bins[nrBins - 1 - i].bounds);
					rightArea[nrBins - 2 - i] = rightBox.GetArea();
				}
				scale = (boundsMax - boundsMin) / nrBins;
				for (uint32_t i = 0; i < nrBins - 1; ++i)
				{
					const float planeCost{ leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i] };

					if (planeCost < bestCost)
					{
						splitPos = boundsMin + scale * (i + 1);
						axis = a;
						bestCost = planeCost;
					}
				}
			}
			return bestCost;
		}
		inline float CalculateNodeCost(const BVHNode& node) const
		{
			Vector3 boxSize = node.aabbMax - node.aabbMin;
			float parentArea = boxSize.x * boxSize.y + boxSize.y * boxSize.z + boxSize.z * boxSize.x;
			return node.indicesCount * parentArea;
		}
	};
#pragma endregion
#pragma region LIGHT
	enum class LightType
	{
		Point,
		Directional
	};

	struct Light
	{
		Vector3 origin{};
		Vector3 direction{};
		ColorRGB color{};
		float intensity{};

		LightType type{};
	};
#pragma endregion
#pragma region MISC
	struct Ray
	{
		Vector3 origin{};
		Vector3 direction{};
		Vector3 invertedDirection{};

		float min{ 0.0001f };
		float max{ FLT_MAX };
	};

	struct HitRecord
	{
		Vector3 origin{};
		Vector3 normal{};
		float t = FLT_MAX;

		bool didHit{ false };
		unsigned char materialIndex{ 0 };
	};
#pragma endregion
}