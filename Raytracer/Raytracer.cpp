// Author Alhajras Algdairy, 1.7.2021, Uni-Freiburg
// A simple program that uses ray-tracing to render triangles and spheres
// 

// Copyright (C) 2012  www.scratchapixel.com


#include <cstdio>
#include <utility>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <cstdlib>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sstream>
#include <chrono>
#include "geometry.h"

const float kInfinity = std::numeric_limits<float>::max();
template <> const Matrix44f Matrix44f::kIdentity = Matrix44f();

#define M_PI 3.141592653589793
constexpr float kEpsilon = 1e-8;

inline
float deg2rad(const float& deg)
{
	return deg * M_PI / 180;
}

inline
float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(hi, v));
}

enum MaterialType { DIFFUSE_AND_GLOSSY, REFLECTION_AND_REFRACTION, REFLECTION };

struct Settings
{
	uint32_t width = 640;
	uint32_t height = 480;
	float fov = 90;
	Vec3f backgroundColor = Vec3f(1, 1, 1); // Standard bg color is white
	float bias = 0.0001; // Error allowed
	uint32_t maxDepth = 0; // Max number of ray trating into the scene
	uint32_t aa_samples = 5; // Anti aliasing samples
};

//struct IsectInfo
//{
//	const Object* hitObject = nullptr;
//	float tNear = kInfinity;
//	Vec2f uv;
//	uint32_t index = 0;
//};


class Object
{
public:
	// [comment]
	// Setting up the object-to-world and world-to-object matrix
	// [/comment]
	Object(const Matrix44f& o2w) : objectToWorld(o2w), worldToObject(o2w.inverse()) {}
	virtual ~Object() {}
	virtual bool intersect(const Vec3f&, const Vec3f&, float&, uint32_t&, Vec2f&) const = 0;
	virtual void getSurfaceProperties(const Vec3f&, const Vec3f&, const uint32_t&, const Vec2f&, Vec3f&, Vec2f&) const = 0;
	virtual void displayInfo() const = 0;
	Matrix44f objectToWorld, worldToObject;
	MaterialType type = DIFFUSE_AND_GLOSSY;
	Vec3f albedo = 0.18;
	float Kd = 0.8; // phong model diffuse weight
	float Ks = 0.2; // phong model specular weight
	float n = 10;   // phong specular exponent
	Vec3f BBox[2] = { kInfinity, -kInfinity };
};

bool rayTriangleIntersect(
	const Vec3f& orig, const Vec3f& dir,
	const Vec3f& v0, const Vec3f& v1, const Vec3f& v2,
	float& t, float& u, float& v)
{
	Vec3f v0v1 = v1 - v0;
	Vec3f v0v2 = v2 - v0;
	Vec3f pvec = dir.crossProduct(v0v2);
	float det = v0v1.dotProduct(pvec);

	// ray and triangle are parallel if det is close to 0
	if (fabs(det) < kEpsilon) return false;

	float invDet = 1 / det;

	Vec3f tvec = orig - v0;
	u = tvec.dotProduct(pvec) * invDet;
	if (u < 0 || u > 1) return false;

	Vec3f qvec = tvec.crossProduct(v0v1);
	v = dir.dotProduct(qvec) * invDet;
	if (v < 0 || u + v > 1) return false;

	t = v0v2.dotProduct(qvec) * invDet;

	return (t > 0) ? true : false;
}


class TriangleMesh : public Object
{
public:
	// Build a triangle mesh from a face index array and a vertex index array
	TriangleMesh(
		const Matrix44f& o2w,
		const uint32_t nfaces,
		const std::unique_ptr<uint32_t[]>& faceIndex,
		const std::unique_ptr<uint32_t[]>& vertsIndex,
		const std::unique_ptr<Vec3f[]>& verts,
		std::unique_ptr<Vec3f[]>& normals,
		std::unique_ptr<Vec2f[]>& st,
		bool singleVertAttr = true) :
		Object(o2w),
		numTris(0),
		isSingleVertAttr(singleVertAttr)
	{
		uint32_t k = 0, maxVertIndex = 0;
		// find out how many triangles we need to create for this mesh
		for (uint32_t i = 0; i < nfaces; ++i) {
			numTris += faceIndex[i] - 2;
			for (uint32_t j = 0; j < faceIndex[i]; ++j)
				if (vertsIndex[k + j] > maxVertIndex)
					maxVertIndex = vertsIndex[k + j];
			k += faceIndex[i];
		}
		maxVertIndex += 1;

		// allocate memory to store the position of the mesh vertices
		P = std::unique_ptr<Vec3f[]>(new Vec3f[maxVertIndex]);
		for (uint32_t i = 0; i < maxVertIndex; ++i) {
			objectToWorld.multVecMatrix(verts[i], P[i]);
		}

		// allocate memory to store triangle indices
		trisIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
		Matrix44f transformNormals = worldToObject.transpose();
		// [comment]
		// Sometimes we have 1 vertex attribute per vertex per face. So for example of you have 2
		// quads this would be defefined by 6 vertices but 2 * 4 vertex attribute values for
		// each vertex attribute (normal, tex. coordinates, etc.). But in some cases you may
		// want to have 1 single value per vertex. So in the quad example this would be 6 vertices
		// and 6 vertex attributes values per attribute. We need to provide both option to users.
		// [/comment]
		if (isSingleVertAttr) {
			N = std::unique_ptr<Vec3f[]>(new Vec3f[maxVertIndex]);
			texCoordinates = std::unique_ptr<Vec2f[]>(new Vec2f[maxVertIndex]);
			for (uint32_t i = 0; i < maxVertIndex; ++i) {
				texCoordinates[i] = st[i];
				transformNormals.multDirMatrix(normals[i], N[i]);
			}
		}
		else {
			N = std::unique_ptr<Vec3f[]>(new Vec3f[numTris * 3]);
			texCoordinates = std::unique_ptr<Vec2f[]>(new Vec2f[numTris * 3]);
			for (uint32_t i = 0, k = 0, l = 0; i < nfaces; ++i) { // for each  face
				for (uint32_t j = 0; j < faceIndex[i] - 2; ++j) {
					transformNormals.multDirMatrix(normals[k], N[l]);
					transformNormals.multDirMatrix(normals[k + j + 1], N[l + 1]);
					transformNormals.multDirMatrix(normals[k + j + 2], N[l + 2]);
					N[l].normalize();
					N[l + 1].normalize();
					N[l + 2].normalize();
					texCoordinates[l] = st[k];
					texCoordinates[l + 1] = st[k + j + 1];
					texCoordinates[l + 2] = st[k + j + 2];
				}
				k += faceIndex[i];
			}
		}

		// generate the triangle index array and set normals and st coordinates
		for (uint32_t i = 0, k = 0, l = 0; i < nfaces; ++i) { // for each  face
			for (uint32_t j = 0; j < faceIndex[i] - 2; ++j) { // for each triangle in the face
				trisIndex[l] = vertsIndex[k];
				trisIndex[l + 1] = vertsIndex[k + j + 1];
				trisIndex[l + 2] = vertsIndex[k + j + 2];
				l += 3;
			}
			k += faceIndex[i];
		}
	}
	// Test if the ray interesests this triangle mesh
	bool intersect(const Vec3f& orig, const Vec3f& dir, float& tNear, uint32_t& triIndex, Vec2f& uv) const
	{
		uint32_t j = 0;
		bool isect = false;
		for (uint32_t i = 0; i < numTris; ++i) {
			const Vec3f& v0 = P[trisIndex[j]];
			const Vec3f& v1 = P[trisIndex[j + 1]];
			const Vec3f& v2 = P[trisIndex[j + 2]];
			float t = kInfinity, u, v;
			if (rayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v) && t < tNear) {
				tNear = t;
				uv.x = u;
				uv.y = v;
				triIndex = i;
				isect = true;
			}
			j += 3;
		}

		return isect;
	}
	void getSurfaceProperties(
		const Vec3f& hitPoint,
		const Vec3f& viewDirection,
		const uint32_t& triIndex,
		const Vec2f& uv,
		Vec3f& hitNormal,
		Vec2f& hitTextureCoordinates) const
	{
		uint32_t vai[3]; // vertex attr index
		if (isSingleVertAttr) {
			vai[0] = trisIndex[triIndex * 3];
			vai[1] = trisIndex[triIndex * 3 + 1];
			vai[2] = trisIndex[triIndex * 3 + 2];
		}
		else {
			vai[0] = triIndex * 3;
			vai[1] = triIndex * 3 + 1;
			vai[2] = triIndex * 3 + 2;
		}
		if (smoothShading) {
			// vertex normal
			const Vec3f& n0 = N[vai[0]];
			const Vec3f& n1 = N[vai[1]];
			const Vec3f& n2 = N[vai[2]];
			hitNormal = (1 - uv.x - uv.y) * n0 + uv.x * n1 + uv.y * n2;
		}
		else {
			// face normal
			const Vec3f& v0 = P[trisIndex[triIndex * 3]];
			const Vec3f& v1 = P[trisIndex[triIndex * 3 + 1]];
			const Vec3f& v2 = P[trisIndex[triIndex * 3 + 2]];
			hitNormal = (v1 - v0).crossProduct(v2 - v0);
		}

		// doesn't need to be normalized as the N's are normalized but just for safety
		hitNormal.normalize();

		// texture coordinates
		const Vec2f& st0 = texCoordinates[vai[0]];
		const Vec2f& st1 = texCoordinates[vai[1]];
		const Vec2f& st2 = texCoordinates[vai[2]];
		hitTextureCoordinates = (1 - uv.x - uv.y) * st0 + uv.x * st1 + uv.y * st2;
	}
	void displayInfo() const
	{
		std::cerr << "Number of triangles in this mesh: " << numTris << std::endl;
		std::cerr << BBox[0] << ", " << BBox[1] << std::endl;
	}
	// member variables
	uint32_t numTris;                         // number of triangles
	std::unique_ptr<Vec3f[]> P;              // triangles vertex position
	std::unique_ptr<uint32_t[]> trisIndex;   // vertex index array
	std::unique_ptr<Vec3f[]> N;              // triangles vertex normals
	std::unique_ptr<Vec2f[]> texCoordinates; // triangles texture coordinates
	bool smoothShading = true;                // smooth shading by default
	bool isSingleVertAttr = true;
};


Vec3f evalBezierCurve(const Vec3f* P, const float& t)
{

	// n = 4
//   float b0 = (1 - t) * (1 - t) * (1 - t) * (1 - t);
   //float b1 = 4 * t * (1 - t) * (1 - t) * (1 - t);
   //float b2 = 6 * t * t * (1 - t) * (1 - t);
   //float b3 = 4 * t * t * t * (1 - t);
   //float b4 = t * t * t * t;

   // n = 3
   //float b0 = (1 - t) * (1 - t) * (1 - t);
   //float b1 = 3 * t * (1 - t) * (1 - t);
   //float b2 = 3 * t * t * (1 - t);
   //float b3 = t * t * t;

   // n = 2
   //float b0 = (1 - t) * (1 - t);
   //float b1 = 2 * t * (1 - t);
   //float b2 = t * t;

   // n = 1
	float b0 = (1 - t);
	float b1 = t;


	return P[0] * b0 + P[1] * b1;
}


Vec3f derivBezier(const Vec3f* P, const float& t)
{
	return -3 * (1 - t) * (1 - t) * P[0] +
		(3 * (1 - t) * (1 - t) - 6 * t * (1 - t)) * P[1] +
		(6 * t * (1 - t) - 3 * t * t) * P[2] +
		3 * t * t * P[3];
}


// [comment]
// Bezier curve control points
// [/comment]
constexpr uint32_t curveNumPts = 30;
Vec3f curveData[curveNumPts] = {
	{-0.001370324, 0.0097554422, 0},
	{-0.0556627219, 0.0293327560, 0},
	{-0.0029370324, 0.0297554422, 0},
	{-0.1556627219, 0.3293327560, 0},
	{-0.2613958914, 0.9578577085, 0},
	{-0.2555218265, 1.3044275420, 0},
	{-0.2496477615, 1.6509973760, 0},
	{-0.1262923970, 2.0445597290, 0},
	{ 0.1791589818, 2.2853963930, 0},
	{ 0.4846103605, 2.5262330570, 0},
	{ 0.9427874287, 2.2560260680, 0},
	{ 1.0132762080, 1.9212043650, 0},
	{ 1.0837649880, 1.5863826610, 0},
	{ 0.9369133637, 1.2750572170, 0},
	{ 0.6667063748, 1.2691831520, 0},
	{ 0.3964993859, 1.2633090870, 0},
	{ 0.2320255666, 1.3514200620, 0},
	{ 0.1850330468, 1.5276420110, 0},
	{ 0.1380405269, 1.7038639600, 0},
	{ 0.2026552417, 1.8918340400, 0},
	{ 0.4082475158, 1.9564487540, 0},
	{ 0.6138397900, 2.0210634690, 0},
	{ 0.7606914144, 1.8800859100, 0},
	{ 0.7606914144, 1.7038639600, 0},
	{ 0.4082475158, 2.9564487540, 0},
	{ 0.6138397900, 3.0210634690, 0},
	{ 0.7606914144, 2.8800859100, 0},
	{ 0.7606914144, 4.7038639600, 0},
	{ 0.7606914144, 4.8800859100, 0},
	{ 0.7606914144, 4.7038639600, 0}
};

// [comment]
// Generate a thin cylinder centred around a Bezier curve
// [/comment]
void createCurveGeometry(std::vector<std::unique_ptr<Object>>& objects)
{
	uint32_t ndivs = 2;
	uint32_t ncurves = 1 + (curveNumPts - 4) / 3;
	Vec3f pts[4];
	std::unique_ptr<Vec3f[]> P(new Vec3f[(ndivs + 1) * ndivs * ncurves + 1]);
	std::unique_ptr<Vec3f[]> N(new Vec3f[(ndivs + 1) * ndivs * ncurves + 1]);
	std::unique_ptr<Vec2f[]> st(new Vec2f[(ndivs + 1) * ndivs * ncurves + 1]);
	for (uint32_t i = 0; i < ncurves; ++i) {
		for (uint32_t j = 0; j < ndivs; ++j) {
			pts[0] = 2 * curveData[i * 3];
			pts[1] = 2 * curveData[i * 3 + 1];
			pts[2] = 2 * curveData[i * 3 + 2];
			pts[3] = 2 * curveData[i * 3 + 3];
			float s = j / (float)ndivs;
			Vec3f pt = evalBezierCurve(pts, s);
			Vec3f tangent = derivBezier(pts, s).normalize();
			bool swap = false;

			uint8_t maxAxis;
			if (std::abs(tangent.x) > std::abs(tangent.y))
				if (std::abs(tangent.x) > std::abs(tangent.z))
					maxAxis = 0;
				else
					maxAxis = 2;
			else if (std::abs(tangent.y) > std::abs(tangent.z))
				maxAxis = 1;
			else
				maxAxis = 2;

			Vec3f up, forward, right;

			switch (maxAxis) {
			case 0:
			case 1:
				up = tangent;
				forward = Vec3f(0, 0, 1);
				right = up.crossProduct(forward);
				forward = right.crossProduct(up);
				break;
			case 2:
				up = tangent;
				right = Vec3f(0, 0, 1);
				forward = right.crossProduct(up);
				right = up.crossProduct(forward);
				break;
			default:
				break;
			};

			float sNormalized = (i * ndivs + j) / float(ndivs * ncurves);
			float rad = 0.1 * (1 - sNormalized);
			for (uint32_t k = 0; k <= ndivs; ++k) {
				float t = k / (float)ndivs;
				float theta = t * 2 * M_PI;
				Vec3f pc(cos(theta) * rad, 0, sin(theta) * rad);
				float x = pc.x * right.x + pc.y * up.x + pc.z * forward.x;
				float y = pc.x * right.y + pc.y * up.y + pc.z * forward.y;
				float z = pc.x * right.z + pc.y * up.z + pc.z * forward.z;
				P[i * (ndivs + 1) * ndivs + j * (ndivs + 1) + k] = Vec3f(pt.x + x, pt.y + y, pt.z + z);
				N[i * (ndivs + 1) * ndivs + j * (ndivs + 1) + k] = Vec3f(x, y, z).normalize();
				st[i * (ndivs + 1) * ndivs + j * (ndivs + 1) + k] = Vec2f(sNormalized, t);
			}
		}
	}
	P[(ndivs + 1) * ndivs * ncurves] = curveData[curveNumPts - 1];
	N[(ndivs + 1) * ndivs * ncurves] = (curveData[curveNumPts - 2] - curveData[curveNumPts - 1]).normalize();
	st[(ndivs + 1) * ndivs * ncurves] = Vec2f(1, 0.5);
	uint32_t numFaces = ndivs * ndivs * ncurves;
	std::unique_ptr<uint32_t[]> verts(new uint32_t[numFaces]);
	for (uint32_t i = 0; i < numFaces; ++i)
		verts[i] = (i < (numFaces - ndivs)) ? 4 : 3;
	std::unique_ptr<uint32_t[]> vertIndices(new uint32_t[ndivs * ndivs * ncurves * 4 + ndivs * 3]);
	uint32_t nf = 0, ix = 0;
	for (uint32_t k = 0; k < ncurves; ++k) {
		for (uint32_t j = 0; j < ndivs; ++j) {
			if (k == (ncurves - 1) && j == (ndivs - 1)) { break; }
			for (uint32_t i = 0; i < ndivs; ++i) {
				vertIndices[ix] = nf;
				vertIndices[ix + 1] = nf + (ndivs + 1);
				vertIndices[ix + 2] = nf + (ndivs + 1) + 1;
				vertIndices[ix + 3] = nf + 1;
				ix += 4;
				++nf;
			}
			nf++;
		}
	}

	for (uint32_t i = 0; i < ndivs; ++i) {
		vertIndices[ix] = nf;
		vertIndices[ix + 1] = (ndivs + 1) * ndivs * ncurves;
		vertIndices[ix + 2] = nf + 1;
		ix += 3;
		nf++;
	}

	objects.push_back(std::unique_ptr<TriangleMesh>(new TriangleMesh(Matrix44f::kIdentity, numFaces, verts, vertIndices, P, N, st)));
}

void fresnel(const Vec3f& I, const Vec3f& N, const float& ior, float& kr)
{
	float cosi = clamp(-1, 1, I.dotProduct(N));
	float etai = 1, etat = ior;
	if (cosi > 0) { std::swap(etai, etat); }
	// Compute sini using Snell's law
	float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
	// Total internal reflection
	if (sint >= 1) {
		kr = 1;
	}
	else {
		float cost = sqrtf(std::max(0.f, 1 - sint * sint));
		cosi = fabsf(cosi);
		float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
		float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
		kr = (Rs * Rs + Rp * Rp) / 2;
	}
	// As a consequence of the conservation of energy, transmittance is given by:
	// kt = 1 - kr;
}

class Triangle
{
public:
	Vec3f v0;                           /// position of the sphere
	Vec3f v1;                           /// position of the sphere
	Vec3f v2;                           /// position of the sphere
	Vec3f surfaceColor, emissionColor;      /// surface color and emission (light)
	float transparency, reflection;         /// surface transparency and reflectivity
	MaterialType materialType;
	Triangle(
		const Vec3f& v_0,
		const Vec3f& v_1,
		const Vec3f& v_2,
		const Vec3f& sc,
		const float& refl = 0,
		const float& transp = 0,
		const Vec3f& ec = 0) :
		v0(v_0), v1(v_1), v2(v_2), surfaceColor(sc), emissionColor(ec),
		transparency(transp), reflection(refl), materialType(DIFFUSE_AND_GLOSSY)
	{ /* empty */
	}
	//[comment]
	// Compute a ray-sphere intersection using the geometric solution
	//[/comment]
	bool rayTriangleIntersect(
		const Vec3f& orig, const Vec3f& dir,
		float& t) const
	{
		float u;
		float v;
#ifdef MOLLER_TRUMBORE
		Vec3f v0v1 = v1 - v0;
		Vec3f v0v2 = v2 - v0;
		Vec3f pvec = dir.crossProduct(v0v2);
		float det = v0v1.dotProduct(pvec);
#ifdef CULLING
		// if the determinant is negative the triangle is backfacing
		// if the determinant is close to 0, the ray misses the triangle
		if (det < kEpsilon) return false;
#else
		// ray and triangle are parallel if det is close to 0
		if (fabs(det) < kEpsilon) return false;
#endif
		float invDet = 1 / det;

		Vec3f tvec = orig - v_0;
		u = tvec.dotProduct(pvec) * invDet;
		if (u < 0 || u > 1) return false;

		Vec3f qvec = tvec.crossProduct(v0v1);
		v = dir.dotProduct(qvec) * invDet;
		if (v < 0 || u + v > 1) return false;

		t = v0v2.dotProduct(qvec) * invDet;

		return true;
#else
		// compute plane's normal
		Vec3f v0v1 = v1 - v0;
		Vec3f v0v2 = v2 - v0;
		// no need to normalize
		Vec3f N = v0v1.crossProduct(v0v2); // N
		float denom = N.dotProduct(N);

		// Step 1: finding P

		// check if ray and plane are parallel ?
		float NdotRayDirection = N.dotProduct(dir);
		if (fabs(NdotRayDirection) < kEpsilon) // almost 0
			return false; // they are parallel so they don't intersect ! 

		// compute d parameter using equation 2
		float d = N.dotProduct(v0);

		// compute t (equation 3)
		t = (N.dotProduct(orig) + d) / NdotRayDirection;
		// check if the triangle is in behind the ray
		if (t < 0) return false; // the triangle is behind

		// compute the intersection point using equation 1
		Vec3f P = orig + t * dir;

		// Step 2: inside-outside test
		Vec3f C; // vector perpendicular to triangle's plane

		// edge 0
		Vec3f edge0 = v1 - v0;
		Vec3f vp0 = P - v0;
		C = edge0.crossProduct(vp0);
		if (N.dotProduct(C) < 0) return false; // P is on the right side

		// edge 1
		Vec3f edge1 = v2 - v1;
		Vec3f vp1 = P - v1;
		C = edge1.crossProduct(vp1);
		if ((u = N.dotProduct(C)) < 0)  return false; // P is on the right side

		// edge 2
		Vec3f edge2 = v0 - v2;
		Vec3f vp2 = P - v2;
		C = edge2.crossProduct(vp2);
		if ((v = N.dotProduct(C)) < 0) return false; // P is on the right side;

		u /= denom;
		v /= denom;

		return true; // this ray hits the triangle
#endif
	}
};

inline
Vec3f mix(const Vec3f& a, const Vec3f& b, const float& mixValue)
{
	return a * (1 - mixValue) + b * mixValue;
}
class Sphere
{
public:
	Vec3f center;                           /// position of the sphere
	float radius, radius2;                  /// sphere radius and radius^2
	Vec3f surfaceColor, emissionColor;      /// surface color and emission (light)
	float transparency, reflection;         /// surface transparency and reflectivity
	MaterialType materialType;

	Sphere(
		const Vec3f& c,
		const float& r,
		const Vec3f& sc,
		const float& refl = 0,
		const float& transp = 0,
		const Vec3f& ec = 0) :
		center(c), radius(r), radius2(r* r), surfaceColor(sc), emissionColor(ec),
		transparency(transp), reflection(refl), materialType(DIFFUSE_AND_GLOSSY)
	{ /* empty */
	}
	//[comment]
	// Compute a ray-sphere intersection using the geometric solution
	//[/comment]
	bool raySphereIntersect(const Vec3f& rayorig, const Vec3f& raydir, float& t0, float& t1) const
	{
		Vec3f l = center - rayorig;
		float tca = l.dotProduct(raydir);
		if (tca < 0) return false;
		float d2 = l.dotProduct(l) - tca * tca;
		if (d2 > radius2) return false;
		float thc = sqrt(radius2 - d2);
		t0 = tca - thc;
		t1 = tca + thc;

		return true;
	}

	Vec3f evalDiffuseColor(const Vec2f& st) const
	{
		float scale = 5;
		float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
		return mix(Vec3f(0.815, 0.235, 0.031), Vec3f(0.937, 0.937, 0.231), pattern);
	}
};





Vec3f reflect(const Vec3f& I, const Vec3f& N)
{
	return I - 2 * I.dotProduct(N) * N;
}

Vec3f refract(const Vec3f& I, const Vec3f& N, const float& ior)
{
	float cosi = clamp(-1, 1, I.dotProduct(N));
	float etai = 1, etat = ior;
	Vec3f n = N;
	if (cosi < 0) { cosi = -cosi; }
	else { std::swap(etai, etat); n = -N; }
	float eta = etai / etat;
	float k = 1 - eta * eta * (1 - cosi * cosi);
	return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}


bool trace_more(
	const Vec3f& orig, const Vec3f& dir,
	std::vector<Sphere>& spheres,
	float& tNear, uint32_t& index, Vec2f& uv, Sphere* hitObject)
{

	bool hit = false;
	for (uint32_t k = 0; k < spheres.size(); ++k) {
		float tNearK = kInfinity;
		uint32_t indexK;
		Vec2f uvK;
		float t1;
		if (spheres[k].raySphereIntersect(orig, dir, tNearK, t1) && tNearK < tNear) {
			hitObject = &spheres[k];
			hit = true;
			tNear = tNearK;
			index = 0;
			uv = uvK;
		}
	}

	for (uint32_t k = 0; k < spheres.size(); ++k) {
		float tNearK = kInfinity;
		uint32_t indexK;
		Vec2f uvK;
		float t1;
		if (spheres[k].raySphereIntersect(orig, dir, tNearK, t1) && tNearK < tNear) {
			hitObject = &spheres[k];
			hit = true;
			tNear = tNearK;
			index = 0;
			uv = uvK;
		}
	}

	return hit;
	//return (*hitObject != nullptr);
}


Vec3f castRay(
	const Vec3f& rayorig,
	const Vec3f& raydir,
	std::vector<Sphere>& spheres,
	std::vector<Sphere>& lights,
	const int& depth)
{

	if (depth > 5) {
		return Vec3f(0.6, 0.8, 1);
	}

	Vec2f uv;
	uint32_t index = 0;

	Vec3f  hitColor = Vec3f(0.6, 0.8, 1);

	Vec3f color(0, 0, 0);
	//if (raydir.length() != 1) std::cerr << "Error " << raydir << std::endl;
	float tnear = INFINITY;
	Sphere* sphere = NULL;
	float t0, t1;
	int type = 0;
	t0 = INFINITY;


	//find intersection of this ray with the sphere in the scene
	for (unsigned i = 0; i < spheres.size(); ++i) {
		t0 = INFINITY, t1 = INFINITY;
		if (spheres[i].raySphereIntersect(rayorig, raydir, t0, t1)) {
			if (t0 < 0) t0 = t1;
			if (t0 < tnear) {
				tnear = t0;
				sphere = &spheres[i];
				hitColor = sphere->surfaceColor;
			}
		}
	}

	t0 = 500000;

	// if there's no intersection return black or background color
	if (!type && !sphere) return hitColor;
	Vec3f surfaceColor = 0; // color of the ray/surfaceof the object intersected by the ray
	Vec3f hitPoint = rayorig + raydir * tnear; // point of intersection
	Vec3f nhit = NULL;
	nhit = hitPoint - sphere->center;
	nhit.normalize(); // normalize normal direction
	// If the normal and the view direction are not opposite to each other
	// reverse the normal direction. That also means we are inside the sphere so set
	// the inside bool to true. Finally reverse the sign of IdotN which we want
	// positive.
	float bias = 1e-4; // add some bias to the point from which we will be tracing
	bool inside = false;
	if (raydir.dotProduct(nhit) > 0) nhit = -nhit, inside = true;


	Vec3f N = nhit; // normal
	Vec2f st; // st coordinates
	//hitObject->getSurfaceProperties(hitPoint, dir, index, uv, N, st);
	Vec3f tmp = hitPoint;

	// it's a diffuse object, no need to raytrace any further

	if (!type) {
		switch (sphere->materialType) {
		case REFLECTION_AND_REFRACTION:
		{
			Vec3f reflectionDirection = reflect(raydir, N).normalize();
			Vec3f refractionDirection = refract(raydir, N, 3).normalize();
			Vec3f reflectionRayOrig = (reflectionDirection.dotProduct(N) < 0) ?
				hitPoint - N * bias :
				hitPoint + N * bias;
			Vec3f refractionRayOrig = (refractionDirection.dotProduct(N) < 0) ?
				hitPoint - N * bias :
				hitPoint + N * bias;
			Vec3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, spheres, lights, depth + 1);
			Vec3f refractionColor = castRay(refractionRayOrig, refractionDirection, spheres, lights, depth + 1);
			float kr;
			fresnel(raydir, N, 2, kr);
			hitColor = refractionColor * (1 - kr);
			break;
		}
		case REFLECTION:
		{

			Vec3f reflectionDirection = reflect(raydir, N).normalize();
			Vec3f reflectionRayOrig = (reflectionDirection.dotProduct(N) < 0) ?
				hitPoint - N * bias :
				hitPoint + N * bias;
			Vec3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, spheres, lights, depth + 1);
			float kr;
			fresnel(raydir, N, 2, kr);
			hitColor = (1 - kr);
			break;
		}            default:
		{
			for (unsigned i = 0; i < lights.size(); ++i) {


				// this is a light
				Vec3f lightAmt = 0, specularColor = 0;
				Vec3f shadowPointOrig = (raydir.dotProduct(N) < 0) ?
					hitPoint + N * bias :
					hitPoint - N * bias;
				// [comment]
				// Loop over all lights in the scene and sum their contribution up
				// We also apply the lambert cosine law here though we haven't explained yet what this means.
				// [/comment]
				//for (uint32_t i = 0; i < lights.size(); ++i) {
				Vec3f lightDir = lights[i].center - hitPoint;
				// square of the distance between hitPoint and the light
				float lightDistance2 = lightDir.dotProduct(lightDir);
				lightDir = lightDir.normalize();
				float LdotN = std::max(0.f, lightDir.dotProduct(N));
				Sphere* shadowHitObject = nullptr;
				float tNearShadow = kInfinity;
				// is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
				bool inShadow = trace_more(shadowPointOrig, lightDir, spheres, tNearShadow, index, uv, shadowHitObject) &&
					tNearShadow * tNearShadow < lightDistance2;
				lightAmt += (1 - inShadow) * lights[i].emissionColor * LdotN;
				Vec3f reflectionDirection = reflect(-lightDir, N);
				specularColor += powf(std::max(0.f, -reflectionDirection.dotProduct(raydir)), 25) * lights[i].emissionColor;
				//}
				Vec2f st(0.2);
				hitColor = lightAmt * lights[i].evalDiffuseColor(st) * 0.8  +specularColor * 0.2;
				hitColor += sphere->surfaceColor;

			}

		}
		}
	}
	return (hitColor);

	//return Vec3f(1, 1, 1);
}

// This will generate a random number to be used for the anti alisaing to generate radndom scatterd rays.
inline double random_double()
{
	static std::uniform_real_distribution<double > distribution(0.0, 1.0);
	static std::mt19937 generator;
	return distribution(generator);
}

//  This will write pixels into a file
void write_into_file(const Settings& settings, int frame, Vec3f* image) {
	// Save result to a PPM image (keep these flags if you compile under Windows)
	std::ofstream ofs("./" + std::to_string(frame) + "out.ppm", std::ios::out | std::ios::binary);
	ofs << "P6\n" << settings.width << " " << settings.height << "\n255\n";
	for (unsigned i = 0; i < settings.width * settings.height; ++i) {
		ofs << (unsigned char)(std::min(float(1), image[i].x / settings.aa_samples) * 255) <<
			(unsigned char)(std::min(float(1), image[i].y / settings.aa_samples) * 255) <<
			(unsigned char)(std::min(float(1), image[i].z / settings.aa_samples) * 255);
	}
	ofs.close();

	delete[] image;
}
void render(const Settings& settings, std::vector<Sphere>& spheres, std::vector<Sphere>& lights, std::vector<Triangle> triangles, int frame)
{
	Vec3f* image = new Vec3f[settings.width * settings.height], * pixel = image;
	float invWidth = 1 / float(settings.width), invHeight = 1 / float(settings.height);
	float fov = 30, aspectratio = settings.width / float(settings.height);
	float angle = tan(M_PI * 0.5 * fov / 180.);
	// Trace rays
	for (unsigned y = 0; y < settings.height; ++y) {
		for (unsigned x = 0; x < settings.width; ++x, ++pixel) {
			Vec3f sampled_pixel(0, 0, 0);

			for (unsigned sample = 0; sample < settings.aa_samples; ++sample) {
				float xx = (2 * ((x + random_double()) * invWidth) - 1) * angle * aspectratio;
				float yy = (1 - 2 * ((y + random_double()) * invHeight)) * angle;
				Vec3f raydir(xx, yy, -1);
				raydir.normalize();
				sampled_pixel += castRay(Vec3f(0), raydir, spheres, lights, 0);
			}
			*pixel = sampled_pixel;
		}
	}
	write_into_file(settings, frame, image);
}




int main(int argc, char** argv)
{
	Settings settings;

	for (int frame = 15; frame > 14; frame--) {
		int shift = 20;
		int sh_y = 4;
		std::vector<Sphere> spheres;
		std::vector<Sphere> lights;

		std::vector<Triangle> triangles;


		Sphere light = Sphere(Vec3f(0, 30, -10), 1, Vec3f(1, 1, 1), 0, 0.0, Vec3f(1));
		Sphere gray = Sphere(Vec3f(0, 0, -20), 2, Vec3f(0.1, 0.4, 0.6), 1, 0.0);
		Sphere gray_1 = Sphere(Vec3f(-0.5, 0, -23), 0.5, Vec3f(0, 0, 0), 1, 0.0);

		gray.materialType = REFLECTION_AND_REFRACTION;

		spheres.push_back(gray); //gray left
		spheres.push_back(gray_1); //gray left
		spheres.push_back(Sphere(Vec3f(0.0, -100, -20), 98, Vec3f(0.20, 0.20, 0.20), 0, 0.0)); // ground
		//spheres.push_back(Sphere(Vec3f(0, 0, -20), 2, Vec3f(0.1, 0.77, 0.97), 1, 0.0)); //yellow right
		lights.push_back(light);


		render(settings, spheres, lights, triangles, frame);
	}
	return 0;
}