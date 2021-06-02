//[header]
// A simple program that uses ray-tracing to render a single triangle
//[/header]
//[compile]
// Download the raytri.cpp and geometry.h files to a folder.
// Open a shell/terminal, and run the following command where the files is saved:
//
// c++ -o raytri raytri.cpp -O3 -std=c++11 -DMOLLER_TRUMBORE
//
// Run with: ./raytri. Open the file ./out.png in Photoshop or any program
// reading PPM files.
//[/compile]
//[ignore]
// Copyright (C) 2012  www.scratchapixel.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//[/ignore]

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <random>
#include "geometry.h"
#include <string>

const float kInfinity = std::numeric_limits<float>::max();

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

// [comment]
// The main ray-triangle intersection routine. You can test both methoods: the
// geometric method and the Moller-Trumbore algorithm (use the flag -DMOLLER_TRUMBORE
// when you compile.
// [/comment]

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
			//Vec3f reflectionDirection = reflect(raydir, N).normalize();
			Vec3f refractionDirection = refract(raydir, N, 1.5).normalize();
			//Vec3f reflectionRayOrig = (reflectionDirection.dotProduct(N) < 0) ?
			//	hitPoint - N * bias :
			//	hitPoint + N * bias;
			Vec3f refractionRayOrig = (refractionDirection.dotProduct(N) < 0) ?
				hitPoint - N * bias :
				hitPoint + N * bias;
			//Vec3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, spheres, depth + 1);
			Vec3f refractionColor = castRay(refractionRayOrig, refractionDirection, spheres, lights, depth + 1);
			float kr;
			fresnel(raydir, N, 1.5, kr);
			 hitColor = refractionColor * (1 - kr);
			break;
		}
		case REFLECTION:
		{
			
			Vec3f R = reflect(raydir, N);
			R.normalize();
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
					hitColor = lightAmt * lights[i].evalDiffuseColor(st) * 0.8 + specularColor * 0.1;
					hitColor += sphere->surfaceColor;
				
			}

		}
		}
	}
	return (hitColor);

	//return Vec3f(1, 1, 1);
}

void render(std::vector<Sphere>& spheres, std::vector<Sphere>& lights, std::vector<Triangle> triangles, int frame)
{
	unsigned width = 640, height = 480;
	Vec3f* image = new Vec3f[width * height], * pixel = image;
	float invWidth = 1 / float(width), invHeight = 1 / float(height);
	float fov = 30, aspectratio = width / float(height);
	float angle = tan(M_PI * 0.5 * fov / 180.);
	// Trace rays
	for (unsigned y = 0; y < height; ++y) {
		for (unsigned x = 0; x < width; ++x, ++pixel) {
			float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectratio;
			float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;
			Vec3f raydir(xx, yy, -1);
			raydir.normalize();
			*pixel = castRay(Vec3f(0), raydir, spheres, lights, 0);
		}
	}

	// Save result to a PPM image (keep these flags if you compile under Windows)
	std::ofstream ofs("./" + std::to_string(frame) + "out.ppm", std::ios::out | std::ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";
	for (unsigned i = 0; i < width * height; ++i) {
		ofs << (unsigned char)(std::min(float(1), image[i].x) * 255) <<
			(unsigned char)(std::min(float(1), image[i].y) * 255) <<
			(unsigned char)(std::min(float(1), image[i].z) * 255);
	}
	ofs.close();

	delete[] image;
}

int main(int argc, char** argv)
{

	for (int frame = 15; frame > 14; frame--) {
		int shift = 20;
		int sh_y = 4;
		std::vector<Sphere> spheres;
		std::vector<Sphere> lights;

		std::vector<Triangle> triangles;


		Sphere light = Sphere(Vec3f(0, frame, -20), 4, Vec3f(1, 1, 1), 0, 0.0, Vec3f(2));
		Sphere gray = Sphere(Vec3f(-5.5, 0, -20), 3, Vec3f(0.1, 0.4, 0.6), 1, 0.0);
		gray.materialType = REFLECTION;

		spheres.push_back(gray); //gray left
		spheres.push_back(Sphere(Vec3f(0.0, -100, -20), 98, Vec3f(0.20, 0.20, 0.20), 0, 0.0)); // ground
		spheres.push_back(Sphere(Vec3f(5, 0, -20), 2, Vec3f(0.1, 0.77, 0.97), 1, 0.0)); //yellow right
		lights.push_back(light);


		render(spheres, lights, triangles, frame);
	}
	return 0;
}