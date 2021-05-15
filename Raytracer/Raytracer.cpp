// Raytracer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include "ray.h"
#include "Triangle.h"
using namespace std;
constexpr float kEpsilon = 1e-8;

bool hit_sphere(const vec3& center, float radius, const ray& r) {
	vec3 oc = r.origin() - center;
	float a = dot(r.direction(), r.direction());
	float b = 2.0 * dot(oc, r.direction());
	float c = dot(oc, oc) - radius * radius;
	float discrimnant = b * b - 4 * a * c;
	return (discrimnant > 0);
}
bool hit_triangle(Triangle& tri, const ray& r) {
	 double inter = tri.findIntersection(r.direction(), r.origin());
	 return (inter != -1);
}

bool rayTriangleIntersect(
	const vec3& orig, const vec3& dir,
	const vec3& v0, const vec3& v1, const vec3& v2,
	float& t, float& u, float& v)
{
#ifdef MOLLER_TRUMBORE 
	vec3 v0v1 = v1 - v0;
	vec3 v0v2 = v2 - v0;
	vec3 pvec = dir.crossProduct(v0v2);
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

	vec3 tvec = orig - v0;
	u = tvec.dotProduct(pvec) * invDet;
	if (u < 0 || u > 1) return false;

	vec3 qvec = tvec.crossProduct(v0v1);
	v = dir.dotProduct(qvec) * invDet;
	if (v < 0 || u + v > 1) return false;

	t = v0v2.dotProduct(qvec) * invDet;

	return true;
#else 
	// compute plane's normal
	vec3 v0v1 = v1 - v0;
	vec3 v0v2 = v2 - v0;
	// no need to normalize
	vec3 N = cross(v0v1, v0v2); // N 
	float denom = dot(N, N);

	// Step 1: finding P

	// check if ray and plane are parallel ?
	float NdotRayDirection = dot(N, dir);
	if (fabs(NdotRayDirection) < kEpsilon) // almost 0 
		return false; // they are parallel so they don't intersect ! 

	// compute d parameter using equation 2
	float d = dot(N, v0);

	// compute t (equation 3)
	t = (dot(N, orig) + d) / NdotRayDirection;
	// check if the triangle is in behind the ray
	if (t < 0) return false; // the triangle is behind 

	// compute the intersection point using equation 1
	vec3 P = orig + t * dir;

	// Step 2: inside-outside test
	vec3 C; // vector perpendicular to triangle's plane 

	// edge 0
	vec3 edge0 = v1 - v0;
	vec3 vp0 = P - v0;
	C = cross(edge0, vp0);
	if (dot(N, C) < 0) return false; // P is on the right side 

	// edge 1
	vec3 edge1 = v2 - v1;
	vec3 vp1 = P - v1;
	C = cross(edge1, vp1);
	if ((u = dot(N, C)) < 0)  return false; // P is on the right side 

	// edge 2
	vec3 edge2 = v0 - v2;
	vec3 vp2 = P - v2;
	C = cross(edge2, vp2);
	if ((v = dot(N, C)) < 0) return false; // P is on the right side; 

	u /= denom;
	v /= denom;

	return true; // this ray hits the triangle 
#endif 
}
vec3 color(Triangle& tri, const ray& r) {
	//if (hit_triangle(tri, r))
	//	return vec3(1, 0, 0);
	float t, u, v;
	vec3 v0(-1, -1, -5);
	vec3 v1(1, -1, -5);
	vec3 v2(0, 1, -5);
	if (rayTriangleIntersect(r.origin(), r.direction(), v0, v1, v2, t, u, v)) {
		return vec3(1, 0, 0);
	}
	vec3 unit_direction = unit_vector(r.direction());
	t = 0.5 * (unit_direction.y() + 1.0);
	return (1.0 - t) * vec3(1.0, 1.0, 1.0) + t * vec3(0.5, 0.7, 1.0);
}

int main()
{
	ofstream myfile;
	myfile.open("image.ppm");

	int nx = 200; // number of columns width
	int ny = 100; // number of rows hight
	myfile << "P3\n" << nx << " " << ny << "\n255\n";
	vec3 lower_left_corner(-1.0, -0.5, -0.5);
	vec3 horizontal(2.0, 0.0, 0.0);
	vec3 vertical(0.0, 1.0, 0.0);
	vec3 origin(0.0, 0.0, 0.0);
	Triangle tri(vec3(-0.001, 0, 0), vec3(0, 0.1, 0), vec3(0, 0, -0.001));


	for (int j = ny - 1; j >= 0; j--) {
		for (int i = 0; i < nx; i++) {
			float u = float(i) / float(nx);
			float v = float(j) / float(ny);

			ray r(origin, lower_left_corner + u * horizontal + v * vertical);
			vec3 col = color(tri, r);

			int ir = int(255.99 * col[0]);
			int ig = int(255.99 * col[1]);
			int ib = int(255.99 * col[2]);

			myfile << ir << " " << ig << " " << ib << "\n";
		}

	}
	myfile.close();

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
