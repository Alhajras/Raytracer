// Raytracer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <string>

#include <iostream>
#include <fstream>
#include "ray.h"
#include "Triangle.h"
#include "camera.h"

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
	const vec3& rayOrigin, const vec3& rayDirection,
	const vec3& a, const vec3& b, const vec3& c,
	float& t, float& u, float& v)
{
	vec3 e1 = b - a;
	vec3 e2 = c - a;
	u = dot((rayOrigin - a), cross(rayDirection, e2)) / dot(e1, (cross(rayDirection, e2)));
	v = dot(rayDirection, (cross((rayOrigin - a), e1)) / dot(e1, cross(rayDirection, e2)));
	float w = 1 - u - v;

	float t0 = dot(e2, cross((rayOrigin - a), e1)) / dot(e1, cross(rayDirection, e2));

	if ((u < 0) || (u > 1)) {
		return false;
	}
	else if ((v < 0) || (u + v > 1)) {
		return false;
	}
	else if (t0 <= 0) {
		return false;
	}
	else {
		t = t0;
		return true;
	}
}
vec3 color(Triangle& tri, const ray& r) {
	//if (hit_triangle(tri, r))
	//	return vec3(1, 0, 0);
	float t, u, v;
	vec3 v0(-1, -1, 1);
	vec3 v1(1, -1, 1);
	vec3 v2(1, 1, 1);
	if (rayTriangleIntersect(r.origin(), r.direction(), v0, v1, v2, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 0, 0) + t * vec3(0.5, 0.7, 1.0);

	}

	vec3 v3(-1, -1, 1);
	vec3 v4(1, 1, 1);
	vec3 v5(-1, 1, 1);
	if (rayTriangleIntersect(r.origin(), r.direction(), v3, v4, v5, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 0, 1) + t * vec3(0.5, 0.7, 1.0);

	}

	vec3 v6(1, -1, 1);
	vec3 v7(1, -1, -1);
	vec3 v8(1, 1, -1);
	if (rayTriangleIntersect(r.origin(), r.direction(), v6, v7, v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0.5, 0.1, 0) + t * vec3(0.5, 0.7, 1.0);
	}


	if (rayTriangleIntersect(r.origin(), r.direction(), vec3(1, -1, 1), vec3(1, 1, -1), vec3(1, 1, 1), t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0,0, 0) + t * vec3(0.5, 0.7, 1.0);
	}


	if (rayTriangleIntersect(r.origin(), r.direction(), vec3(1, -1, -1), vec3(-1, -1, -1), vec3(-1, 1, -1), t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 0.51, 1) + t * vec3(0.5, 0.7, 1.0);

	}


	if (rayTriangleIntersect(r.origin(), r.direction(), vec3(1, -1, -1), vec3(-1, 1, -1), vec3(1, 1, -1), t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0, 1, 0) + t * vec3(0.5, 0.7, 1.0);

	}

	
	if (rayTriangleIntersect(r.origin(), r.direction(), vec3(-1, -1, -1), vec3(-1, -1, 1), vec3(-1, 1, 1), t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 1, 0) + t * vec3(0.5, 0.7, 1.0);

	}

	if (rayTriangleIntersect(r.origin(), r.direction(), vec3(-1, -1, -1), vec3(-1, 1, 1), vec3(-1, 1, -1), t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 0, 0.22) + t * vec3(0.5, 0.7, 1.0);

	}


	if (rayTriangleIntersect(r.origin(), r.direction(), vec3(-1, 1, 1), vec3(1, 1, 1), vec3(1, 1, -1), t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 1, 0) + t * vec3(0.5, 0.7, 1.0);

	}


	if (rayTriangleIntersect(r.origin(), r.direction(), vec3(-1, 1, 1),vec3(1, 1, -1),vec3(-1, 1, -1), t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0, 1, 0.60) + t * vec3(0.5, 0.7, 1.0);

	}

	if (rayTriangleIntersect(r.origin(), r.direction(), vec3(1, -1, 1), vec3(-1, -1, -1), vec3(1, -1, -1), t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0.91, 0.41, 0) + t * vec3(0.5, 0.7, 1.0);

	}

	if (rayTriangleIntersect(r.origin(), r.direction(), vec3(1, -1, 1), vec3(-1, -1, 1), vec3(-1, -1, -1), t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0, 0, 0) + t * vec3(0.5, 0.7, 1.0);

	}

	//if (rayTriangleIntersect(r.origin(), r.direction(), vec3(1, -1, 1), vec3(-1, -1, 1), vec3(-1, -1, -1), t, u, v)) {
	//	vec3 unit_direction = unit_vector(r.direction());
	//	t = 0.5 * (unit_direction.y() + 1.0);
	//	return (1.0 - t) * vec3(0, 0, 0) + t * vec3(0.5, 0.7, 1.0);

	//}
	  
	vec3 unit_direction = unit_vector(r.direction());
	t = 0.5 * (unit_direction.y() + 1.0);
	return vec3(1, 1, 1);
	//return (1.0 - t) * vec3(1.0, 1.0, 1.0) + t * vec3(0.5, 0.7, 1.0);
}

int main()
{
	camera cam;
	ofstream myfile;
	for (int image = 0; image <= 1; image++){
	myfile.open("scene" + std::to_string(image) + ".ppm" );

	int nx = 800; // number of columns width
	int ny = 400; // number of rows hight
	myfile << "P3\n" << nx << " " << ny << "\n255\n";

	Triangle tri(vec3(-0.001, 0, 0), vec3(0, 0.1, 0), vec3(0, 0, -0.001));


	for (int j = ny - 1; j >= 0; j--) {
		for (int i = 0; i < nx; i++) {
			float u = float(i) / float(nx);
			float v = float(j) / float(ny);

			ray r = cam.get_ray(u,v);
			//vec3 p = r.point_at_parameter(2.0);
			vec3 col = color(tri, r);

			int ir = int(255.99 * col[0]);
			int ig = int(255.99 * col[1]);
			int ib = int(255.99 * col[2]);

			myfile << ir << " " << ig << " " << ib << "\n";
		}

	}
	myfile.close();
	}
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
