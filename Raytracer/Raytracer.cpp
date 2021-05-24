// Raytracer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <string>

#include <iostream>
#include <fstream>
#include "ray.h"
#include "Triangle.h"
#include "camera.h"
#include "rtweekend.h"
#include "color.h"
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>


using namespace std;
constexpr float kEpsilon = 1e-8;

vec3 rotate(const vec3& v, const vec3& k, double theta)
{

	double cos_theta = cos(theta);
	double sin_theta = sin(theta);

	vec3 rotated = (v * cos_theta) + (cross(k, v) * sin_theta) + (k * dot(k, v)) * (1 - cos_theta);

	return rotated;
}

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


bool rayTriangleIntersect(const vec3& rayOrigin, const vec3& rayDirection,
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
vec3 color(Triangle& tri, const ray& r, double theta) {
	//if (hit_triangle(tri, r))
	//	return vec3(1, 0, 0);
	float t, u, v;
	vec3 v0(-1, -1, 1);
	vec3 v1(1, -1, 1);
	vec3 v2(1, 1, 1);

	vec3 o = r.origin();
	vec3 dir = r.direction();

		if (hit_sphere(vec3(-6, 0, 0), 2, r)) {
			vec3 unit_direction = unit_vector(r.direction());
			t = 0.5 * (unit_direction.y() + 1.0);
			return (1.0 - t) * vec3(1,0.8 , 0.3) + t * vec3(0.5, 0.7, 1.0);

		}
	// Rotate 'v', a unit vector on the x-axis 180 degrees
	// around 'k', a unit vector pointing up on the z-axis.
	vec3 center = vec3(0, 0, 0);
	vec3 rotated_v0 = rotate(v0, center, theta);
	vec3 rotated_v1 = rotate(v1, center, theta);
	vec3 rotated_v2 = rotate(v2, center, theta);

	if (rayTriangleIntersect(o,dir , rotated_v0, rotated_v1, rotated_v2, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 0, 0) + t * vec3(0.5, 0.7, 1.0);

	}

	vec3 v3(-1, -1, 1);
	vec3 v4(1, 1, 1);
	vec3 v5(-1, 1, 1);
	vec3 rotated_v3 = rotate(v3, center, theta);
	vec3 rotated_v4 = rotate(v4, center, theta);
	vec3 rotated_v5 = rotate(v5, center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v3, rotated_v4, rotated_v5, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 0, 1) + t * vec3(0.5, 0.7, 1.0);

	}

	vec3 v6(1, -1, 1);
	vec3 v7(1, -1, -1);
	vec3 v8(1, 1, -1);
	vec3 rotated_v6 = rotate(v6, center, theta);
	vec3 rotated_v7 = rotate(v7, center, theta);
	vec3 rotated_v8 = rotate(v8, center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0.5, 0.1, 0) + t * vec3(0.5, 0.7, 1.0);
	}

	 rotated_v6 = rotate(vec3(1, -1, 1), center, theta);
	 rotated_v7 = rotate(vec3(1, 1, -1), center, theta);
	 rotated_v8 = rotate(vec3(1, 1, 1), center, theta);
	
	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0.5, 0.8, 0.2) + t * vec3(0.5, 0.7, 1.0);
	}

	 rotated_v6 = rotate(vec3(1, -1, -1), center, theta);
	 rotated_v7 = rotate(vec3(-1, -1, -1), center, theta);
	 rotated_v8 = rotate(vec3(-1, 1, -1), center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0, 0.51, 1) + t * vec3(0.5, 0.7, 1.0);

	}


	rotated_v6 = rotate(vec3(1, -1, -1), center, theta);
	rotated_v7 = rotate(vec3(-1, 1, -1), center, theta);
	rotated_v8 = rotate(vec3(1, 1, -1), center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0, 1, 0) + t * vec3(0.5, 0.7, 1.0);

	}

	rotated_v6 = rotate(vec3(-1, -1, -1), center, theta);
	rotated_v7 = rotate(vec3(-1, -1, 1), center, theta);
	rotated_v8 = rotate(vec3(-1, 1, 1), center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 1, 0) + t * vec3(0.5, 0.7, 1.0);

	}


	rotated_v6 = rotate(vec3(-1, -1, -1), center, theta);
	rotated_v7 = rotate(vec3(-1, 1, 1), center, theta);
	rotated_v8 = rotate(vec3(-1, 1, -1), center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 0, 0.22) + t * vec3(0.5, 0.7, 1.0);

	}


	rotated_v6 = rotate(vec3(-1, 1, 1), center, theta);
	rotated_v7 = rotate(vec3(1, 1, 1), center, theta);
	rotated_v8 = rotate(vec3(1, 1, -1), center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(1, 0.6, 0.1) + t * vec3(0.5, 0.7, 1.0);

	}


	rotated_v6 = rotate(vec3(-1, 1, 1), center, theta);
	rotated_v7 = rotate(vec3(1, 1, -1), center, theta);
	rotated_v8 = rotate(vec3(-1, 1, -1), center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0, 1, 0.60) + t * vec3(0.5, 0.7, 1.0);

	}


	rotated_v6 = rotate(vec3(1, -1, 1), center, theta);
	rotated_v7 = rotate(vec3(-1, -1, -1), center, theta);
	rotated_v8 = rotate(vec3(1, -1, -1), center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0.91, 0.41, 0) + t * vec3(0.5, 0.7, 1.0);

	}


	rotated_v6 = rotate(vec3(1, -1, 1), center, theta);
	rotated_v7 = rotate(vec3(-1, -1, 1), center, theta);
	rotated_v8 = rotate(vec3(-1, -1, -1), center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0.2, 0, 1) + t * vec3(0.5, 0.7, 1.0);

	}

	// tetrahidron
	//1.0, 1.0, 1.0, // V0
   //-1.0, 1.0, -1.0, // V1
   // 1.0, -1.0, -1.0, // V2
	//-1.0, -1.0, 1.0  // v3

	//{1, 2, 3}, // F0
	
		
		
	rotated_v6 = rotate(vec3(-1.0+2, 1.0 + 2, -1.0 +2), center, theta);
	rotated_v7 = rotate(vec3(1.0 +2, -1.0 +2, -1.0 + 2), center, theta);
	rotated_v8 = rotate(vec3(-1.0 +2, -1.0 + 2, 1.0 + 2), center, theta);

	if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
		vec3 unit_direction = unit_vector(r.direction());
		t = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - t) * vec3(0.3, 0, 0) + t * vec3(0.5, 0.7, 1.0);

	}
	//{ 0, 3, 2 }, // F1
		
		
		
			rotated_v6 = rotate(vec3(1.0 + 2, 1.0 + 2, 1.0 + 2), center, theta);
		rotated_v7 = rotate(vec3(-1.0 + 2, -1.0 + 2, 1.0 + 2), center, theta);
		rotated_v8 = rotate(vec3(1.0 + 2, -1.0 + 2, -1.0 + 2), center, theta);

		if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
			vec3 unit_direction = unit_vector(r.direction());
			t = 0.5 * (unit_direction.y() + 1.0);
			return (1.0 - t) * vec3(0, 0.3, 0) + t * vec3(0.5, 0.7, 1.0);

		}
	//{ 0, 1, 3 }, // F2
		
		
		
			rotated_v6 = rotate(vec3(1.0 + 2, 1.0 + 2, 1.0 + 2), center, theta);
		rotated_v7 = rotate(vec3(-1.0 + 2, 1.0 + 2, -1.0 + 2), center, theta);
		rotated_v8 = rotate(vec3(-1.0 + 2, -1.0 + 2, 1.0 + 2), center, theta);

		if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
			vec3 unit_direction = unit_vector(r.direction());
			t = 0.5 * (unit_direction.y() + 1.0);
			return (1.0 - t) * vec3(0, 0, 0.3) + t * vec3(0.5, 0.7, 1.0);

		}
	//{ 0, 2, 1 }  // F3
		
		
		
			rotated_v6 = rotate(vec3(1.0 + 2, 1.0 + 2, 1.0 + 2), center, theta);
		rotated_v7 = rotate(vec3(1.0 + 2, -1.0 + 2, -1.0 + 2), center, theta);
		rotated_v8 = rotate(vec3(-1.0 + 2, 1.0 + 2, -1.0 + 2), center, theta);

		if (rayTriangleIntersect(o, dir, rotated_v6, rotated_v7, rotated_v8, t, u, v)) {
			vec3 unit_direction = unit_vector(r.direction());
			t = 0.5 * (unit_direction.y() + 1.0);
			return (1.0 - t) * vec3(0.3, 0.3, 0) + t * vec3(0.5, 0.7, 1.0);

		}


	vec3 unit_direction = unit_vector(r.direction());
	t = 0.5 * (unit_direction.y() + 1.0);
	return vec3(1, 1, 1);
	//return (1.0 - t) * vec3(1.0, 1.0, 1.0) + t * vec3(0.5, 0.7, 1.0);
}


int main()
{
	int angles[7] = { -90,-45 ,-30, 0, 30, 45 , 90 };
	const int samples_per_pixel = 50;
	ofstream myfile;
	for (int image = 0; image <= 30; image++) {
		myfile.open("scene" + std::to_string(image) + ".ppm");

		int nx = 2000; // number of columns width
		int ny = 1000; // number of rows hight
		myfile << "P3\n" << nx << " " << ny << "\n255\n";

		Triangle tri(vec3(-0.001, 0, 0), vec3(0, 0.1, 0), vec3(0, 0, -0.001));
		//float c = float(image) / float(10);

		camera cam(vec3(0, -3, 4.5), vec3(-0.5, 1, -3), vec3(image*0.1, 3,-2), -150, float(nx) / float(ny));


		for (int j = ny - 1; j >= 0; j--) {
			for (int i = 0; i < nx; i++) {
				float u = float(i) / float(nx);
				float v = float(j) / float(ny);

				ray r = cam.get_ray(u, v);
				//vec3 p = r.point_at_parameter(2.0);
				vec3 col = color(tri, r, 0);

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
