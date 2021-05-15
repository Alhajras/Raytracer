#ifndef _Triangle_H
#define _Triangle_H

#include "math.h"
#include "vec3.h"

class Triangle {
	vec3 A, B, C; // Three vectors that represent the triangelc corners
	vec3 normal;
	double distance;

public:

	Triangle();

	Triangle(vec3, vec3, vec3);

	// method functions
	vec3 getTriangleNormal() {
		vec3 CA(C.x() - A.x(), C.y() - A.y(), C.z() - A.z()); // This calculate the vector form CA
		vec3 BA(B.x() - A.x(), B.y() - A.y(), B.z() - A.z()); // This calculate the vector form CA
		normal = normalize(cross(CA, BA));
		return normal;
	}

	double getTriangleDistance() {
		normal = getTriangleNormal();
		distance = dot(normal, A);
		return distance; }

	virtual vec3 getNormalAt(vec3 point) {
		normal = getTriangleNormal();
		return normal;
	}

	virtual double findIntersection(vec3 ray_direction, vec3 ray_origin) {
		double a = dot(ray_direction, normal);

		normal = getTriangleNormal();
		distance = getTriangleDistance();

		if (a == 0) {
			// ray is parallel to the Triangle
			return -1;
		}
		else {
			// Check on this
			double b = dot(normal, operator+(ray_origin, negative(operator*(normal, distance))));

			double distance2plane = -1 * b / a;

			// understand this please
			double Qx = operator*(ray_direction, distance2plane).x() + ray_origin.x();
			double Qy = operator*(ray_direction, distance2plane).y() + ray_origin.y();
			double Qz = operator*(ray_direction, distance2plane).z() + ray_origin.z();

			vec3 Q(Qx, Qy, Qz);

			// [CAxQA].n >= 0 
			vec3 CA(C.x() - A.x(), C.y() - A.y(), C.z() - A.z()); // This calculate the vector form CA
			vec3 QA(Q.x() - A.x(), Q.y() - A.y(), Q.z() - A.z()); // This calculate the vector form CA
			double test1 = dot(cross(CA, QA), normal);
			// [BCxQC].n >= 0
			vec3 BC(B.x() - C.x(), B.y() - C.y(), B.z() - C.z()); // This calculate the vector form CA
			vec3 QC(Q.x() - C.x(), Q.y() - C.y(), Q.z() - C.z()); // This calculate the vector form CA
			double test2 = dot(cross(BC, QC), normal);

			// [ABxQB].n >= 0
			vec3 AB(A.x() - B.x(), A.y() - B.y(), A.z() - B.z()); // This calculate the vector form CA
			vec3 QB(Q.x() - B.x(), Q.y() - B.y(), Q.z() - B.z()); // This calculate the vector form CA
			double test3 = dot(cross(AB, QB), normal);


			if ((test1 >= 0) && (test2 >= 0) && (test3 >= 0))
			{
				// inside triangle
				return -1 * b / a;
			}
			else
			{
				// missided the triangle
				return -1;
			}
		}
	}

};

Triangle::Triangle() {
	A = vec3(1, 0, 0);
	B = vec3(0, 1, 0);
	C = vec3(0, 0, 1);
}

Triangle::Triangle(vec3 pointA, vec3 pointB, vec3 pointC) {
	A = pointA;
	B = pointB;
	C = pointC;
}

#endif
