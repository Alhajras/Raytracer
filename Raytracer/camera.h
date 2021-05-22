#ifndef CAMERA_H
#define CAMERA_H
//==============================================================================================
// Originally written in 2016 by Peter Shirley <ptrshrl@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related and
// neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public Domain Dedication
// along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//==============================================================================================

//#include "rtweekend.h"
#include "ray.h"

class camera {
    public:
        camera(vec3 lookfrom, vec3 lookat, vec3 vup, float vfov, float aspect) {
			vec3 u, v, w;
			float theta = vfov * (atan(1) * 4) / 180;
			float half_height = tan(theta / 2);
			float half_width = aspect * half_height;
			origin = lookfrom;
			w = unit_vector(lookfrom - lookat); 
			u = unit_vector(cross(vup, w));
			v = cross(w, u);
			lower_left_corner = vec3(-half_width, -half_height, -1.0);
            horizontal = 2 * half_width*u;
            vertical = 2 * half_height * v;
        }

        ray get_ray(double u, double v)  {
            return ray(
                origin,
                lower_left_corner + u*horizontal + v*vertical);
        }

    private:
		vec3 origin;
		vec3 lower_left_corner;
        vec3 horizontal;
        vec3 vertical;
};

#endif
