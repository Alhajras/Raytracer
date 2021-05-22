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
        camera() {
            origin = vec3(0, 2.0, 5);
            horizontal = vec3(4.0, 0.0, 0.0);
            vertical = vec3(0.0, 2.0, 0.0);
            lower_left_corner = vec3(-2.0, -1, -1);
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
