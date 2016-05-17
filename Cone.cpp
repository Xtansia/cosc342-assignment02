/* $Rev: 250 $ */
#include "Cone.h"

#include "utility.h"

Cone::Cone() : Object() {

}

Cone::Cone(const Cone &cone) : Object(cone) {

}

Cone::~Cone() {

}

const Cone &Cone::operator=(const Cone &cone) {
    if (this != &cone) {
        Object::operator=(cone);
    }
    return *this;
}

std::vector<RayIntersection> Cone::intersect(const Ray &ray) const {
    std::vector<RayIntersection> result;

    Ray inverseRay = transform.applyInverse(ray);

    // Intersection is of the form ad^2 + bd + c, where d = distance along the ray

    double dirX = inverseRay.direction(0);
    double dirY = inverseRay.direction(1);
    double dirZ = inverseRay.direction(2);
    double pntX = inverseRay.point(0);
    double pntY = inverseRay.point(1);
    double pntZ = inverseRay.point(2);

    double a = dirX * dirX + dirY * dirY - dirZ * dirZ;
    double b = 2 * pntX * dirX + 2 * pntY * dirY - 2 * pntZ * dirZ;
    double c = pntX * pntX + pntY * pntY - pntZ * pntZ;

    double dStart = (1 - pntZ) / dirZ;
    double dEnd = (0 - pntZ) / dirZ;

    RayIntersection hit;
    hit.material = material;

    double b2_4ac = b * b - 4 * a * c;
    double d;
    switch (sign(b2_4ac)) {
        case -1:
            // No intersections
            break;
        case 0:
            // One intersection
            d = -b / (2 * a);
            if (d > dStart && d < dEnd) {
                // Intersection is in front of the ray's start point
                hit.point = transform.apply(Point(inverseRay.point + d * inverseRay.direction));

                // Normal is point with negated z
                Normal norm = inverseRay.point + d * inverseRay.direction;
                norm(2) *= -1;

                hit.normal = transform.apply(Normal(norm));
                if (hit.normal.dot(ray.direction) > 0) {
                    hit.normal = -hit.normal;
                }
                hit.distance = (hit.point - ray.point).norm() * sign(d);
                result.push_back(hit);
            }
            break;
        case 1:
            // Two intersections
            d = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
            if (d > dStart && d < dEnd) {
                // Intersection is in front of the ray's start point
                hit.point = transform.apply(Point(inverseRay.point + d * inverseRay.direction));

                // Normal is point with negated z
                Normal norm = inverseRay.point + d * inverseRay.direction;
                norm(2) *= -1;

                hit.normal = transform.apply(Normal(norm));
                if (hit.normal.dot(ray.direction) > 0) {
                    hit.normal = -hit.normal;
                }
                hit.distance = (hit.point - ray.point).norm() * sign(d);
                result.push_back(hit);
            }

            d = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
            if (d > dStart && d < dEnd) {
                // Intersection is in front of the ray's start point
                hit.point = transform.apply(Point(inverseRay.point + d * inverseRay.direction));

                // Normal is point with negated z
                Normal norm = inverseRay.point + d * inverseRay.direction;
                norm(2) *= -1;

                hit.normal = transform.apply(Normal(norm));
                if (hit.normal.dot(ray.direction) > 0) {
                    hit.normal = -hit.normal;
                }
                hit.distance = (hit.point - ray.point).norm() * sign(d);
                result.push_back(hit);
            }
            break;
        default:
            // Shouldn't be possible, but just in case
            std::cerr << "Something's wrong - sign(x) should be -1, +1 or 0" << std::endl;
            exit(-1);
            break;
    }

    // Base circle
    if (std::abs(dirZ) > epsilon) {
        // Find intersect with plane Z=1
        d = dStart;
        Point intersection = inverseRay.point + d * inverseRay.direction;
        // Check intersection is in base circle
        if (intersection(0) * intersection(0) + intersection(1) * intersection(1) < 1) {
            hit.point = transform.apply(Point(intersection));
            hit.normal = transform.apply(Normal(0, 0, 1));
            hit.distance = (hit.point - ray.point).norm() * sign(d);
            result.push_back(hit);
        }
    }

    return result;
}
