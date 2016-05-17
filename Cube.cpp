//
// Copyright (c) 2016  Thomas Farr.
//

#include <cassert>
#include "Cube.h"
#include "utility.h"

Cube::Cube() : Object() {

}

Cube::Cube(const Cube &cube) : Object(cube) {

}

Cube::~Cube() {

}

const Cube &Cube::operator=(const Cube &cube) {
    if (this != &cube) {
        Object::operator=(cube);
    }
    return *this;
}

std::vector<RayIntersection> Cube::intersect(const Ray &ray) const {

    std::vector<RayIntersection> result;

    Ray inverseRay = transform.applyInverse(ray);

    double dirX = inverseRay.direction(0);
    double dirY = inverseRay.direction(1);
    double dirZ = inverseRay.direction(2);
    double pntX = inverseRay.point(0);
    double pntY = inverseRay.point(1);
    double pntZ = inverseRay.point(2);

    RayIntersection hit;
    hit.material = material;

    double d;

    // Check YZ planes
    // Check if parallel
    if (std::abs(dirX) > epsilon) {
        d = (-1 - pntX) / dirX;
        // intersects -ve YZ plane
        if (std::abs(pntY + d * dirY) <= 1 && std::abs(pntZ + d * dirZ) <= 1) {
            hit.point = transform.apply(Point(inverseRay.point + d * inverseRay.direction));
            hit.normal = transform.apply(Normal(-1, 0, 0));
            if (hit.normal.dot(ray.direction) > 0) {
                hit.normal = -hit.normal;
            }
            hit.distance = (hit.point - ray.point).norm() * sign(d);
            result.push_back(hit);
        }

        d = (1 - pntX) / dirX;
        // intersects +ve YZ plane
        if (std::abs(pntY + d * dirY) <= 1 && std::abs(pntZ + d * dirZ) <= 1) {
            hit.point = transform.apply(Point(inverseRay.point + d * inverseRay.direction));
            hit.normal = transform.apply(Normal(1, 0, 0));
            if (hit.normal.dot(ray.direction) > 0) {
                hit.normal = -hit.normal;
            }
            hit.distance = (hit.point - ray.point).norm() * sign(d);
            result.push_back(hit);
        }
    } else if (pntX < -1 || pntX > 1) {
        // Parallel to yz-plane and not inside cube 'range', so can't intersect
        return result;
    }

    // Check XZ planes
    // Check if parallel
    if (std::abs(dirY) > epsilon) {
        d = (-1 - pntY) / dirY;
        // intersects -ve XZ plane
        if (std::abs(pntX + d * dirX) <= 1 && std::abs(pntZ + d * dirZ) <= 1) {
            hit.point = transform.apply(Point(inverseRay.point + d * inverseRay.direction));
            hit.normal = transform.apply(Normal(0, -1, 0));
            if (hit.normal.dot(ray.direction) > 0) {
                hit.normal = -hit.normal;
            }
            hit.distance = (hit.point - ray.point).norm() * sign(d);
            result.push_back(hit);
        }

        d = (1 - pntY) / dirY;
        // intersects +ve XZ plane
        if (std::abs(pntX + d * dirX) <= 1 && std::abs(pntZ + d * dirZ) <= 1) {
            hit.point = transform.apply(Point(inverseRay.point + d * inverseRay.direction));
            hit.normal = transform.apply(Normal(0, 1, 0));
            if (hit.normal.dot(ray.direction) > 0) {
                hit.normal = -hit.normal;
            }
            hit.distance = (hit.point - ray.point).norm() * sign(d);
            result.push_back(hit);
        }
    } else if (pntY < -1 || pntY > 1) {
        // Parallel to xz-plane and not inside cube 'range', so can't intersect
        return result;
    }

    // Check XY planes
    // Check if parallel
    if (std::abs(dirZ) > epsilon) {
        d = (-1 - pntZ) / dirZ;
        // intersects -ve XY plane
        if (std::abs(pntX + d * dirX) <= 1 && std::abs(pntY + d * dirY) <= 1) {
            hit.point = transform.apply(Point(inverseRay.point + d * inverseRay.direction));
            hit.normal = transform.apply(Normal(0, 0, -1));
            if (hit.normal.dot(ray.direction) > 0) {
                hit.normal = -hit.normal;
            }
            hit.distance = (hit.point - ray.point).norm() * sign(d);
            result.push_back(hit);
        }

        d = (1 - pntZ) / dirZ;
        // intersects +ve XY plane
        if (std::abs(pntX + d * dirX) <= 1 && std::abs(pntY + d * dirY) <= 1) {
            hit.point = transform.apply(Point(inverseRay.point + d * inverseRay.direction));
            hit.normal = transform.apply(Normal(0, 1, 0));
            if (hit.normal.dot(ray.direction) > 0) {
                hit.normal = -hit.normal;
            }
            hit.distance = (hit.point - ray.point).norm() * sign(d);
            result.push_back(hit);
        }
    } else if (pntZ < -1 || pntZ > 1) {
        // Parallel to xy-plane and not inside cube 'range', so can't intersect
        return result;
    }

    assert(result.size() <= 2);

    return result;
}