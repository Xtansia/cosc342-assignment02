//
// Copyright (c) 2016  Thomas Farr.
//

#ifndef RAYTRACER_CUBE_H
#define RAYTRACER_CUBE_H

#include "Object.h"

class Cube : public Object {
public:
    Cube();

    Cube(const Cube &cube);

    ~Cube();

    const Cube &operator=(const Cube &cube);

    std::vector<RayIntersection> intersect(const Ray &ray) const;
};


#endif //RAYTRACER_CUBE_H
