#pragma once

#ifndef CUBE_H_INCLUDED
#define CUBE_H_INCLUDED

#include "Object.h"

/**
 * \file
 * \brief Cube class header file.
 */

/**
 * \brief Class for Cube objects.
 *
 * This class provides and Object which is a cube that has it's center at
 * the origin, with it's faces a distance of 1 unit from the center, giving
 * a 2x2x2 cube.
 */
class Cube : public Object {

public:

  /**
   * \brief Cube default constructor.
   *
   * A newly constructed Cube has it's center at the origin,
   * and it's faces a distance of 1 unit from the center.
   * It may be moved, rotated and scaled through it's transform member.
   */
  Cube();

  /**
   * \brief Cube copy constructor.
   *
   * \param cube The Cube to copy.
   */
  Cube(const Cube &cube);

  /**
   * \brief Cube destructor.
   */
  ~Cube();

  /**
   * \brief Cube assignment operator.
   *
   * \param cube The Cube to assign to \c this.
   * \return A reference to \c this to allow for chaining of assignment.
   */
  const Cube &operator=(const Cube &cube);

  /**
   * \brief Cube-Ray intersection computation.
   *
   * The intersection of a Ray with the faces of a Cube are determined by
   * the Ray's intersection with the 8 planes at -1 and +1 for each axis,
   * and if the intersection occurs within the -1 -> +1 bounding box for
   * the axes laying on the plane. There should never be more than 2
   * intersections.
   *
   * \param ray The Ray to intersect with this Cube.
   * \return A list (std::vector) or intersections, which may be empty.
   */
  std::vector<RayIntersection> intersect(const Ray &ray) const;
};


#endif //CUBE_H_INCLUDED
