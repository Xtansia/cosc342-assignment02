/* $Rev: 250 $ */
#include <algorithm>
#include "CSG.h"

#include "utility.h"

CSG::CSG() : Object() {
  left = std::shared_ptr<Sphere>(new Sphere());
  right = std::shared_ptr<Sphere>(new Sphere());
  right->transform.translate(-1, 0, 0);
}

CSG::CSG(const CSG &csg) : Object(csg) {

}

CSG::~CSG() {

}

const CSG &CSG::operator=(const CSG &csg) {
  if (this != &csg) {
    Object::operator=(csg);
  }

  return *this;
}

bool nearer(RayIntersection a, RayIntersection b) {
  return (a < b);
}

void CSG::setupCSG(std::string type) {
  csgType = type;
}

std::vector<RayIntersection> CSG::intersect(const Ray &ray) const {
  csg_state csg_table[3][3] = {
    {OUTSIDE, OUTSIDE, OUTSIDE},
    {OUTSIDE, OUTSIDE, OUTSIDE},
    {OUTSIDE, OUTSIDE, OUTSIDE}
  };

  if (csgType == "INTERSECTION") {
    // make the csg_table appropriate for intersection
    csg_table[INSIDE][BORDER] = BORDER;
    csg_table[INSIDE][INSIDE] = INSIDE;
    csg_table[BORDER][INSIDE] = BORDER;
  } else if (csgType == "DIFFERENCE") {
    csg_table[INSIDE][OUTSIDE] = INSIDE;
    csg_table[BORDER][OUTSIDE] = BORDER;
    csg_table[INSIDE][BORDER] = BORDER;
  } else if (csgType == "UNION") {
    csg_table[INSIDE][OUTSIDE] = INSIDE;
    csg_table[BORDER][OUTSIDE] = BORDER;
    csg_table[INSIDE][INSIDE] = INSIDE;
    csg_table[OUTSIDE][INSIDE] = INSIDE;
    csg_table[BORDER][INSIDE] = INSIDE;
    csg_table[INSIDE][BORDER] = INSIDE;
    csg_table[OUTSIDE][BORDER] = BORDER;
    csg_table[BORDER][BORDER] = BORDER;
  }

  std::vector<RayIntersection> result;
  Ray inverseRay = transform.applyInverse(ray);

  std::vector<RayIntersection> leftIntersections;
  std::vector<RayIntersection> rightIntersections;

  leftIntersections = left->intersect(inverseRay);
  rightIntersections = right->intersect(inverseRay);

  // sort intersections
  std::sort(leftIntersections.begin(), leftIntersections.end(), nearer);
  std::sort(rightIntersections.begin(), rightIntersections.end(), nearer);

  int numLeftIs = leftIntersections.size();
  int numRightIs = rightIntersections.size();

  // We will assume that an odd number of intersections
  // indicates being inside an object. This is not correct for
  // glancing intersections. These artifacts could be fixed, up
  // to a point, by tracing a ray in the reverse direction and
  // looking for object intersections, however for this ray
  // tracer, we will just live with the (rare) glitches.
  csg_state leftState = OUTSIDE;
  csg_state rightState = OUTSIDE;

  if (numLeftIs % 2 == 1) {
    leftState = INSIDE;
  }

  if (numRightIs % 2 == 1) {
    rightState = INSIDE;
  }

  // We also need to keep track of the previous state.
  // This allows us to update correctly as we transition across a border
  // So if previousLeftState is inside, and we cross a border we end up outside
  csg_state previousLeftState = leftState;
  csg_state previousRightState = rightState;

  // iterate through both child objects' ray intersections
  int lIi = 0;
  int rIi = 0;

  RayIntersection possibleIntersection;

  while (lIi < numLeftIs || rIi < numRightIs) {
    // there's still work to do
    if (rIi == numRightIs ||
        (lIi < numLeftIs && leftIntersections[lIi] < rightIntersections[rIi])
       ) {
      // process leftIntersections[lIi]

      possibleIntersection.point = transform.apply(Point(
                                     leftIntersections[lIi].point));
      possibleIntersection.normal = transform.apply(Normal(
                                      leftIntersections[lIi].normal));
      possibleIntersection.material = leftIntersections[lIi].material;
      possibleIntersection.distance = (possibleIntersection.point - ray.point).norm()
                                      * sign(leftIntersections[lIi].distance);

      if (previousLeftState == OUTSIDE) {
        leftState = BORDER;
      } else if (previousLeftState == INSIDE) {
        leftState = BORDER;
      }

      lIi++;
    } else {
      // process rightIntersections[lIi]

      possibleIntersection.point = transform.apply(Point(
                                     rightIntersections[rIi].point));
      possibleIntersection.normal = transform.apply(Normal(
                                      rightIntersections[rIi].normal));
      possibleIntersection.material = rightIntersections[rIi].material;
      possibleIntersection.distance = (possibleIntersection.point - ray.point).norm()
                                      * sign(rightIntersections[rIi].distance);

      if (previousRightState == OUTSIDE) {
        rightState = BORDER;
      } else if (previousRightState == INSIDE) {
        rightState = OUTSIDE;
      }

      rIi++;
    }

    // Check the possible intersection against the
    // csg_table to see if it is actually a hitpoint of
    // the overall CSG object.  If it is a hitpoint, add
    // it to the result.
    if (csg_table[leftState][rightState] == BORDER
        || csg_table[leftState][rightState] == INSIDE) {
      if (possibleIntersection.normal.dot(ray.direction) > 0) {
        possibleIntersection.normal = -possibleIntersection.normal;
      }

      result.push_back(possibleIntersection);
    }

    // Update the left and/or the right tree to
    // ensure that they advance beyond their BORDER state,
    // as the tracing ray resumes its progression through
    // the scene.

    if (leftState == BORDER) {
      if (previousLeftState == OUTSIDE) {
        leftState = INSIDE;
      } else if (previousLeftState == INSIDE) {
        leftState = OUTSIDE;
      }
    }

    if (rightState == BORDER) {
      if (previousRightState == OUTSIDE) {
        rightState = INSIDE;
      } else if (previousRightState == INSIDE) {
        rightState = OUTSIDE;
      }
    }

    previousLeftState = leftState;
    previousRightState = rightState;
  }

  return result;
}
