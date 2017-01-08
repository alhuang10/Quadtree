/**
 * Copyright (c) 2012 the Massachusetts Institute of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 **/

#include "./IntersectionDetection.h"

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#include "./Line.h"
#include "./Vec.h"

// Detect if lines l1 and l2 will intersect between now and the next time step.
IntersectionType intersect(Line *l1, Line *l2, double time) {
  assert(compareLines(l1, l2) < 0);

  if(intersectLines(l1->p1, l1->p2, l2->p1, l2->p2)) {
    return ALREADY_INTERSECTED;
  }

	// p1 is l2->p1 offset by the relative velocity of l2 wrt l1.
	Vec p1 = {l2->fut_p1.x - l1->velocity.x*time, l2->fut_p1.y - l1->velocity.y*time};
	Vec p2 = {l2->fut_p2.x - l1->velocity.x*time, l2->fut_p2.y - l1->velocity.y*time};

    int num_line_intersections = 0;
    bool top_intersected = false;
    bool bottom_intersected = false;


    //check to see if the future points line intersects with l1
    if (intersectLines(l1->p1, l1->p2, p1, p2)) {
      num_line_intersections++;
    }
    //check to see if l2->p1 and l2->fut_p1 are on same side of l1 line and fut_l1 line respectively
    //(compare l2->p1 value for l1 to l2->fut_p1 value for fut_l1)
    if (intersectLines(l1->p1, l1->p2, p1, l2->p1)) {
      num_line_intersections++;
      top_intersected = true;

    }

    //same thing but with p2
    if (intersectLines(l1->p1, l1->p2, p2, l2->p2)) {
      num_line_intersections++;
      bottom_intersected = true;
    }

    //Can only have a max of two in this variable and this is case where L2 is the arrow and L1 is the wall
    if (num_line_intersections == 2) {
      return L2_WITH_L1;
    }


    if (pointInParallelogram(l1->p1, l2->p1, l2->p2, p1, p2)
        && pointInParallelogram(l1->p2, l2->p1, l2->p2, p1, p2)) {
      return L1_WITH_L2;
    }

    if (num_line_intersections == 0) {
      return NO_INTERSECTION;
    }

    //Vec_angle returns the angle difference between v1 and v2 (v1 angle - v2 angle)
    //"Principal arc tangent of y/x, in the interval [-pi,+pi] radians."
    double angle = atan2(l1->p1.y-l1->p2.y, l1->p1.x-l1->p2.x) - atan2(l2->p1.y-l2->p2.y, l2->p1.x-l2->p2.x);

    if (top_intersected) {
      if (angle < 0) {
        return L2_WITH_L1;
      } else {
        return L1_WITH_L2;
      }
    }

    if (bottom_intersected && angle > 0) {
      return L2_WITH_L1;
    }

    return L1_WITH_L2;
}

// Check if a point is in the parallelogram.
bool pointInParallelogram(Vec point, Vec p1, Vec p2, Vec p3, Vec p4) {
  double d1 = direction(p1, p2, point);
  double d2 = direction(p3, p4, point);
  double d3 = direction(p1, p3, point);
  double d4 = direction(p2, p4, point);

  return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0))
      && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)));
}



// Check if two lines intersect.
bool intersectLines(Vec p1, Vec p2, Vec p3, Vec p4) {
  // Relative orientation
	// this is for the vertical line case
	if (p1.x == p2.x && p3.x == p4.x && p1.x == p3.x &&
			((p1.y < p3.y && p2.y > p3.y)
			||(p1.y > p3.y && p2.y < p3.y)
			||(p1.y < p4.y && p2.y > p4.y)
			||(p1.y > p4.y && p2.y < p4.y))) {
		return true;
	}
	// this is for all other cases
	return (((p4.x - p1.x)*(p2.y - p1.y)-(p2.x - p1.x)*(p4.y - p1.y)) *
			((p2.y - p1.y)*(p3.x - p1.x)-(p2.x - p1.x)*(p3.y - p1.y)) < 0)
		 &&(((p3.y - p1.y)*(p4.x - p3.x)-(p4.y - p3.y)*(p3.x - p1.x)) *
		    ((p4.x - p3.x)*(p3.y - p2.y)-(p3.x - p2.x)*(p4.y - p3.y)) < 0);
}

// Staff provided implementation
// Only used when either of the lines is vertical (slope cannot be calculated).
Vec verticalGetIntersectionPoint(Vec p1, Vec p2, Vec p3, Vec p4) {
  double u;
  u = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x))
      / ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y));
  return Vec_add(p1, Vec_multiply(Vec_subtract(p2, p1), u));
}

//New method
Vec getIntersectionPoint(Vec p1, Vec p2, Vec p3, Vec p4) {

  if (p1.x == p2.x || p3.x == p4.x) {
	  return verticalGetIntersectionPoint(p1, p2, p3, p4);
  }
  double m1 = (p2.y-p1.y)/(p2.x-p1.x);
  double m2 = (p4.y-p3.y)/(p4.x-p3.x);
  double b1 = p1.y-m1*p1.x;
  double b2 = p3.y-m2*p3.x;
  double x_int = (b2-b1)/(m1-m2);
  double y_int = m1*x_int + b1;
  Vec newVector = {x_int, y_int};
  return newVector;
}

// Check the direction of two lines (pi, pj) and (pi, pk).
inline double direction(Vec pi, Vec pj, Vec pk) {
  return crossProduct(pk.x - pi.x, pk.y - pi.y, pj.x - pi.x, pj.y - pi.y);
}

// Check if a point pk is in the line segment (pi, pj).
// pi, pj, and pk must be collinear.
inline bool onSegment(Vec pi, Vec pj, Vec pk) {
	//return ((pk.x - pi.x) * (pk.x - pj.x) <= 0) & ((pk.y - pi.y) * (pk.y - pj.y) <= 0);

	return (((pi.x <= pk.x && pk.x <= pj.x) || (pj.x <= pk.x && pk.x <= pi.x))
      && ((pi.y <= pk.y && pk.y <= pj.y) || (pj.y <= pk.y && pk.y <= pi.y)));

}

// Calculate the cross product.
inline double crossProduct(double x1, double y1, double x2, double y2) {
  return x1 * y2 - x2 * y1;
}

