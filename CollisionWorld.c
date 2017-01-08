/** 
 * CollisionWorld.c -- detect and handle line segment intersections
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

#include "./CollisionWorld.h"

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#include "./IntersectionDetection.h"
#include "./IntersectionEventList.h"
#include "./Line.h"
#include "./Quadtree.h"

void setStartAndMid(IntersectionEventNode * headNode, IntersectionEventNode ** start, IntersectionEventNode ** mid);
IntersectionEventNode * combineSortedLists(IntersectionEventNode * start, IntersectionEventNode * mid);
void mergeSort(IntersectionEventNode ** headNode);


CollisionWorld* CollisionWorld_new(const unsigned int capacity) {
  assert(capacity > 0);
  __cilkrts_set_param("nworkers", "1"); 
  CollisionWorld* collisionWorld = malloc(sizeof(CollisionWorld));
  if (collisionWorld == NULL) {
    return NULL;
  }

  collisionWorld->numLineWallCollisions = 0;
  collisionWorld->numLineLineCollisions = 0;
  collisionWorld->timeStep = 0.5;
  collisionWorld->lines = malloc(capacity * sizeof(Line*));
  collisionWorld->numOfLines = 0;
  return collisionWorld;
}

void CollisionWorld_delete(CollisionWorld* collisionWorld) {
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    free(collisionWorld->lines[i]);
  }
  free(collisionWorld->lines);
  free(collisionWorld);
}

unsigned int CollisionWorld_getNumOfLines(CollisionWorld* collisionWorld) {
  return collisionWorld->numOfLines;
}

void CollisionWorld_addLine(CollisionWorld* collisionWorld, Line *line) {
  collisionWorld->lines[collisionWorld->numOfLines] = line;
  collisionWorld->numOfLines++;
}

Line* CollisionWorld_getLine(CollisionWorld* collisionWorld,
                             const unsigned int index) {
  if (index >= collisionWorld->numOfLines) {
    return NULL;
  }
  return collisionWorld->lines[index];
}

void CollisionWorld_updateLines(CollisionWorld* collisionWorld,
		IntersectionEventListReducer * X) {


	LineNode * lineNode = NULL;
	// find all line line collisions:
	traverseQuadtree(globalQuadtree, X, lineNode);

	collisionWorld->numLineLineCollisions += processCollisionList(X->value, collisionWorld);

	// update the positions of all the lines in the collisionworld.
	CollisionWorld_updatePosition(collisionWorld);

	//then find and process all wall line collisions
	collisionWorld->numLineWallCollisions += getWallCollisions(globalQuadtree);

  	updateNode(globalQuadtree);
	attachBuffers(globalQuadtree);
}

void CollisionWorld_updatePosition(CollisionWorld* collisionWorld) {
//  double t = collisionWorld->timeStep;
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];

    //When we updatePosition, calculate what the position will be next time step
    line->p1 = line->fut_p1;
    line->p2 = line->fut_p2;

    //Will this change p1/p2?
    updateLineFuturePoints(line);
  }
}

void CollisionWorld_lineWallCollision(CollisionWorld* collisionWorld) {
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];
    bool collide = false;

    // Right side
    if ((line->p1.x > BOX_XMAX || line->p2.x > BOX_XMAX)
        && (line->velocity.x > 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Left side
    if ((line->p1.x < BOX_XMIN || line->p2.x < BOX_XMIN)
        && (line->velocity.x < 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Top side
    if ((line->p1.y > BOX_YMAX || line->p2.y > BOX_YMAX)
        && (line->velocity.y > 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Bottom side
    if ((line->p1.y < BOX_YMIN || line->p2.y < BOX_YMIN)
        && (line->velocity.y < 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Update total number of collisions.
    if (collide == true) {
      collisionWorld->numLineWallCollisions++;
    }
  }
}

void mergeSort(IntersectionEventNode ** headNode){

	IntersectionEventNode * head = *headNode;
	IntersectionEventNode * start;
	IntersectionEventNode * mid;

	//base case
	if((head == NULL) || (head->next == NULL)){
		return;
	}

	//sets the start and mid indices
	setStartAndMid(head, &start, &mid);

	mergeSort(&start);
	mergeSort(&mid);

	*headNode = combineSortedLists(start, mid);
	

}

IntersectionEventNode * combineSortedLists(IntersectionEventNode * start, IntersectionEventNode * mid){

	IntersectionEventNode * resultHeadPointer = NULL;

	//ending case once we reach end of a or end of b
	if (start == NULL){
		return mid;
	}
	else if (mid == NULL){
		return start;
	}

	//if less than zero than the node start points to is less than the node mid points to
	if (IntersectionEventNode_compareData(start, mid) < 0){

		resultHeadPointer = start;
		resultHeadPointer->next = combineSortedLists(start->next, mid);
	}

	else{
		resultHeadPointer = mid;
		resultHeadPointer->next = combineSortedLists(start, mid->next);
	}

	return resultHeadPointer;
}

void setStartAndMid(IntersectionEventNode * headNode, IntersectionEventNode ** start, IntersectionEventNode ** mid){

	//Increment slow by 1 and fast by 2 until we get to the end of the list
	IntersectionEventNode * slow;
	IntersectionEventNode * fast;

	if(headNode == NULL || headNode->next == NULL){

		*start = headNode; //will be headnode in the 1 length case and null in the length zero case
		*mid = NULL;
	}
	else{
		slow = headNode;
		fast = headNode->next;

		while(fast != NULL){

			fast = fast->next;

			//Haven't reached end of array yet
			if (fast != NULL){
				fast = fast->next;
				slow = slow->next;
			}
		}

		*start = headNode;
		*mid = slow->next;
		slow->next = NULL; //cutoff the end
	}
}


int processCollisionList(IntersectionEventList intersectionEventList, CollisionWorld * collisionWorld) {
	  int count = 0;
	  // Sort the intersection event list.
	  IntersectionEventNode* startNode = intersectionEventList.head;

	  mergeSort(&startNode);
 			  
	  IntersectionEventNode * curNode = startNode;
	  intersectionEventList.head = startNode;

	  while (curNode != NULL) {
	    CollisionWorld_collisionSolver(collisionWorld, curNode->l1, curNode->l2,
	                                   curNode->intersectionType);
	    curNode = curNode->next;
	    count ++;
	  }

	  IntersectionEventList_deleteNodes(&intersectionEventList);
   	  return count;
}

void CollisionWorld_detectIntersection(CollisionWorld* collisionWorld) {
  IntersectionEventList intersectionEventList = IntersectionEventList_make();

  // Test all line-line pairs to see if they will intersect before the
  // next time step.
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *l1 = collisionWorld->lines[i];

    for (int j = i + 1; j < collisionWorld->numOfLines; j++) {
      Line *l2 = collisionWorld->lines[j];

      IntersectionType intersectionType =
          intersect(l1, l2, collisionWorld->timeStep);
      if (intersectionType != NO_INTERSECTION) {
        IntersectionEventList_appendNode(&intersectionEventList, l1, l2,
                                         intersectionType);
        collisionWorld->numLineLineCollisions++;
      }
    }
  }

  // Sort the intersection event list.
  IntersectionEventNode* startNode = intersectionEventList.head;
  while (startNode != NULL) {
    IntersectionEventNode* minNode = startNode;
    IntersectionEventNode* curNode = startNode->next;
    while (curNode != NULL) {
      if (IntersectionEventNode_compareData(curNode, minNode) < 0) {
        minNode = curNode;
      }
      curNode = curNode->next;
    }
    if (minNode != startNode) {
      IntersectionEventNode_swapData(minNode, startNode);
    }
    startNode = startNode->next;
  }

  // Call the collision solver for each intersection event.
  IntersectionEventNode* curNode = intersectionEventList.head;

  while (curNode != NULL) {
    CollisionWorld_collisionSolver(collisionWorld, curNode->l1, curNode->l2,
                                   curNode->intersectionType);
    curNode = curNode->next;
  }

  IntersectionEventList_deleteNodes(&intersectionEventList);
}

unsigned int CollisionWorld_getNumLineWallCollisions(
    CollisionWorld* collisionWorld) {
  return collisionWorld->numLineWallCollisions;
}

unsigned int CollisionWorld_getNumLineLineCollisions(
    CollisionWorld* collisionWorld) {
  return collisionWorld->numLineLineCollisions;
}

void CollisionWorld_collisionSolver(CollisionWorld* collisionWorld,
                                    Line *l1, Line *l2,
                                    IntersectionType intersectionType) {

  // Despite our efforts to determine whether lines will intersect ahead
  // of time (and to modify their velocities appropriately), our
  // simplified model can sometimes cause lines to intersect.  In such a
  // case, we compute velocities so that the two lines can get unstuck in
  // the fastest possible way, while still conserving momentum and kinetic
  // energy.
  if (intersectionType == ALREADY_INTERSECTED) {
    Vec p = getIntersectionPoint(l1->p1, l1->p2, l2->p1, l2->p2);

    double l1_p1_p = Vec_length(Vec_subtract(l1->p1, p));
    double l1_p2_p = l1->length - l1_p1_p;

    double l2_p1_p = Vec_length(Vec_subtract(l2->p1, p));
	double l2_p2_p = l2->length - l2_p1_p;

	//Pre-computing distance to intersection
    if (l1_p1_p < l1_p2_p) {
      l1->velocity = Vec_multiply(Vec_divide(Vec_subtract(l1->p2, p), l1_p2_p),
                                  Vec_length(l1->velocity));

	  updateLineFuturePoints(l1);
    } else {
      l1->velocity = Vec_multiply(Vec_divide(Vec_subtract(l1->p1, p), l1_p1_p),
                                  Vec_length(l1->velocity));

      updateLineFuturePoints(l1);
    }
    if (l2_p1_p < l2_p2_p) {
      l2->velocity = Vec_multiply(Vec_divide(Vec_subtract(l2->p2, p), l2_p2_p),
                                  Vec_length(l2->velocity));

      updateLineFuturePoints(l2);
    } else {
      l2->velocity = Vec_multiply(Vec_divide(Vec_subtract(l2->p1, p), l2_p1_p),
                                  Vec_length(l2->velocity));

      updateLineFuturePoints(l2);
    }
    return;
  }

  // Compute the collision face/normal vectors.
  Vec face;
  Vec normal;
  if (intersectionType == L1_WITH_L2) {
    Vec v = Vec_makeFromLine(*l2);
    face = Vec_divide(v, l2->length);
  } else {
    Vec v = Vec_makeFromLine(*l1);
    face = Vec_divide(v, l1->length);
  }
  normal = Vec_orthogonal(face);

  // Obtain each line's velocity components with respect to the collision
  // face/normal vectors.
  double v1Face = Vec_dotProduct(l1->velocity, face);
  double v2Face = Vec_dotProduct(l2->velocity, face);
  double v1Normal = Vec_dotProduct(l1->velocity, normal);
  double v2Normal = Vec_dotProduct(l2->velocity, normal);

  // Compute the mass of each line (we simply use its length).
  double m1 = l1->length;
  double m2 = l2->length;

  // Perform the collision calculation (computes the new velocities along
  // the direction normal to the collision face such that momentum and
  // kinetic energy are conserved).
  double newV1Normal = ((m1 - m2) / (m1 + m2)) * v1Normal
      + (2 * m2 / (m1 + m2)) * v2Normal;
  double newV2Normal = (2 * m1 / (m1 + m2)) * v1Normal
      + ((m2 - m1) / (m2 + m1)) * v2Normal;

  // Combine the resulting velocities.
  l1->velocity = Vec_add(Vec_multiply(normal, newV1Normal),
                         Vec_multiply(face, v1Face));

  updateLineFuturePoints(l1);

  l2->velocity = Vec_add(Vec_multiply(normal, newV2Normal),
                         Vec_multiply(face, v2Face));

  updateLineFuturePoints(l2);

  return;
}
