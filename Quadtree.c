/*
 * Quadtree.c
 *
 *  Created on: Oct 3, 2015
 *      Author: alhuang
 */

/*
Need to write quadtree instantiation method

Figure out where to add passing down information down the quadtree

*/
#include "./Quadtree.h"
#include "./Line.h"
#include "./Vec.h"
#include "./CollisionWorld.h"
#include "./IntersectionEventList.h"

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <cilk/cilk.h>
#include <cilk/reducer.h>
//#include <cilk/cilk_stub.h>

int nodeContainsPoint(Node * node, Vec * v);

//Uses node_contains to return the quadrants that the line/traversal parallelogram is located in
quadrant_t getLineQuadrant(Node * node, Line * l);

// get's the quadrant within the node that the vector belongs to.
quadrant_t getPointQuadrant(Node *node, Vec * vector);

//If necessary (too many lines), split up the node into 4
void divideNode(Node * node);
// ======================================================
inline int overlapsRight(Line *line);
inline int overlapsLeft(Line *line);
inline int overlapsTop(Line *line);
inline int overlapsBottom(Line *line);
// =======================================================


//void allocateLineNodeList(int numberOfLines) {
//	allLineNodes = malloc(numberOfLines * sizeof(LineNode *));
//}

int maxLines = 50;

int nodeContainsPoint(Node *node, Vec * v){

	return v->x > node->xMin
			&& v->x < node->xMax
			&& v->y > node->yMin
			&& v->y < node->yMax;
}


int nodeContainsLine(Node *node, Line *l, double time){

	Vec * p1;
	Vec * p2;
	Vec * line_p1 = &(l->p1);
	Vec * line_p2 = &(l->p2);
//	Vec p;
	// Get the parallelogram.
//	p = Vec_add(*line_p1, Vec_multiply(l->velocity, time));
	p1 = &(l->fut_p1);
//	p = Vec_add(*line_p1, Vec_multiply(l->velocity, time));
	p2 = &(l->fut_p2);
	return nodeContainsPoint(node, p1)
			&& nodeContainsPoint(node, p2)
			&& nodeContainsPoint(node, line_p1)
			&& nodeContainsPoint(node, line_p2);
}

/*
Precondition: point is within one of the four quadrants.
If the line is contained fully within one quadrant, it'll return the quadrant it's in.
*/
quadrant_t getPointQuadrant(Node *node, Vec * vector) {
	double xMid = (node->xMin + node->xMax)/2.0;
	double yMid = (node->yMin + node->yMax)/2.0;
	if (vector->x >= xMid) {
		if (vector->y >= yMid) {
			return NE;
		}
		return SE;
	}
	if (vector->y >= yMid) {
		return NW;
	}
	return SW;
}

/*
Precondition: the line is fully inside the original node.
If the line is contained fully within one quadrant, it'll return:
0: NW
1: NE
2: SE
3: SW
If it's not fully in any quadrant, returns -1.
*/
quadrant_t getLineQuadrant(Node * node, Line * line){
	Vec *p1;
	Vec *p2;
	Vec *line_p1 = &(line->p1);
	Vec *line_p2 = &(line->p2);
//	Vec p;

	// Get the parallelogram.
	p1 = &(line->fut_p1);
	//	p = Vec_add(*line_p1, Vec_multiply(l->velocity, time));
	p2 = &(line->fut_p2);

	quadrant_t lineFirstPoint = getPointQuadrant(node, line_p1);
	quadrant_t lineSecondPoint = getPointQuadrant(node, line_p2);
	quadrant_t firstPoint = getPointQuadrant(node, p1);
	quadrant_t secondPoint = getPointQuadrant(node, p2);

	if (lineFirstPoint == lineSecondPoint && firstPoint == secondPoint
			&& lineFirstPoint == firstPoint) {
		return firstPoint;
	}
	return NONE;
}


// This instantiates the root of the initial quadtree.
Node * instantiateRoot(CollisionWorld * collisionWorld) {
	Node * root = create_node(BOX_XMIN, BOX_XMAX, BOX_YMIN, BOX_YMAX);
	root->parent = NULL;
	if (collisionWorld->numOfLines == 0) {
		return root;
	}
	// root->enclosedLines = malloc(collisionWorld->numOfLines * sizeof(Line *));

	for (int i = 0; i < collisionWorld->numOfLines; i++) {
		addQuadtreeLineNode(root, createLineNode(root->lines, collisionWorld->lines[i]));
	}
	divideNode(root);
	return root;
}

void attachBuffers(Node * node) {
	//Attach the lines in the buffer to the lines currently in the node
	if (node->bufferLineCount != 0) {
		if (node->numberOfLines == 0) {
			node->firstQuadtreeLineNode = node->bufferEnd;
		}
		node->bufferEnd->next = node->lines;
		node->lines = node->buffer;
		node->buffer = NULL;
		node->bufferEnd = NULL;
		node->numberOfLines += node->bufferLineCount;
		node->bufferLineCount = 0;
	}
	if (node->nw !=NULL) {
		attachBuffers(node->nw);
		attachBuffers(node->ne);
		attachBuffers(node->sw);
		attachBuffers(node->se);
	} else {
		divideNode(node);
	}
}

void insertLineNodeUpwardDuringUpdate(Node * node, LineNode * lineNode) {
	int inQuadrant = nodeContainsLine(node, lineNode->line, globalTimeStep);
	if (inQuadrant == 0) {
		if (node->parent != NULL) {
			insertLineNodeUpwardDuringUpdate(node->parent, lineNode);
			return;
		}
		else {
			insertLineNodeDownwardDuringUpdate(node, lineNode);
			return;
		}
	}
	else {
		insertLineNodeDownwardDuringUpdate(node, lineNode);
	}
}

void insertLineNodeDownwardDuringUpdate(Node * node, LineNode * lineNode) {
	if (node->nw == NULL) {
		addToBuffer(node, lineNode);
		return;
	}
	quadrant_t quadrant = getLineQuadrant(node, lineNode->line);
	Node * desiredNode = NULL;
	switch(quadrant) {
		case NONE:
			addToBuffer(node, lineNode);
			break;
		case NW:
			desiredNode = node->nw;
			insertLineNodeDownwardDuringUpdate(desiredNode, lineNode);
			break;
		case NE:
			desiredNode = node->ne;
			insertLineNodeDownwardDuringUpdate(desiredNode, lineNode);
			break;
		case SE:
			desiredNode = node->se;
			insertLineNodeDownwardDuringUpdate(desiredNode, lineNode);
			break;
		case SW:
			desiredNode = node->sw;
			insertLineNodeDownwardDuringUpdate(desiredNode, lineNode);
			break;
		default:
			break;
	}
}

//Starting from the root, call this function on each node in order to test each line to see
//if it belongs in the node still
void updateNode(Node * root) {
	LineNode * currentLineNode = root->lines;
	LineNode * previousLineNode = createLineNode(root->lines,NULL);
	LineNode * firstLineNode = previousLineNode;
	LineNode * tempLineNode;

//	for (int i = 0; i < root->numberOfLines; i++) {
	while (currentLineNode != NULL) {

		// this is what the next linenode we process is
		tempLineNode = currentLineNode->next;
		Line * line = currentLineNode->line;
		int contains = nodeContainsLine(root, line, globalTimeStep);
		if (contains == 0) { //linenode not in quadtreenode
			if (root->parent != NULL) {
				// go to the parent
				insertLineNodeUpwardDuringUpdate(root->parent, currentLineNode);
				root->numberOfLines--;
				previousLineNode->next = tempLineNode;
			} else {
				previousLineNode = currentLineNode;
			}
		}
		else { //line is in the node
			if (root->nw != NULL) {
				quadrant_t quadrant = getLineQuadrant(root, line);
				Node * desiredNode;
				//assign linenode to a subquadrant or to node itself
				switch (quadrant) {
				case NW:
					desiredNode = root->nw;
					insertLineNodeDownwardDuringUpdate(desiredNode, currentLineNode);
					root->numberOfLines--;
					previousLineNode->next = tempLineNode;
					break;
				case NE:
					desiredNode = root->ne;
					insertLineNodeDownwardDuringUpdate(desiredNode, currentLineNode);
					root->numberOfLines--;
					previousLineNode->next = tempLineNode;
					break;
				case SE:
					desiredNode = root->se;
					insertLineNodeDownwardDuringUpdate(desiredNode, currentLineNode);
					root->numberOfLines--;
					previousLineNode->next = tempLineNode;
					break;
				case SW:
					desiredNode = root->sw;
					insertLineNodeDownwardDuringUpdate(desiredNode, currentLineNode);
					root->numberOfLines--;
					previousLineNode->next = tempLineNode;
					break;
				default:
					previousLineNode = currentLineNode;
					break;
				}
			}
			else { //no children
				//if no children and the line is in the node, we skip it by assigning
				//previousLineNode to current and moving currentLineNode forward in the code
				//5 lines below
				previousLineNode = currentLineNode;
			}
		}

		//no matter what we set currentLineNode to templineNode (which is the next element)
		currentLineNode = tempLineNode;

	}
	// finally update what the previous
	root->lines = firstLineNode->next;
	if (previousLineNode == firstLineNode) {
		root->firstQuadtreeLineNode = NULL;
	} else {
		root->firstQuadtreeLineNode = previousLineNode;
	}

	if (root->nw != NULL) {
		updateNode(root->nw);
		updateNode(root->ne);
		updateNode(root->sw);
		updateNode(root->se);
	}

	free(firstLineNode);
}

void divideNode(Node *node){
	int numberOfLines = node->numberOfLines;
	if (numberOfLines < maxLines) {
		return;
	}

	LineNode * currentLineNode = node->lines;

	double xMin = node->xMin;
	double xMax = node->xMax;
	double yMin = node->yMin;
	double yMax = node->yMax;
	double xMid = (node->xMin + node->xMax) / 2.0;
	double yMid = (node->yMin + node->yMax) / 2.0;


	node->nw = create_node(xMin, xMid, yMid, yMax);
	node->ne = create_node(xMid, xMax, yMid, yMax);
	node->sw = create_node(xMin, xMid, yMin, yMid);
	node->se = create_node(xMid, xMax, yMin, yMid);

	node->nw->parent = node;
	node->ne->parent = node;
	node->sw->parent = node;
	node->se->parent = node;

	node->numberOfLines = 0;

	node->lines = NULL;
	node->firstQuadtreeLineNode = NULL;
//	LineNode * previousLineNode = NULL;

	while (currentLineNode != NULL) {
		Line * line = currentLineNode->line;
		LineNode * tempLineNode = currentLineNode->next;
		quadrant_t quadrant = getLineQuadrant(node, line);
		if (quadrant == NONE) {
			// newLines[numberOfNewLines] = line;
			reAddQuadtreeLineNode(node, currentLineNode);
			if (node->firstQuadtreeLineNode == NULL) {
				node->firstQuadtreeLineNode = currentLineNode;
			}
		} else {
			Node * desiredNode = NULL;
			switch(quadrant) {
				case NW:
					desiredNode = node->nw;
					addQuadtreeLineNode(desiredNode, currentLineNode);
					break;
				case NE:
					desiredNode = node->ne;
					addQuadtreeLineNode(desiredNode, currentLineNode);
					break;
				case SE:
					desiredNode = node->se;
					addQuadtreeLineNode(desiredNode, currentLineNode);
					break;
				case SW:
					desiredNode = node->sw;
					addQuadtreeLineNode(desiredNode, currentLineNode);
					break;
				default:
					break;
			}
		}
		currentLineNode = tempLineNode;
	}
//	node->firstQuadtreeLineNode = previousLineNode;
	// Line ** tempLineListPointer = node->enclosedLines;
	// node->enclosedLines = newLines;
	// free(tempLineListPointer);

	divideNode(node->nw);
	divideNode(node->ne);
	divideNode(node->se);
	divideNode(node->sw);
}

// This is to initialize any line node, with the next pointer passed in.
LineNode * createLineNode(LineNode * lineNode, Line * line) {
	LineNode * newLineNode = malloc(sizeof(LineNode));
	newLineNode->next = lineNode;
	newLineNode->line = line;
	return newLineNode;
}

void quadtreeFree(Node *root){
	if (! (root == NULL)) {
		freeNode(root);
	}
}


static inline void helperAddLineNode(Line * line, Line * nextLine, IntersectionEventListReducer * intersectionEventListReducer) {
	Line * firstLine, * secondLine;
	if (compareLines(line, nextLine) < 0) {
		firstLine = line;
		secondLine = nextLine;
	} else {
		firstLine = nextLine;
		secondLine = line;
	}
	IntersectionType intersectionType = intersect(firstLine, secondLine, globalTimeStep);
	if (intersectionType != NO_INTERSECTION) {
		IntersectionEventList_appendNode(&REDUCER_VIEW(*intersectionEventListReducer),
				firstLine, secondLine, intersectionType);
	}
}

// This adds line nodes to the list for intersection processing.
LineNode * addLineNode(Line * line, LineNode * lineNode,
		IntersectionEventListReducer * intersectionEventListReducer) {

	LineNode * newLineNode = createLineNode(lineNode, line);
	LineNode * nextLineNode = newLineNode->next;

	//traverse through the linked list, comparing the newly added line to each thing
	while (nextLineNode != NULL) {
		 helperAddLineNode(line, nextLineNode->line, intersectionEventListReducer);
		nextLineNode = nextLineNode->next;
	}
	;
	return newLineNode;
}

void testNewCollisionLineNode(LineNode * lineNode,
		IntersectionEventListReducer * intersectionEventListReducer) {
	LineNode * nextLineNode = lineNode->next;
	Line * line = lineNode->line;
	Line * nextLine;
	//traverse through the linked list, comparing the newly added line to each thing
	while (nextLineNode != NULL) {
		nextLine = nextLineNode->line;
		Line * firstLine, * secondLine;

	    if(line->maxXpointer->x < nextLine->minXpointer->x || line->minXpointer->x > nextLine->maxXpointer->x
			  || line->maxYpointer->y < nextLine->minYpointer->y || line->minYpointer->y > nextLine->maxYpointer->y){
	    	nextLineNode = nextLineNode->next;
	    	continue;
	    }
	    if (line->id < nextLine->id) {
			firstLine = line;
			secondLine = nextLine;
		} else {
			firstLine = nextLine;
			secondLine = line;
		}
		IntersectionType intersectionType = intersect(firstLine, secondLine, globalTimeStep);
		if (intersectionType != NO_INTERSECTION) {
			IntersectionEventList_appendNode(&REDUCER_VIEW(*intersectionEventListReducer),
					firstLine, secondLine, intersectionType);
		}
		nextLineNode = nextLineNode->next;
	}

	return;
}

//l_node points to the head of the linked list that contains the rest of the points
void traverseQuadtree(Node *node,
		IntersectionEventListReducer * intersectionEventListReducer,
		LineNode * lineNode){
	//iterate through each of the lines and attach each to the front of the linked list
		//by creating a linkedLineNode with the line and pointing it to the head
	if (node->numberOfLines == 0) {
//		return intersectionEventListReducer;
		if (node->nw != NULL) {
			cilk_spawn traverseQuadtree(node->nw, intersectionEventListReducer, lineNode);
			cilk_spawn traverseQuadtree(node->ne, intersectionEventListReducer, lineNode);
			cilk_spawn traverseQuadtree(node->sw, intersectionEventListReducer, lineNode);
			cilk_spawn traverseQuadtree(node->se, intersectionEventListReducer, lineNode);
			cilk_sync;
		}
		return;
	}
	node->firstQuadtreeLineNode->next = lineNode;

	if (node->nw != NULL) {
		cilk_spawn traverseQuadtree(node->nw, intersectionEventListReducer, node->lines);
		cilk_spawn traverseQuadtree(node->ne, intersectionEventListReducer, node->lines);
		cilk_spawn traverseQuadtree(node->sw, intersectionEventListReducer, node->lines);
		cilk_spawn traverseQuadtree(node->se, intersectionEventListReducer, node->lines);
	}

	LineNode * currentQuadtreeLineNode = node->lines;
	while (currentQuadtreeLineNode != lineNode) {

		testNewCollisionLineNode(currentQuadtreeLineNode, intersectionEventListReducer);
		currentQuadtreeLineNode = currentQuadtreeLineNode->next;
	}
	cilk_sync;
	node->firstQuadtreeLineNode->next = NULL;
}

int overlapsRight(Line *line) {
	if ((line->p1.x > BOX_XMAX || line->p2.x > BOX_XMAX)
	        && (line->velocity.x > 0)) {
	  line->velocity.x = -line->velocity.x;

	  updateLineFuturePoints(line);

	  return 1;
	}
	return 0;
}

int overlapsLeft(Line *line) {
	if ((line->p1.x < BOX_XMIN || line->p2.x < BOX_XMIN)
	        && (line->velocity.x < 0)) {
	  line->velocity.x = -line->velocity.x;

	  updateLineFuturePoints(line);

	  return 1;
	}
	return 0;
}

int overlapsTop(Line *line) {
	if ((line->p1.y > BOX_YMAX || line->p2.y > BOX_YMAX)
			&& (line->velocity.y > 0)) {
	  line->velocity.y = -line->velocity.y;

	  updateLineFuturePoints(line);

	  return 1;
	}
	return 0;
}

int overlapsBottom(Line *line) {
	if ((line->p1.y < BOX_YMIN || line->p2.y < BOX_YMIN)
							&& (line->velocity.y < 0)) {
	  line->velocity.y = -line->velocity.y;

	  updateLineFuturePoints(line);

	  return 1;
	}
	return 0;
}

/*0: north
1: east
2: south
3: west
*/
int traverseQuadtreeSide(Node *node, side_t side) {
	if (node == NULL) {return 0;}
	int count = 0;
	int count1 = 0;
	int count2 = 0;
	LineNode * currentQuadtreeLineNode = node->lines;
	switch (side) {
		case NORTH:
			while (currentQuadtreeLineNode != NULL) {
//			for (int i=0; i < node->numberOfLines; i++) {
				Line * line = currentQuadtreeLineNode->line;
				count += overlapsTop(line);
				currentQuadtreeLineNode = currentQuadtreeLineNode->next;
			}
			count1 = traverseQuadtreeSide(node->nw,NORTH);
			count2 = traverseQuadtreeSide(node->ne,NORTH);
			break;
		case EAST:
			while (currentQuadtreeLineNode != NULL) {
//			for (int i=0; i < node->numberOfLines; i++) {
				Line * line = currentQuadtreeLineNode->line;
				count += overlapsRight(line);
				currentQuadtreeLineNode = currentQuadtreeLineNode->next;
			}
			count1 = traverseQuadtreeSide(node->ne,EAST);
			count2 = traverseQuadtreeSide(node->se,EAST);
			break;
		case SOUTH:
			while (currentQuadtreeLineNode != NULL) {
//			for (int i=0; i < node->numberOfLines; i++) {
				Line * line = currentQuadtreeLineNode->line;
				count += overlapsBottom(line);
				currentQuadtreeLineNode = currentQuadtreeLineNode->next;
			}
			count1 = traverseQuadtreeSide(node->sw,SOUTH);
			count2 = traverseQuadtreeSide(node->se,SOUTH);
			break;
		case WEST:
			while (currentQuadtreeLineNode != NULL) {
//			for (int i=0; i < node->numberOfLines; i++) {
				Line * line = currentQuadtreeLineNode->line;
				count += overlapsLeft(line);
				currentQuadtreeLineNode = currentQuadtreeLineNode->next;
			}
			count1 = traverseQuadtreeSide(node->sw,WEST);
			count2 = traverseQuadtreeSide(node->nw,WEST);
			break;
	}
	return count + count1 + count2;
}

/*
 * 0: NW
 * 1: NE
 * 2: SE
 * 3: SW
 */
int traverseQuadtreeCorner(Node *node, quadrant_t corner) {
	if (node == NULL) {return 0;}
	int count = 0;
	int count1 = 0;
	int count2 = 0;
	int count3 = 0;
	LineNode * currentQuadtreeLineNode = node->lines;
	switch (corner) {
		case NW:
			while (currentQuadtreeLineNode != NULL) {
//			for (int i=0; i < node->numberOfLines; i++) {
				Line * line = currentQuadtreeLineNode->line;
				count += overlapsTop(line);
				count += overlapsLeft(line);
				currentQuadtreeLineNode = currentQuadtreeLineNode->next;
			}
			count1 = traverseQuadtreeCorner(node->nw,NW);
			count2 = traverseQuadtreeSide(node->ne,NORTH);
			count3 = traverseQuadtreeSide(node->sw,WEST);
			break;
		case NE:
			while (currentQuadtreeLineNode != NULL) {
//			for (int i=0; i < node->numberOfLines; i++) {
				Line * line = currentQuadtreeLineNode->line;
				count += overlapsTop(line);
				count += overlapsRight(line);
				currentQuadtreeLineNode = currentQuadtreeLineNode->next;
			}
			count1 = traverseQuadtreeCorner(node->ne,NE);
			count2 = traverseQuadtreeSide(node->nw,NORTH);
			count3 = traverseQuadtreeSide(node->se,EAST);
			break;
		case SE:
			while (currentQuadtreeLineNode != NULL) {
//			for (int i=0; i < node->numberOfLines; i++) {
				Line * line = currentQuadtreeLineNode->line;
				count += overlapsRight(line);
				count += overlapsBottom(line);
				currentQuadtreeLineNode = currentQuadtreeLineNode->next;
			}
			count1 = traverseQuadtreeCorner(node->se,SE);
			count2 = traverseQuadtreeSide(node->sw,SOUTH);
			count3 = traverseQuadtreeSide(node->ne,EAST);
			break;
		case SW:
			while (currentQuadtreeLineNode != NULL) {
//			for (int i=0; i < node->numberOfLines; i++) {
				Line * line = currentQuadtreeLineNode->line;
				count += overlapsBottom(line);
				count += overlapsLeft(line);
				currentQuadtreeLineNode = currentQuadtreeLineNode->next;
			}
			count1 = traverseQuadtreeCorner(node->sw,SW);
			count2 = traverseQuadtreeSide(node->se,SOUTH);
			count3 = traverseQuadtreeSide(node->nw,WEST);
			break;
		default:
			break;

	}
	return count + count1 + count2 + count3;
}

int getWallCollisions (Node * root) {
	int countRoot = 0;
	LineNode * currentQuadtreeLineNode = root->lines;
	while (currentQuadtreeLineNode != NULL) {
		Line * line = currentQuadtreeLineNode->line;
		countRoot += overlapsTop(line);
		countRoot += overlapsRight(line);
		countRoot += overlapsBottom(line);
		countRoot += overlapsLeft(line);
		currentQuadtreeLineNode = currentQuadtreeLineNode->next;
	}
	int count1 = traverseQuadtreeCorner(root->nw, NW);
	int count2 = traverseQuadtreeCorner(root->ne, NE);
	int count3 = traverseQuadtreeCorner(root->se, SE);
	int count4 = traverseQuadtreeCorner(root->sw, SW);
	return countRoot + count1 + count2 + count3 + count4;
	return countRoot;
}



