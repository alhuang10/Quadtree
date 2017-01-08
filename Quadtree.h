/*
 * Quadtree.h
 *
 *  Created on: Oct 3, 2015
 *      Author: alhuang
 */


#ifndef QUADTREE_H_
#define QUADTREE_H_

#include "./Line.h"
#include "./CollisionWorld.h"
#include "./IntersectionEventList.h"

struct quadtree_node * globalQuadtree;

struct quadtree_node{

	struct quadtree_node *nw;
	struct quadtree_node *ne;
	struct quadtree_node *sw;
	struct quadtree_node *se;

	double xMax;
	double xMin;
	double yMax;
	double yMin;

	struct quadtree_node *parent;
	struct LinkedLineNode * buffer;
	struct LinkedLineNode * bufferEnd;
	int bufferLineCount;

	// (enclosedLines)
	// this is the line that points to null
	struct LinkedLineNode * firstQuadtreeLineNode;
	//this is the line that points to the list of all other lines
	struct LinkedLineNode * lines;//pointer to the last LinkedLineNode
	int numberOfLines;

} quadtree_node_t;
typedef struct quadtree_node Node;



struct LinkedLineNode {
	struct LinkedLineNode *next;
	Line * line;
} LinkedLineNode;
typedef struct LinkedLineNode LineNode;


typedef enum{NW, NE, SE, SW, NONE} quadrant_t;
typedef enum{NORTH, EAST, SOUTH, WEST} side_t;


Node * create_node(double x_min, double x_max, double y_min, double y_max);
void addLine(Node* node, Line * line);
void freeNode(Node * node);
LineNode * createLineNode(LineNode * lineNode, Line * line);

Node * instantiateRoot(CollisionWorld * collisionWorld);
void traverseQuadtree(Node *node,
		IntersectionEventListReducer * intersectionEventListReducer,
		LineNode * lineNode);
int getWallCollisions (Node * root);

void addQuadtreeLineNode(Node * node, LineNode * lineNode);
void freeQuadtreeLineNode(LineNode * lineNode);
void reAddQuadtreeLineNode(Node * node, LineNode * lineNode);
void insertLineNodeUpwardDuringUpdate(Node * node, LineNode * lineNode);
void insertLineNodeDownwardDuringUpdate(Node * node, LineNode * lineNode);
void updateNode(Node * root);
void attachBuffers(Node * node);
void addToBuffer(Node * node, LineNode * lineNode);
LineNode * addLineNode(Line * line, LineNode * lineNode,
		IntersectionEventListReducer * intersectionEventListReducer);
void testNewCollisionLineNode(LineNode * lineNode,
		IntersectionEventListReducer * intersectionEventListReducer);

#endif /* QUADTREE_H_ */
