/*
 * node.c
 *
 *  Created on: Oct 3, 2015
 *      Author: alhuang
 */

#include "Quadtree.h"
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

Node * create_node(double x_min, double x_max, double y_min, double y_max){

	Node *node;
	node = malloc(sizeof(Node));

	node->nw = NULL;
	node->ne = NULL;
	node->sw = NULL;
	node->se = NULL;

	node->xMax = x_max;
	node->xMin = x_min;
	node->yMax = y_max;
	node->yMin = y_min;

	node->numberOfLines = 0;
	node->bufferLineCount = 0;
	node->firstQuadtreeLineNode= NULL;

	node->buffer = NULL;
	node->bufferEnd = NULL;
	node->lines = NULL;

	return node;
}

void addLine(Node* node, Line * line) {
	// node->enclosedLines[node->numberOfLines] = line;
	node->numberOfLines++;
	node->lines = createLineNode(node->lines, line);
	if (node->numberOfLines == 1) {
		node->firstQuadtreeLineNode = node->lines;
	}
}

void reAddQuadtreeLineNode(Node * node, LineNode * lineNode) {
	lineNode->next = node->lines;
	node->lines = lineNode;
	node->numberOfLines++;
}

void addToBuffer(Node * node, LineNode * lineNode) {
	if (node->buffer == NULL) {
		node->bufferEnd = lineNode;
	}
	lineNode->next = node->buffer;
	node->buffer = lineNode;
	node->bufferLineCount++;
}

void addQuadtreeLineNode(Node * node, LineNode * lineNode) {
	lineNode->next = node->lines;
	node->lines = lineNode;
	node->numberOfLines++;
	if (node->firstQuadtreeLineNode == NULL) {
		node->firstQuadtreeLineNode = lineNode;
	}
}

void freeQuadtreeLineNode(LineNode * lineNode) {
	if (lineNode != NULL) {
		freeQuadtreeLineNode(lineNode->next);
		free(lineNode);
	}
}

void freeNode(Node * node){
	if (!(node == NULL)) {
		freeNode(node->nw);
		freeNode(node->ne);
		freeNode(node->sw);
		freeNode(node->se);
		// free(node->enclosedLines);
		freeQuadtreeLineNode(node->lines);
		free(node);
	}
}
