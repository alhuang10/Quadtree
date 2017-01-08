/**
 * Copyright (c) 2013 the Massachusetts Institute of Technology
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

#include "./IntersectionEventList.h"

#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

int IntersectionEventNode_compareData(IntersectionEventNode* node1,
                                      IntersectionEventNode* node2) {
  int firstComparison = compareLines(node1->l1, node2->l1);
  if (firstComparison == 0) {
	  return compareLines(node1->l2,node2->l2);
  }
  return firstComparison;
}

void IntersectionEventNode_swapData(IntersectionEventNode* node1,
                                    IntersectionEventNode* node2) {
  {
    Line* temp = node1->l1;
    node1->l1 = node2->l1;
    node2->l1 = temp;
  }
  {
    Line* temp = node1->l2;
    node1->l2 = node2->l2;
    node2->l2 = temp;
  }
  {
    IntersectionType temp = node1->intersectionType;
    node1->intersectionType = node2->intersectionType;
    node2->intersectionType = temp;
  }
}

IntersectionEventList IntersectionEventList_make() {
  IntersectionEventList intersectionEventList;
  intersectionEventList.head = NULL;
  intersectionEventList.tail = NULL;
  return intersectionEventList;
}
void IntersectionEventList_appendNode(
    IntersectionEventList* intersectionEventList, Line* l1, Line* l2,
    IntersectionType intersectionType) {
  assert(compareLines(l1, l2) < 0);
  IntersectionEventNode* newNode = malloc(sizeof(IntersectionEventNode));
  if (newNode == NULL) {
    return;
  }

  newNode->l1 = l1;
  newNode->l2 = l2;
  newNode->intersectionType = intersectionType;
  newNode->next = NULL;
  if (intersectionEventList->head == NULL) {
    intersectionEventList->head = newNode;
  } else {
    intersectionEventList->tail->next = newNode;
  }
  intersectionEventList->tail = newNode;
}

void IntersectionEventList_deleteNodes(
    IntersectionEventList* intersectionEventList) {
  IntersectionEventNode* curNode = intersectionEventList->head;
  IntersectionEventNode* nextNode = NULL;
  while (curNode != NULL) {
    nextNode = curNode->next;
    free(curNode);
    curNode = nextNode;
  }
  intersectionEventList->head = NULL;
  intersectionEventList->tail = NULL;
}

void IntersectionEventList_reduce (IntersectionEventList* key, IntersectionEventList* left, IntersectionEventList* right){
	if (left->tail == NULL) {
		left->head = right->head;
		left->tail = right->tail;
		right->head = NULL;
		right->tail = NULL;
		return;
	}
	if (right->tail == NULL) {
		return;
	}
	left->tail->next = right->head;
	left->tail = right->tail;
	right->head = NULL;
	right->tail = NULL;
}

void IntersectionEventList_identity(IntersectionEventList* key, IntersectionEventList* value){
	value->head = NULL;
	value->tail = NULL;
}
void IntersectionEventList_destroy(IntersectionEventList* key, IntersectionEventList* value){
	IntersectionEventList_deleteNodes(value);
}

