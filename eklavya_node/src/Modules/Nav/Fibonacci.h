

/*	Fibonacci Heap Data Structure Implementation
 *	for efficiency enhancement in modified A* algorithm
 *	for robot path planning and navigation.
 */

#ifndef FIBONACCI
#define FIBONACCI

#include <stdio.h>
#include <math.h>

#define PHI 1.61803399

class FibonacciHeapMinTreeNode
{
public:
	FibonacciHeapMinTreeNode();
	FibonacciHeapMinTreeNode(int initialCost);
	FibonacciHeapMinTreeNode *next,*prev,*child,*rootContainingMinimumKey;
	int noOfChildren;
	bool isMarked;
	int cost;
	void print();
};

class FibonacciHeap
{
public:
	FibonacciHeapMinTreeNode *min;
	int noOfNodesInTheHeap;
	FibonacciHeap();
	FibonacciHeap(FibonacciHeapMinTreeNode *root);
	void merge(FibonacciHeap* other);
	void insert(FibonacciHeapMinTreeNode *newNode);
	FibonacciHeapMinTreeNode *findMin();
	void deleteMin();
	void consolidate();
	void fibonacciHeapLink(FibonacciHeapMinTreeNode *x,FibonacciHeapMinTreeNode *y);
	void print();
private:
	int D();
};

#endif