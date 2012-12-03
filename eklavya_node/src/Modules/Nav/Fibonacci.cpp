
#include <stdio.h>
#include <stdlib.h>
#include "Fibonacci.h"
#define INFINITY 32767

FibonacciHeapMinTreeNode::FibonacciHeapMinTreeNode()
{
	this->next=this;
	this->prev=this;
	this->child=NULL;
	this->isMarked=false;
	this->noOfChildren=0;
	this->rootContainingMinimumKey=NULL;
	this->cost=INFINITY;
}

FibonacciHeapMinTreeNode::FibonacciHeapMinTreeNode(int initialCost)
{
	this->next=this;
	this->prev=this;
	this->child=NULL;
	this->isMarked=false;
	this->noOfChildren=0;
	this->rootContainingMinimumKey=NULL;
	this->cost=initialCost;
}

FibonacciHeap::FibonacciHeap()
{
	this->noOfNodesInTheHeap=0;
	this->min=NULL;
}

FibonacciHeap::FibonacciHeap(FibonacciHeapMinTreeNode *root)
{
	this->min=root;
	this->noOfNodesInTheHeap=1;
}

void FibonacciHeap::insert(FibonacciHeapMinTreeNode *newNode)
{
	FibonacciHeap newHeap;
	newHeap.min=newNode;
	newHeap.noOfNodesInTheHeap=1;
	merge(&newHeap);
}

FibonacciHeapMinTreeNode *FibonacciHeap::findMin()
{
	return this->min;
}

void FibonacciHeap::deleteMin()
{
	FibonacciHeapMinTreeNode *z=this->min,*current;
	if(z!=NULL)
	{
		if(z->noOfChildren)
		{
			z->child->prev->next=z->next;
			z->next->prev=z->child->prev;
			z->child->prev=z;
			z->next=z->child;
			current=z->child;
			while(z->noOfChildren--)
			{
				current->rootContainingMinimumKey=NULL;
				current=current->next;
			}
			z->child=NULL;
		}

		if(z==z->next)
		{
			this->min=NULL;
		}
		else
		{
			this->min=z->next;
			consolidate();
		}
		this->noOfNodesInTheHeap--;
	}
}

void FibonacciHeap::consolidate()
{
	FibonacciHeapMinTreeNode *A[30],*x,*y,*temp,*last;
	int d;
	for(int i=0;i<30;i++)
	{
		A[i]=NULL;
	}
	x=this->min;
	last=this->min->prev;
	do
	{
		d=x->noOfChildren;
		while(A[d]!=NULL)
		{
			y=A[d];
			if(x->cost>y->cost)
			{
				temp=x;
				x=y;
				y=temp;
			}
			this->fibonacciHeapLink(y,x);
			A[d++]=NULL;
		}
		A[d]=x;
		x=x->next;
	}
	while(x->prev!=last);
	this->min=NULL;
	for(int i=0;i<D();i++)
	{
		if(A[i]!=NULL)
		{
			if(this->min==NULL)
			{
				this->min=A[i];
			}
			else
			{
				if(A[i]->cost<this->min->cost)
				{
					this->min=A[i];
				}
			}
		}
	}
}

int FibonacciHeap::D()
{
	return int(log((double)this->noOfNodesInTheHeap)/log(PHI));
}

void FibonacciHeap::fibonacciHeapLink(FibonacciHeapMinTreeNode *x,FibonacciHeapMinTreeNode *y)
{
	x->next->prev=x->prev;
	x->prev->next=x->next;
	if(y->child==NULL)
	{
		y->child=x;
		x->next=x;
		x->prev=x;
	}
	else
	{
		x->next=y->child->next;
		x->prev=y->child;
		y->child->next->prev=x;
		y->child->next=x;
	}
	y->noOfChildren++;
	x->isMarked=false;
}

/**
 *	Both Nodes should be root nodes.
 */
void FibonacciHeap::merge(FibonacciHeap *other)
{
	this->min->next->prev=other->min->prev;
	other->min->prev->next=this->min->next;
	this->min->next=other->min;
	other->min->prev=this->min;
	this->noOfNodesInTheHeap=this->noOfNodesInTheHeap+other->noOfNodesInTheHeap;
	if(this->min->cost>other->min->cost)
	{
		this->min=other->min;
	}
}

void FibonacciHeapMinTreeNode::print()
{
	printf("%d ",this->cost);
	if(this->child!=NULL)
	{
		FibonacciHeapMinTreeNode *current=this->child;
		printf("(");
		for(int i=0;i<this->noOfChildren;i++)
		{
			current->print();
			current=current->next;
		}
		printf(" )");
	}
}

void FibonacciHeap::print()
{
	FibonacciHeapMinTreeNode *current=this->min;
	do
	{
		current->print();
		current=current->next;
	}
	while(current!=this->min);
}