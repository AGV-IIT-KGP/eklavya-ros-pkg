#ifndef _MINHEAP_H
#define _MINHEAP_H

#include <iostream>
#include <vector>
#include <iterator>
using namespace std;

template <class T>
class MinHeap 
{
public:
    MinHeap();
    ~MinHeap();
    void insert(T element);
    T getMinElem() { return minHeap.front();}
    T deletemin();
    void print();
    int size() { return minHeap.size(); }
private:
    int left(int parent);
    int right(int parent);
    int parent(int child);
    void minHeapifyup(int index);
    void minHeapifydown(int index);
private:
    vector<T> minHeap;
};
template <class T>
MinHeap<T>::MinHeap()
{
}
template <class T>
MinHeap<T>::~MinHeap()
{
}
template <class T>
void MinHeap<T>::insert(T element)
{
    minHeap.push_back(element);
    minHeapifyup(minHeap.size() - 1);
}
template <class T>
T MinHeap<T>::deletemin()
{
    T min = minHeap.front();
    minHeap[0] = minHeap.at(minHeap.size() - 1);
    minHeap.pop_back();
    minHeapifydown(0);
    return min;
}
template <class T>
void MinHeap<T>::print()
{
    class vector<T>::iterator pos = minHeap.begin();
    cout << "MinHeap = ";
    while ( pos != minHeap.end() ) {
        cout << *pos << " ";
        ++pos;
    }
    cout << endl;
}
template <class T>
void MinHeap<T>::minHeapifyup(int index)
{    
    //cout << "index=" << index << endl;
    //cout << "parent(index)=" << parent(index) << endl;
    //cout << "minHeap[parent(index)]=" << minHeap[parent(index)] << endl;
    //cout << "minHeap[index]=" << minHeap[index] << endl;
    while ( ( index > 0 ) && ( parent(index) >= 0 ) &&
            ( minHeap[parent(index)] > minHeap[index] ) )
    {
        T tmp = minHeap[parent(index)];
        minHeap[parent(index)] = minHeap[index];
        minHeap[index] = tmp;
        index = parent(index);
    }
}
template <class T>
void MinHeap<T>::minHeapifydown(int index)
{     
    //cout << "index=" << index << endl;
    //cout << "left(index)=" << left(index) << endl;
    //cout << "right(index)=" << right(index) << endl;
    int child = left(index);
    if ( ( child > 0 ) && ( right(index) > 0 ) &&
         ( minHeap[child] > minHeap[right(index)] ) )
    {
        child = right(index);
    }
    if ( child > 0 )
    {
        T tmp = minHeap[index];
        minHeap[index] = minHeap[child];
        minHeap[child] = tmp;
        minHeapifydown(child);
    }
}
template <class T>
int MinHeap<T>::left(int parent)
{
    int i = ( parent << 1 ) + 1; // 2 * parent + 1
    return ( i < minHeap.size() ) ? i : -1;
}
template <class T>
int MinHeap<T>::right(int parent)
{
    int i = ( parent << 1 ) + 2; // 2 * parent + 2
    return ( i < minHeap.size() ) ? i : -1;
}
template <class T>
int MinHeap<T>::parent(int child)
{
    if (child != 0)
    {
        int i = (child - 1) >> 1;
        return i;
    }
    return -1;
}
namespace Nav
{
	struct PointXY
	{
		int x, y;
		double cost;
		PointXY():x(0), y(0), cost(0) {}
		PointXY(int x, int y, double cost=0):x(x), y(y), cost(cost) {}
		void print()
		{
		  printf("(%d, %d, %lf)\n", x, y, cost);
		}
		friend bool operator>(PointXY &p1, PointXY &p2);
		friend bool operator<(PointXY &p1, PointXY &p2);
	};
	bool operator>(PointXY &p1, PointXY &p2)
	{
		return (p1.cost > p2.cost)?1:0;
	}
	bool operator<(PointXY &p1, PointXY &p2)
	{
		return (p1.cost > p2.cost)?0:1;
	}
}
#endif
