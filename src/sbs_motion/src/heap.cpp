#include "Node.hpp"
#include "heap.hpp"

heap::heap(int maxSize){
  nodes=new Node*[maxSize];
  max=maxSize;
  size=0;
}
heap::~heap(){
  delete[] nodes;
}

bool heap::empty(){return size==0;}

int heap::getSize(){return size;}

void heap::update(Node *node){
  sortUp(node);
}

bool heap::contains(Node * node){
  if(node->heapIdx>=0)
    return *node==*nodes[node->heapIdx];
  return false;
}

Node* heap::pop(){
  Node *item=nodes[0];
  size--;
  nodes[0]=nodes[size];
  nodes[0]->heapIdx=0;
  sortDown(nodes[0]);
  return item;
}

void heap::sortDown(Node* node){
  while(true){
    int left=2*node->heapIdx+1; 
    int right=2*node->heapIdx+2;
    int swapIdx=0;
    if(left<size){
      swapIdx=left;
      if(right<size){
	if(nodes[right]->compare(nodes[left]))
	  swapIdx=right;
      }
      if(nodes[swapIdx]->compare(node)){
	swap(node, nodes[swapIdx]);
      }else
	return;
    }else
      return;
  }
}

void heap::add(Node * node){
  nodes[size]=node;
  node->heapIdx=size;
  sortUp(node);
  size++;
}

void heap::sortUp(Node *node){
  while(true){
    int parentIndex=(node->heapIdx-1)/2;
    if(node->compare(nodes[parentIndex])){
      swap(node,nodes[parentIndex]);
    }
    else
      return;
  }
}

void heap::swap(Node *first, Node* second){
  nodes[first->heapIdx]=second;
  nodes[second->heapIdx]=first;
  int idxTmp=first->heapIdx;
  first->heapIdx=second->heapIdx;
  second->heapIdx=idxTmp;
}
