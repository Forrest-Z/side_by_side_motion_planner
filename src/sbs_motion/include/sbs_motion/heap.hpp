#ifndef HEAP_H
#define HEAP_H
class heap
{
public:
  heap(int maxSize);
  ~heap();
  void add(Node *node);
  Node* pop();
  bool contains(Node *node);
  void update(Node *node);
  bool empty();
  int getSize();
private:
  void sortUp(Node *size);
  void swap(Node *first,Node* second);
  void sortDown(Node* size);
  int max;
  int size;
  Node** nodes;
  bool simple;
};
#endif
