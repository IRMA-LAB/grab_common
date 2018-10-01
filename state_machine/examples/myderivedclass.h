#ifndef MYDERIVEDCLASS_H
#define MYDERIVEDCLASS_H

#include "mybaseclass.h"

class MyDerivedClass : public mynm::MyBaseClass
{
public:
  MyDerivedClass(const int a);
  ~MyDerivedClass() {}

  void GetState();
};

#endif // MYDERIVEDCLASS_H
