#include "myderivedclass.h"

MyDerivedClass::MyDerivedClass(const int a) : mynm::MyBaseClass(a) {}

void MyDerivedClass::GetState()
{
  printf("Current state : %d\n", (int) GetCurrentState());
}
