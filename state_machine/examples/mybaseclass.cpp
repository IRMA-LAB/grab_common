#include "mybaseclass.h"

namespace mynm
{

MyPureVirtualClass::~MyPureVirtualClass() {}

void MyPureVirtualClass::Dummy() {}

MyBaseClass::MyBaseClass(const int a) : StateMachine(ST_MAX_STATES) { a_ = a; }

void MyBaseClass::MyExtEvent()
{
  BEGIN_TRANSITION_MAP
  TRANSITION_MAP_ENTRY(ST_WORK)
  TRANSITION_MAP_ENTRY(ST_IDLE)
  END_TRANSITION_MAP(NULL)
}

STATE_DEFINE(MyBaseClass, Idle, NoEventData) { printf("I'm in IDLE mode.\n"); }

STATE_DEFINE(MyBaseClass, Work, NoEventData) { printf("I'm in WORK mode.\n"); }

void MyBaseClass::Dummy() {}

} // end namespace
