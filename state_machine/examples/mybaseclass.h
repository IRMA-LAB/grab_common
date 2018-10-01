#ifndef MYBASECLASS_H
#define MYBASECLASS_H

#include "StateMachine.h"

namespace mynm {

class MyPureVirtualClass
{
public:
  MyPureVirtualClass() {}
  virtual ~MyPureVirtualClass() = 0;

private:
  virtual void Check() = 0;
  virtual void Dummy();
};

enum MyBaseClassStates: BYTE
{
  ST_IDLE,
  ST_WORK,
  ST_MAX_STATES
};

class MyBaseClass: public StateMachine, virtual public MyPureVirtualClass
{
public:
  MyBaseClass(const int a);
  ~MyBaseClass() {}

  void MyExtEvent();

  void Check() override final {}

protected:
  void Dummy() override final;

private:
  int a_;

  STATE_DECLARE(MyBaseClass, Idle, NoEventData)
  STATE_DECLARE(MyBaseClass, Work, NoEventData)

  BEGIN_STATE_MAP
  STATE_MAP_ENTRY({&Idle})
  STATE_MAP_ENTRY({&Work})
  END_STATE_MAP
};

} // end namespace

#endif // MYBASECLASS_H
