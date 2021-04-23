#include <iostream>
#include <string>

#include "easycatmaster.h"

using namespace std;

int main()
{
  EasycatMaster master(1000000);

  master.preliminaryOperations();

  master.Start();
  sleep(2);

  cout << "Q to quit (case insensitive).. ";
  char key;
  cin >> key;
  while (key != 'q' && key != 'Q')
  {
    switch (key)
      {
      case 'e':
      case 'E':
        master.extAsyncCallInitiateEnableProcedure();
        break;
      case 'd':
      case 'D':
        master.extAsyncCallDisplayAll();
        break;
      default:
        cout << "Pressed key: " << key << ". Press Q to quit (case insensitive).. ";
        break;
      }
    cin >> key;
  }
  cout << "exiting.." << endl;
  return 0;
}
