#ifndef QTELEMENT_H
#define QTELEMENT_H

#include <iostream>
using namespace std;
class QTElement : public Quadnode
{

public:
  QTElement() 
    {
        NumSons = 0;
        Valid = true;
      
        Sons[0] = false;
        Sons[1] = false;
        Sons[2] = false;
        Sons[3] = false;
    }
    void unvalidate(void)
    {
        Valid = false;
    }

 void validate()
    {
        Valid = true;
    }

    bool isValid()
    {
        return Valid;
    }

    void print()
    {
      cout << " sons=" << NumSons << " " << Sons[0] << "/" << Sons[1] << "/" << Sons[2] << "/" << Sons[3] << "\n";
    }

public:
  bool Sons[4];
  int NumSons;
private:
  bool Valid;
};
#endif
