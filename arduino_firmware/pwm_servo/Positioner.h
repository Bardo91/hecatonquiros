//
// Positioner end-tool code
//
//
//
//

#include "Matrix.h"

class Positioner{
public:
  Positioner(const int _pin0, const int _pin1, const int _pin2, const int _pin3, const int _pin4);

  void baseToHand (float &_x, float &_y, float &_z);
  void handToBase (float &_x, float &_y, float &_z);
  void rawJoints  (float &_j0, float &_j1, float &_j2, float &_j3, float &_j4);
  void angles     (float &_t0, float &_t1, float &_t2, float &_t3, float &_t4);
private:
  float analogToDeg(float _analog);

  
private:
  int mPinJoint0;
  int mPinJoint1;
  int mPinJoint2;
  int mPinJoint3;
  int mPinJoint4;

  const float cL01 = 68.0;
  const float cL12 = 169.0;
  const float cL23 = 169.0;
  const float cL34 = 112.0;
  const float cL4f = 70.0;
};

