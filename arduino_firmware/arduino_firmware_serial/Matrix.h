//
// Positioner end-tool code
//
//
//
//

#include <Arduino.h>

class Matrix4f{
public:
  // Constructors
  Matrix4f(){ memset(mData, 0, 16); };
  Matrix4f(float *_data);
  
  static Matrix4f Identity();

  // Operatos
  Matrix4f operator-(Matrix4f &_other);
  Matrix4f operator+(Matrix4f &_other);
  Matrix4f operator*(Matrix4f &_other);
  Matrix4f operator*(float _scale);

  // Access
  float* data();
  float &at(const int _i, const int _j);
  float &operator()(const int _i, const int _j);

  // misc
  String serialize();

private:
  float  mData[16];
  
  const int cRows = 4;
  const int cCols = 4;
};
