//
// Positioner end-tool code
//
//
//
//

#include "Matrix.h"

Matrix4f::Matrix4f(float *_data){
  for(unsigned i = 0; i < 4; i++){
    for(unsigned j = 0; j < 4; j++){
      mData[i*4+j] = _data[i*4+j];
    }
  }
}

Matrix4f Matrix4f::operator-(Matrix4f &_other){
  Matrix4f result;
  auto data = result.data();

  for(unsigned i = 0; i < 4; i++){
    for(unsigned j = 0; j < 4; j++){
      data[i*4+j] = mData[i*4+j] - _other.mData[i*4+j];
    }
  }
  
  return result;
}


Matrix4f Matrix4f::operator+(Matrix4f &_other){
  Matrix4f result;
  auto data = result.data();

  for(unsigned i = 0; i < 4; i++){
    for(unsigned j = 0; j < 4; j++){
      data[i*4+j] = mData[i*4+j] + _other.mData[i*4+j];
    }
  }
  return result;
}

Matrix4f Matrix4f::operator*(Matrix4f &_other){
  Matrix4f result;
  auto data = result.data();

  for(unsigned i = 0; i < 4; i++){
    for(unsigned j = 0; j < 4; j++){
      for(unsigned k = 0; k < 4; k++){
        data[i*4+j] += mData[i*4+k]*_other.mData[k*4+j];
      }
    }
  }
  
  return result;
}


Matrix4f Matrix4f::operator*(float _scale){
  Matrix4f result;
  auto data = result.data();

  for(unsigned i = 0; i < 4; i++){
    for(unsigned j = 0; j < 4; j++){
      data[i*4+j] = mData[i*4+j]*_scale;
    }
  }
  
  return result;
}

float * Matrix4f::data(){
  return mData;
}


String Matrix4f::serialize(){
  String output = "[";
  for(unsigned i = 0; i < 4; i++){
    for(unsigned j = 0; j < 4; j++){
      output += String(mData[i*4+j]);
      if(j < 3)
        output += ", \t";
      else
        output += ";";
    }
    if(i < 3)
      output += "\n";
  }
  output += "]";
  return output;
}


float &Matrix4f::at(const int _i, const int _j){
  return mData[_i*4+_j];
}


float &Matrix4f::operator()(const int _i, const int _j){
  return mData[_i*4+_j];
}

Matrix4f Matrix4f::Identity(){
  float data[] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
  return Matrix4f(data);
}

