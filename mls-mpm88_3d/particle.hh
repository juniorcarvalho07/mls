#ifndef PARTICLE_HH
#define PARTICLE_HH
#include"taichi.h"
#include "ColorRGBA.hpp"
#include "Cores.h"

using Vec = taichi::Vector3;
using Mat = taichi::Matrix3;


struct Particle {
      taichi::Vector3 x; // position
      taichi::Vector3 v; // velocity
      taichi::Matrix3 C; // affine momentum matrix, unused in this file
      // Deformation gradient
      taichi::Matrix3 F;
      // Determinant of the deformation gradient (i.e. volume)
      taichi::real Jp;
      taichi::real mass;
      taichi::real radius;
      int type;  // 2: elastic   1: plastic   0: liquid -1 blood
      TColorRGBA color;

      Particle(int t=0,taichi::real r=0,Vec x =Vec(0), Vec v=Vec(0)):
      x(x),
      v(v),
        F(1),
        C(0),
        type(t),
        radius(r),
        Jp(1){
          switch(type){
          case -1:
              color = bloodred;
              break;
          case 0:
              color =waterblue1;
              break;
          case 1:
              color =yellow;
              break;
          case 2:
              color =red;
              break;

          }
      }
  };


#endif // PARTICLE_HH
