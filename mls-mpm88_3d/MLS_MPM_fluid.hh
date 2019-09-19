#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "mat3d.h"
//#include "taichi.h"
#include "polarDecomposition.h"

#include "quaternion.h"
#include "visualizer.hh"
#include "ply.h"
#include "of.h"
#include "ofPlane.h"
#include "ofOffPointsReader.h"
#include "ofOffReader.h"
#include "ofOffWriter.h"
#include "ofList.h"
#include "VisOf/Utils/Handler.hpp"
#include "Point.hpp"
#include "printof.hpp"
#include "GL_Interactor.h"

#include "VisOf/iterFunc/CommandComponent.hpp"
#include "VisOf/iterFunc/MyCommands.hpp"

#include "ofVertexStarIteratorSurfaceVertex.h"
#define screenW 800
#define screenH 600
#define PI 3.14159265359
#define animTreshold 0.0333333333333
//#include "GL_Interactor.h"


//Define a malha a ser usada.
typedef of::MyofDefault2D TTraits;
typedef of::ofDefault2D TTraitsSSMesh;
typedef of::ofDefault2D::sVertex TofVertex;
typedef of::ofMesh<TTraits> TMesh;
typedef of::ofMesh<TTraitsSSMesh> TMeshSSM;
typedef of::ofPlane<TTraits> TPlane;
typedef of::ofOffReader<TTraits> TReader;
typedef of::ofOffWriter<TTraits> TWriter;
typedef of::ofRuppert2D<TTraitsSSMesh> TruppertSSM;
typedef Quaternion<real> TQuaternion;
TMesh *malha;
TMesh *MalhaObst;
TReader Reader;
TWriter Writer;
Handler<TMesh> meshHandler;
Handler<TMesh> meshHandlerObst;
int Vymax;
int Nint;

typedef PrintOf<TTraits> TPrintOf;

TPrintOf *Print;
TPrintOf *PrintObst;

typedef MyCommands<TPrintOf> TMyCommands;
typedef CommandComponent TAllCommands;
typedef std::vector<int> vecInt;
std::vector<real> Mtime;
std::vector<std::size_t> MMemory;


TMyCommands *allCommands;

//##################################################################//





using namespace taichi;
using Vec = Vector3;
using Mat = Matrix3;

const int n = 100 /*grid resolution (cells)*/, window_size = 800;
const taichi::real dt = 50e-4_f / n , frame_dt = 1e-3_f, dx = 2.0_f / n,
           inv_dx = 1.0_f / dx;
auto particle_mass = 1.0_f, vol = 1.0_f;
auto hardening = 2.5_f;
taichi::real E = 1e4_f; // Young's modulus
taichi::real nu = 0.2_f; // Poisson's ratio

taichi::real mu_0 = E / (2 * (1 + nu));// Shear modulus (or Dynamic viscosity in fluids)
taichi::real  lambda_0 = E * nu / ((1 + nu) * (1 - 2 * nu)); // Lamé's 1st parameter \lambda=K-(2/3)\mu, where K is the Bulk modulus
const bool plastic = true;
//Grid Resolution

Vector4 grid[n + 1][n + 1][n+1];
std::vector<unsigned int> gridObst[n + 1][n + 1][n+1];

inline real rand01()
{
  return real(std::rand())/RAND_MAX;
}

inline real sphere_function(Vec center, Vec P, real r)
{
  return (P.x-center.x)*(P.x-center.x)+(P.y-center.y)*(P.y-center.y)+(P.z-center.z)*(P.z-center.z)-r*r;
};

inline Vec randomDirection()
{
  real alpha = 2 * M_PI * rand01();
  real beta = 2 * M_PI * rand01();
  
  return Vec(cos(alpha)*cos(beta), sin(alpha)*cos(beta),sin(beta));
}


    struct Parameters {
  // time step
  real dt;


  
   // environment

  // restitution
  real a;
  real mass;

   real gravity;
  // fluid parameters
   real rest_density ;
   real dynamic_viscosity;
  // equation of state
  real eos_stiffness;
  real eos_power;

};




class MLS_MPM
{
public:
// creation parameters
  real jitter;
  real spacing;
std::string screenFilename;
int winWidth, winHeight ;
int screenFilenameNumber;
int num_cells;
Visualizer Vis;
Parameters params;
real Acc_time;
real Acc_total_time;
real NormMaxA, NormMinA, NormMaxV,NormMinV,pressureMin,pressureMax,Wmax,Wmin;

	MLS_MPM(){

	winWidth =800; winHeight =600;
	screenFilename = "screenVis";
	
// environment



    params.a = 0.25;
        
    // time step
    params.dt = 0.0025;


    params.gravity = 9.8f;
    // fluid parameters
    params.rest_density = 1.0f;
    params.dynamic_viscosity = 0.002f;
    // equation of state
    params.eos_stiffness = 1000.0f;
    params.eos_power = 1;
    params.mass=1.25;
    num_cells = n * n* n;
    NormMaxA=-10000.0; NormMinA=10000.0; NormMaxV=-100000.0;NormMinV=10000.0;
    pressureMin=10000.0;pressureMax=-10000.0,Wmax=1.5,Wmin=0.0;
    v_particles.clear();
	};
	~MLS_MPM(){};

    std::vector<std::vector<Particle>> v_particles;



    inline void FillGridObst(TMesh *M)
    {
        if(M!=NULL)
        {
            BoundingBox bb;
            of::ofCellsIterator<TTraits> ic(M);
                for(ic.initialize(); ic.notFinish(); ++ic)
                {
                    Vector p1(M->getVertex(M->getCell(&ic)->getVertexId(0))->getCoord(0),M->getVertex(M->getCell(&ic)->getVertexId(0))->getCoord(1),M->getVertex(M->getCell(&ic)->getVertexId(0))->getCoord(2));
                    Vector p2(M->getVertex(M->getCell(&ic)->getVertexId(1))->getCoord(0),M->getVertex(M->getCell(&ic)->getVertexId(1))->getCoord(1),M->getVertex(M->getCell(&ic)->getVertexId(1))->getCoord(2));
                    Vector p3(M->getVertex(M->getCell(&ic)->getVertexId(2))->getCoord(0),M->getVertex(M->getCell(&ic)->getVertexId(2))->getCoord(1),M->getVertex(M->getCell(&ic)->getVertexId(2))->getCoord(2));
                    bb.lower_boundary.x=p1.x; bb.lower_boundary.y=p1.y;bb.lower_boundary.z=p1.z;
                    bb.upper_boundary.x=p1.x;bb.upper_boundary.y=p1.y;bb.upper_boundary.z=p1.z;
                    if(p2.x<bb.lower_boundary.x) bb.lower_boundary.x = p2.x;
                    if(p2.y<bb.lower_boundary.y) bb.lower_boundary.y = p2.y;
                    if(p2.z<bb.lower_boundary.z) bb.lower_boundary.z = p2.z;
                    if(p2.x>bb.upper_boundary.x) bb.upper_boundary.x = p2.x;
                    if(p2.y>bb.upper_boundary.y) bb.upper_boundary.y = p2.y;
                    if(p2.z>bb.upper_boundary.z) bb.upper_boundary.z = p2.z;
                    if(p3.x<bb.lower_boundary.x) bb.lower_boundary.x = p3.x;
                    if(p3.y<bb.lower_boundary.y) bb.lower_boundary.y = p3.y;
                    if(p3.z<bb.lower_boundary.z) bb.lower_boundary.z = p3.z;
                    if(p3.x>bb.upper_boundary.x) bb.upper_boundary.x = p3.x;
                    if(p3.y>bb.upper_boundary.y) bb.upper_boundary.y = p3.y;
                    if(p3.z>bb.upper_boundary.z) bb.upper_boundary.z = p3.z;
                    Vector3i base_coord_lower =
                        (bb.lower_boundary * inv_dx ).cast<int>();  // element-wise floor
                    Vector3i base_coord_upper =
                        (bb.upper_boundary * inv_dx ).cast<int>();  // element-wise floor
                    int i,j,k;
                    for(i=base_coord_lower.x;i<=base_coord_upper.x;i++)
                        for(j=base_coord_lower.y;j<=base_coord_upper.y;j++)
                            for(k=base_coord_lower.z;k<=base_coord_upper.z;k++)
                                gridObst[i][j][k].push_back(&ic);

                }

        }
    }

    inline void ComputeTriangleMeshNormal(TMesh *M,int Cellid,int dir =1)
    {
      Vector p1(M->getVertex(M->getCell(Cellid)->getVertexId(0))->getCoord(0),M->getVertex(M->getCell(Cellid)->getVertexId(0))->getCoord(1),M->getVertex(M->getCell(Cellid)->getVertexId(0))->getCoord(2));
      Vector p2(M->getVertex(M->getCell(Cellid)->getVertexId(1))->getCoord(0),M->getVertex(M->getCell(Cellid)->getVertexId(1))->getCoord(1),M->getVertex(M->getCell(Cellid)->getVertexId(1))->getCoord(2));
      Vector p3(M->getVertex(M->getCell(Cellid)->getVertexId(2))->getCoord(0),M->getVertex(M->getCell(Cellid)->getVertexId(2))->getCoord(1),M->getVertex(M->getCell(Cellid)->getVertexId(2))->getCoord(2));
      Vector vec21(p2.x-p1.x,p2.y-p1.y,p2.z-p1.z);
      Vector vec31(p3.x-p1.x,p3.y-p1.y,p3.z-p1.z);
      Vector nt = vec21^vec31;
      nt.normalize();
      M->getCell(Cellid)->setNormal(0,dir*nt.x);
      M->getCell(Cellid)->setNormal(1,dir*nt.y);
      M->getCell(Cellid)->setNormal(2,dir*nt.z);
      M->getVertex(M->getCell(Cellid)->getVertexId(0))->setNormalCoord(0,M->getCell(Cellid)->getNormalCoord(0));
      M->getVertex(M->getCell(Cellid)->getVertexId(0))->setNormalCoord(1,M->getCell(Cellid)->getNormalCoord(1));
      M->getVertex(M->getCell(Cellid)->getVertexId(0))->setNormalCoord(2,M->getCell(Cellid)->getNormalCoord(2));
      M->getVertex(M->getCell(Cellid)->getVertexId(0))->incNumberOfCells();
      M->getVertex(M->getCell(Cellid)->getVertexId(1))->setNormalCoord(0,M->getCell(Cellid)->getNormalCoord(0));
      M->getVertex(M->getCell(Cellid)->getVertexId(1))->setNormalCoord(1,M->getCell(Cellid)->getNormalCoord(1));
      M->getVertex(M->getCell(Cellid)->getVertexId(1))->setNormalCoord(2,M->getCell(Cellid)->getNormalCoord(2));
      M->getVertex(M->getCell(Cellid)->getVertexId(1))->incNumberOfCells();
      M->getVertex(M->getCell(Cellid)->getVertexId(2))->setNormalCoord(0,M->getCell(Cellid)->getNormalCoord(0));
      M->getVertex(M->getCell(Cellid)->getVertexId(2))->setNormalCoord(1,M->getCell(Cellid)->getNormalCoord(1));
      M->getVertex(M->getCell(Cellid)->getVertexId(2))->setNormalCoord(2,M->getCell(Cellid)->getNormalCoord(2));
      M->getVertex(M->getCell(Cellid)->getVertexId(2))->incNumberOfCells();
    };


    vector<Vec> sample_hex(real spacing, real jitter, Vec min, Vec max) {
        const real spacing_2 = spacing/2;
        const real yspacing = spacing*sqrt(3.0);
        const real yspacing_2 = yspacing/2;
        const real zspacing_2 = yspacing/2;
        Vec pos;
        vector<Vec> positions;
        bool yraised = false;
        bool zraised = false;
        for (pos.x = min.x; pos.x <= max.x; pos.x += spacing_2)
        {
          yraised = !yraised;
          if (yraised)
            pos.y = min.y + yspacing_2;
          else
            pos.y = min.y;

          while (pos.y <= max.y)
          {




            pos.y += yspacing;
        zraised = !zraised;
          if (zraised)
            pos.z = min.z + yspacing_2;
          else
            pos.z = min.z;

             while (pos.z <= max.z)
          {

            Vec p = pos;

            if (jitter != 0.0f) {
              p += jitter*spacing*rand01()*randomDirection();

            }
            pos.z +=zspacing_2;
            positions.push_back(p);
          }
        }
        }

        return positions;
      }


    vector<Vec> sample_hex_coord_sphere(real spacing, real jitter, Vec min, Vec max, real radius) {
      const real spacing_2 = spacing/2;
        const real yspacing = spacing*sqrt(3.0);
        const real yspacing_2 = yspacing/2;
        const real zspacing_2 = yspacing/2;
        Vec pos,center;
        vector<Vec> positions;
        center.x =(min.x+max.x)*0.5; center.y =(min.y+max.y)*0.5; center.z =(min.z+max.z)*0.5;
        real maxr = radius;real minr=radius/10.0;real rmax,r = max.x - min.x; rmax = r;
        minr=min.x;maxr=max.x;
        if((max.y - min.y)<r)
           r = (max.y - min.y);
        if((max.y - min.y)>rmax)
        {
          rmax =(max.y - min.y);
          minr=min.y;maxr=max.y;
        }
        if((max.z - min.z)<r)
          r = (max.z - min.z);
        if((max.z - min.z)>rmax)
        {
          rmax = (max.z - min.z);
          minr=min.z;maxr=max.z;
        }
        minr=radius/10.0;
        real r1,theta,phi,thetaStep,phiStep,rStep=minr;
        real r2 = rmax*0.5;
        theta=0.0;
        thetaStep= 2*M_PI/(100.0);
        phiStep = M_PI/(100.0);
        bool yraised = false;
        bool zraised = false;
        for (r1 =minr ; r1 <= maxr; r1 +=rStep)
        {
          yraised = !yraised;
          if (yraised)
            theta+=thetaStep;
          else
            theta=0.0;

          while (theta <= 2*M_PI)
          {




            pos.y += yspacing;
        zraised = !zraised;
          if (zraised)
            phi+=phiStep;
          else
            phi=0.0;

             while (phi <= M_PI)
          {
         pos.x= center.x+r1*sin(phi)*cos(theta);
         pos.y= center.y+r1*sin(phi)*sin(theta);
         pos.z= center.z+ r1*cos(phi);
            Vec p = pos;

            if (jitter != 0.0f) {
              p += jitter*spacing*rand01()*randomDirection();
              positions.push_back(p);
            }

            phi+=phiStep;

          }
          theta+=thetaStep;
        }
        }

        return positions;
      }

    vector<Vec> sample_hex_sphere(real spacing, real jitter, Vec min, Vec max) {
    const real spacing_2 = spacing/2;
      const real yspacing = spacing*sqrt(3.0);
      const real yspacing_2 = yspacing/2;
      const real zspacing_2 = yspacing/2;
      Vec pos,center;
      vector<Vec> positions;
      center.x =(min.x+max.x)*0.5; center.y =(min.y+max.y)*0.5; center.z =(min.z+max.z)*0.5;
      real maxr,minr,rmax,r = max.x - min.x; rmax = r;
      minr=min.x;maxr=max.x;
      if((max.y - min.y)<r)
         r = (max.y - min.y);
      if((max.y - min.y)>rmax)
      {
        rmax =(max.y - min.y);
        minr=min.y;maxr=max.y;
      }
      if((max.z - min.z)<r)
        r = (max.z - min.z);
      if((max.z - min.z)>rmax)
      {
        rmax = (max.z - min.z);
        minr=min.z;maxr=max.z;
      }
      real r2 = rmax*0.5;
      bool yraised = false;
      bool zraised = false;
      for (pos.x =center.x-r2 ; pos.x <= center.x+r2; pos.x += spacing_2)
      {
        yraised = !yraised;
        if (yraised)
          pos.y = center.y-r2 + yspacing_2;
        else
          pos.y = center.y-r2;

        while (pos.y <= center.y+r2)
        {




          pos.y += yspacing;
      zraised = !zraised;
        if (zraised)
          pos.z = center.z-r2 + zspacing_2;
        else
          pos.z = center.z-r2;

           while (pos.z <= center.z+r2)
        {

          Vec p = pos;

          if (jitter != 0.0f) {
            p += jitter*spacing*rand01()*randomDirection();

          }
          if(sphere_function(center,pos,r)<=0)
        positions.push_back(p);
          pos.z +=zspacing_2;

        }
      }
      }

      return positions;
    }

      void init_coord_sphere(Vec Vi, Vec Vf,real jitter = 0.02, real spacing = 0.02,int type=2) {

        this->jitter = jitter;
        this->spacing = spacing;

        // fill half a [0,1]^2 box with particles
        vector<Vec> pos = sample_hex_coord_sphere(spacing, jitter, Vi, Vf,2.0);
        std::vector<Particle> particles;
        particles.clear();
        for (int i = 0; i < pos.size(); ++i) {
          particles.push_back(Particle(type,dx,pos[i]));

        }
        v_particles.push_back(particles);







        std::cout << "created " << particles.size() << " particles." << std::endl;
        v_particles.push_back(particles);
      }


      void init_sphere(Vec Vi, Vec Vf,real jitter = 0.02, real spacing = 0.02,int type=2) {

        this->jitter = jitter;
        this->spacing = spacing;

        // fill half a [0,1]^2 box with particles
        vector<Vec> pos = sample_hex_sphere(spacing, jitter, Vi, Vf);
        std::vector<Particle> particles;
        //particles.clear();
        for (int i = 0; i < pos.size(); ++i) {
          particles.push_back(Particle(type,dx,pos[i]));

        }







        std::cout << "created " << particles.size() << " particles." << std::endl;
        v_particles.push_back(particles);
      }
  
      void init( Vec Vi, Vec Vf,real jitter = 0.02, real spacing = 0.02,int type=2) {

        this->jitter = jitter;
        this->spacing = spacing;

        // fill half a [0,1]^2 box with particles
        vector<Vec> pos = sample_hex(spacing, jitter, Vi, Vf);
        std::vector<Particle> particles;

        for (int i = 0; i < pos.size(); ++i) {
          particles.push_back(Particle(type,dx,pos[i]));


        }


        std::cout << "created " << particles.size() << " particles." << std::endl;
        v_particles.push_back(particles);
      }

  void step() {

      NormMaxV=-100000.0;NormMinV=10000.0;
        std::memset(grid, 0, sizeof(grid));
  	// reset grid scratchpad
int jj,ii;
        // P2G
        for(ii=0;ii<v_particles.size();ii++)
        for (jj =0;jj<v_particles[ii].size();jj++) {          // P2G
            Particle &p = v_particles[ii][jj];
          Vector3i base_coord =
              (p.x * inv_dx - Vec(0.5_f)).cast<int>();  // element-wise floor
          Vec fx = p.x * inv_dx - base_coord.cast<real>();
          // Quadratic kernels  [http://mpm.graphics   Eqn. 123, with x=fx, fx-1,fx-2]
          Vec w[3]{Vec(0.5) * sqr(Vec(1.5) - fx), Vec(0.75) - sqr(fx - Vec(1.0)),
                   Vec(0.5) * sqr(fx - Vec(0.5))};

          real J = determinant(p.F);  //                         Current volume

          Mat cauchy;
          if (p.type < 1) {
            cauchy = Mat(params.eos_stiffness * (pow<1>(p.Jp) - 1));
          } else {
              auto e = std::exp(hardening * (1.0_f - p.Jp)), mu = mu_0 * e,
                   lambda = lambda_0 * e;
              Mat r;
              double f[9];
              f[0]=p.F(0,0);f[1]=p.F(0,1);f[2]=p.F(0,2); f[3]=p.F(1,0);f[4]=p.F(1,1);f[5]=p.F(1,2); f[6]=p.F(2,0);f[7]=p.F(2,1);f[8]=p.F(2,2);
              double M_r[9],M_s[9];
              PolarDecomposition pc;
              pc.Compute(f,M_r,M_s);
              r(0,0)=M_r[0];r(0,1)=M_r[1];r(0,2)=M_r[2];r(1,0)=M_r[3];r(1,1)=M_r[4];r(1,2)=M_r[5];r(2,0)=M_r[6];r(2,1)=M_r[7];r(2,2)=M_r[8];
              //s(0,0)=M_s[0];s(0,1)=M_s[1];s(0,2)=M_s[2];s(1,0)=M_s[3];s(1,1)=M_s[4];s(1,2)=M_s[5];s(2,0)=M_s[6];s(2,1)=M_s[7];s(2,2)=M_s[8];
            //  polar_decomp(p.F, r, s);  // Polar decomp. for fixed corotated model
            cauchy = 2 * mu * (p.F - r) * transposed(p.F) + lambda * (J - 1) * J;
          }
          auto stress =  // Cauchy stress times dt and inv_dx
              -4 * inv_dx * inv_dx *  dt * J * cauchy;
          auto affine = stress + particle_mass * p.C;
          for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++){  // Scatter to grid
              auto dpos = (Vec(i, j,k) - fx) * dx;
              Vector4 mv(p.v * particle_mass,
                         particle_mass);  // translational momentum
              grid[base_coord.x + i][base_coord.y + j][base_coord.z + k] +=
                  w[i].x * w[j].y*w[k].z * (mv + Vector4(affine * dpos, 0));
            }
        }


        // For all grid nodes
        for(int i = 0; i <= n; i++) {
          for(int j = 0; j <= n; j++) {
              for(int k = 0; k <= n; k++) {
            auto &g = grid[i][j][k];
            // No need for epsilon here
            if (g[3] > 0) {
              // Normalize by mass
              g /= g[3];
              // Gravity
              g += params.dt * Vector4(0, -params.gravity, 0,0);

              // boundary thickness
              real boundary = 0.025;
              real factor=3.99;
              // Node coordinates
              real x = (real) i / n;
              real y = (real) j / n;
              real z = (real) k / n;

              // Sticky boundary
              if (  y > 0.75 - boundary)
              {
                g[0] *= factor*params.a;
                g[1] *= -params.a;
                g[2] *= factor*params.a;
              }
              if(z > 0.5 - boundary)
              {
                g[0] *= factor*params.a;
                g[1] *= factor*params.a;
                g[2] *= -params.a;
              }
              if(z < boundary)
              {
                g[0] *= factor*params.a;
                g[1] *= factor*params.a;
                g[2] *= -params.a;
              }
              if(x > 0.75 - boundary)
              {
                g[0] *= -params.a;
                g[1] *= factor*params.a;
                g[2] *= factor*params.a;
              }
              if (y < boundary)
              {
                g[0] *= factor*params.a;
                g[1] *= -params.a;
                g[2] *= factor*params.a;
              }
              if(x < boundary )
              {
                g[0] *= -params.a;
                g[1] *= factor*params.a;
                g[2] *= factor*params.a;
              }

            }
            }
          }
        }

Vec ox;
for(ii=0;ii<v_particles.size();ii++)
for (jj =0;jj<v_particles[ii].size();jj++) {          // P2G
    Particle &p = v_particles[ii][jj];
  Vector3i base_coord =
      (p.x * inv_dx - Vec(0.5_f)).cast<int>();  // element-wise floor
  Vec fx = p.x * inv_dx - base_coord.cast<real>();
  Vec w[3]{Vec(0.5) * sqr(Vec(1.5) - fx), Vec(0.75) - sqr(fx - Vec(1.0)),
           Vec(0.5) * sqr(fx - Vec(0.5))};
  p.C = Mat(0);
  p.v = Vec(0);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++){
      auto dpos = (Vec(i, j,k) - fx),
           grid_v = Vec(grid[base_coord.x + i][base_coord.y + j][base_coord.z + k]);
      auto weight = w[i].x * w[j].y* w[k].z;
      p.v += weight * grid_v;  // Velocity
      p.C +=
          4 * inv_dx * Mat::outer_product(weight * grid_v, dpos);  // APIC C
    }
  Vec vdx =dt * p.v;                       // Advection
  Vector vdxt(vdx.x,vdx.y,vdx.z);
  bool tc=testAllCollision(v_particles[ii],jj,1.1*dx,vdxt);
  if(tc==false)
  {
   p.x += vdx;
  }
  if (p.type > 0) {                     // plastic
    auto F = (Mat(1) + dt * p.C) * p.F;  // MLS-MPM F-update
    if (p.type == 1) {
      Mat svd_u, sig, svd_v;
      Mat3d M_F,M_svd_u, M_svd_v;
      Vec3d V_sig;
      M_F.set(F(0,0),F(0,1),F(0,2),F(1,0),F(1,1),F(1,2),F(2,0),F(2,1),F(2,2));
      SVD(M_F,M_svd_u,V_sig,M_svd_v);
      svd_u(0,0)=M_svd_u[0][0];svd_u(0,1)=M_svd_u[0][1];svd_u(0,2)=M_svd_u[0][2];svd_u(1,0)=M_svd_u[1][0];svd_u(1,1)=M_svd_u[1][1];svd_u(1,2)=M_svd_u[1][2];svd_u(2,0)=M_svd_u[2][0];svd_u(2,1)=M_svd_u[2][1];svd_u(2,2)=M_svd_u[2][2];
      sig(0,0)=V_sig[0];sig(0,1)=0.0;sig(0,2)=0.0;sig(1,0)=0.0;sig(1,1)=V_sig[1];sig(1,2)=0.0;sig(2,0)=0.0;sig(2,1)=0.0;sig(2,2)=V_sig[2];
      svd_v(0,0)=M_svd_v[0][0];svd_v(0,1)=M_svd_v[0][1];svd_v(0,2)=M_svd_v[0][2];svd_v(1,0)=M_svd_v[1][0];svd_v(1,1)=M_svd_v[1][1];svd_v(1,2)=M_svd_v[1][2];svd_v(2,0)=M_svd_v[2][0];svd_v(2,1)=M_svd_v[2][1];svd_v(2,2)=M_svd_v[2][2];
      //svd(F, svd_u, sig, svd_v);
      for (int i = 0; i < 3* int(plastic); i++)  // Snow Plasticity
        sig[i][i] = clamp(sig[i][i], 1.0_f - 2.5e-2_f, 1.0_f + 7.5e-3_f);
      real oldJ = determinant(F);
      F = svd_u * sig * transposed(svd_v);
      real Jp_new = clamp(p.Jp * oldJ / determinant(F), 0.6_f, 20.0_f);
      p.Jp = Jp_new;
    }
    p.F = F;
  } else {  // liquid
    p.Jp *= determinant(Mat(1) + dt * p.C);
  }


  real tmpNormV;
  Vector vi(p.v.x,p.v.y,p.v.z);
  tmpNormV=vi.norm();
  if(tmpNormV>NormMaxV)
   NormMaxV = tmpNormV;
  if(tmpNormV<NormMinV)
   NormMinV = tmpNormV;


    }

    }


  inline bool testAllCollision(std::vector<Particle> &particles,int pId, float r,Vector &ndx)
  {

    bool resp = false;
    if(MalhaObst!=NULL)
    {
        Particle &p = particles[pId];
        Vector3i base_coord =
            (p.x * inv_dx).cast<int>();  // element-wise floor

            int ii;
            for(ii=0;ii<gridObst[base_coord.x][base_coord.y][base_coord.z].size();ii++)
            {

               if (TestCollisionRotation(meshHandlerObst,particles,gridObst[base_coord.x][base_coord.y][base_coord.z][ii],pId,r,ndx))
               {
                   auto &g = grid[base_coord.x][base_coord.y][base_coord.z];
                resp= true;
                g[0]=particles[pId].v.x;
                g[1]=particles[pId].v.y;
                g[2]=particles[pId].v.z;
               }
            }
    }
     /*  of::ofCellsIterator<TTraits> ico(&meshHandler);
            for(ico.initialize(); ico.notFinish(); ++ico)
            {

               if (TestCollision(meshHandler,&ico,pId,r,ndx))
                resp= true;
            }*/
            return resp;

  }



  inline float ImplicitLine(float p1x,float p1y,float p2x,float p2y,float px,float py)
  {
      float dx=(p2x-p1x);
      float dy=(p2y-p1y);
      float a =dy,b=-dx,c=(-p1x*dy+p1y*dx);
      return a*px+b*py+c;
  }

  inline bool TestCollisionRotation(Handler<TMesh> &Mesh,std::vector<Particle> &particles,int cellId, int pId,float r, Vector &ndx)
  {
    Particle &pi = particles[pId];
    Vector vnt(Mesh->getCell(cellId)->getNormalCoord(0),Mesh->getCell(cellId)->getNormalCoord(1),Mesh->getCell(cellId)->getNormalCoord(2));
    Vector Pvn(pi.v.x,pi.v.y,pi.v.z);
    float nv =Pvn.norm();
    Pvn.normalize();
    float vn = Pvn*vnt;
    if(vn<0.0)
    {

       // Pvnpi.v;

        Vector p1(Mesh->getVertex(Mesh->getCell(cellId)->getVertexId(0))->getCoord(0),Mesh->getVertex(Mesh->getCell(cellId)->getVertexId(0))->getCoord(1),Mesh->getVertex(Mesh->getCell(cellId)->getVertexId(0))->getCoord(2));
        Vector px(pi.x.x,pi.x.y,pi.x.z);
        float d = (px - p1)*vnt;
        if(fabs(d)<2*r)
        {
        Vector p2(Mesh->getVertex(Mesh->getCell(cellId)->getVertexId(1))->getCoord(0),Mesh->getVertex(Mesh->getCell(cellId)->getVertexId(1))->getCoord(1),Mesh->getVertex(Mesh->getCell(cellId)->getVertexId(1))->getCoord(2));
        Vector p3(Mesh->getVertex(Mesh->getCell(cellId)->getVertexId(2))->getCoord(0),Mesh->getVertex(Mesh->getCell(cellId)->getVertexId(2))->getCoord(1),Mesh->getVertex(Mesh->getCell(cellId)->getVertexId(2))->getCoord(2));
        //Vector p=pi.x;
        Vector p2p1(p2.x-p1.x,p2.y-p1.y,p2.z-p1.z);
        Vector p3p1(p3.x-p1.x,p3.y-p1.y,p3.z-p1.z);
        Vector pip1(pi.x.x-p1.x,pi.x.y-p1.y,pi.x.z-p1.z);
        Vector vntx0(0.0,Mesh->getCell(cellId)->getNormalCoord(1),Mesh->getCell(cellId)->getNormalCoord(2));
        vntx0.normalize();
        Vector axey(0.0,1.0,0.0);
        Vector axez(0.0,0.0,1.0);
        Vector vaxex1(1,0,0);
        Vector vaxez1(0,0,1);
        Vector vaxey1(0,1,0);
        double angle1 = vnt*vntx0;
        vntx0.normalize();
        double angle2 =vntx0*axey;

        TQuaternion qz(angle1,vaxez1);

        if(vnt.x<0.0)
            qz=qz.inverse();

        TQuaternion qx(angle2,vaxex1);
        if(vntx0.z>0.0)
            qx=qx.inverse();

         TQuaternion qr=qz*qx;
         qr.QuatRotation(&p2p1,&p2p1);
         qr.QuatRotation(&p3p1,&p3p1);
         qr.QuatRotation(&pip1,&pip1);
         //qr.QuatRotation(&vnt,&vnt);
         qr.QuatRotation(&Pvn,&Pvn);



        /* qx.QuatRotation(&p2p1,&p2p1);
         qx.QuatRotation(&p3p1,&p3p1);
         qx.QuatRotation(&pip1,&pip1);
         qx.QuatRotation(&vnt,&vnt);
         qx.QuatRotation(&Pvn,&Pvn);*/
         Vector p2p1N(p2p1.x,p2p1.y,p2p1.z);
         p2p1N.normalize();
         double angle3 = axez*p2p1N;
         TQuaternion qy(angle3,vaxey1);
         if(p2p1.x>0.0)
             qy=qy.inverse();

         qy.QuatRotation(&p2p1,&p2p1);
         qy.QuatRotation(&p3p1,&p3p1);
         qy.QuatRotation(&pip1,&pip1);
         qy.QuatRotation(&Pvn,&Pvn);
         float penetration =pip1.y-r;
        float i1,i2;
        i1 = ImplicitLine(0.0,p2p1.z,p3p1.x,p3p1.z,pip1.x,pip1.z);
        i2 = ImplicitLine(0.0,0.0,p3p1.x,p3p1.z,pip1.x,pip1.z);
        //(penetration<0.0)&&((pip1.x)>0.0)&&(i1>=0.0)&&(i2<=0.0) positive orientation
        //(penetration<0.0)&&((pip1.x)<0.0)&&(i1<=0.0)&&(i2>=0.0) negative orientattion
      //  std::cout <<"penetration = " << penetration << " pip1.x = " << pip1.x << " i1 = " << i1 << " i2 = " << i2 << std::endl;
         if((penetration<0.0)&&((pip1.x)<0.0)&&(i1<=0.0)&&(i2>=0.0))
         {
             taichi::real factor=3.99;


            pip1.y=r+ params.a  * (-1.0*penetration) + 0.001*rand01();
            pip1.x+=( (factor*params.a ) * penetration) * dt * pi.v.x;
            pip1.z+=( (factor*params.a ) * penetration) * dt * pi.v.z;
            TQuaternion qiy=qy.inverse();
            //TQuaternion qix=qx.inverse();
            //TQuaternion qiz=qz.inverse();
            TQuaternion qir=qr.inverse();//qiz*qix*qiy;

            qiy.QuatRotation(&pip1,&pip1);
            qir.QuatRotation(&pip1,&pip1);
            //qix.QuatRotation(&pip1,&pip1);
            //qiz.QuatRotation(&pip1,&pip1);

            //qiz.QuatRotation(&vnt,&vnt);
            Pvn.x *= factor*params.a ;
            Pvn.y *= -params.a ;
            Pvn.z *= factor*params.a ;
            //
            qiy.QuatRotation(&Pvn,&Pvn);
            qir.QuatRotation(&Pvn,&Pvn);
            //qix.QuatRotation(&Pvn,&Pvn);
            //qiz.QuatRotation(&Pvn,&Pvn);
           /* Vector univ = pi.v_new; univ.normalize();
             Vector nv =   univ - 2.0*(univ*vnt)*vnt;
             float c,normv=pi.v_new.norm();

            nv.normalize();*/
            pip1.x+=p1.x;pip1.y+=p1.y;pip1.z+=p1.z;
            pi.v.x =nv*Pvn.x;pi.v.y =nv*Pvn.y;pi.v.z =nv*Pvn.z;


              //pi.v = 0.5*(pi.v_new+pi.v_old);
              pi.x.x=  pip1.x; pi.x.y=  pip1.y;pi.x.z=  pip1.z; //+0.001*rand01();


              return true;
         }
         else {
             return false;
         }
        }
        else
            return false;

    }
       else
        return false;


   }

  inline int n_particles() const {
    return v_particles.size();
  }
  
  inline Particle const &particle(int i, int j) const {
    return v_particles[i][j];
  }

  inline Particle &particle(int i,int j) {
    return v_particles[i][j];
  }

  inline void WriteScreenImage() {

	/*
	 * GET FROM http://local.wasp.uwa.edu.au/~pbourke/rendering/windowdump/
	 * 
	 Write the current view to a file
	 The multiple fputc()s can be replaced with
	 fwrite(image,width*height*3,1,fptr);
	 If the memory pixel order is the same as the destination file format.
	 */

	int i, j;
	FILE *fptr;
	char fname[32];
	unsigned char *image;

	int width, height;
	width = winWidth;
	height = winHeight;

	/* Allocate our buffer for the image */
	 image = reinterpret_cast<unsigned char*>(malloc(3*width*height*sizeof(char)));
	if (image == NULL) {
		fprintf(stderr, "Failed to allocate memory for image\n");
		//return (false);
	}

	glPixelStorei(GL_PACK_ALIGNMENT, 1);

	sprintf(fname, "%s_%04d.ppm", this->screenFilename.c_str(),
			this->screenFilenameNumber);

	if ((fptr = fopen(fname, "w")) == NULL) {
		fprintf(stderr, "Failed to open file for window dump\n");
//		return false;
	}

	/* Copy the image into our buffer */
	glReadBuffer(GL_BACK_LEFT);
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);

	/* Write the raw file */
	fprintf(fptr, "P6\n%d %d\n255\n", width, height);
	for (j = height - 1; j >= 0; j--) {
		for (i = 0; i < width; i++) {
			fputc(image[3 * j * width + 3 * i + 0], fptr);
			fputc(image[3 * j * width + 3 * i + 1], fptr);
			fputc(image[3 * j * width + 3 * i + 2], fptr);
		}
	}
	fclose(fptr);

	/* Clean up */
	screenFilenameNumber++;
	// free(image);
	//return true;
}


	
};
