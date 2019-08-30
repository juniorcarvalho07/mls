#include "ColorRGBA.hpp"
#include "Cores.h"
#include "vector.hh"
#include "neighbordata.hh"
#include "kdtree.hh"
#include "taichi.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>




using namespace taichi;
using Vec = Vector2;
using Mat = Matrix2;

const int n = 100 /*grid resolution (cells)*/, window_size = 800;
const real dt = 60e-4_f / n , frame_dt = 1e-2_f, dx = 1.75_f / n,
           inv_dx = 1.0_f / dx;
auto particle_mass = 1.0_f, vol = 1.0_f;
auto hardening = 10.0_f, E = 1e4_f, nu = 0.2_f;
real mu_0 = E / (2 * (1 + nu)), lambda_0 = E * nu / ((1 + nu) * (1 - 2 * nu));
const bool plastic = false;
//Grid Resolution

Vector3 grid[n + 1][n + 1];


inline float rand01()
{
  return float(std::rand())/RAND_MAX;
}

inline Vec randomDirection()
{
  float alpha = 2 * M_PI * rand01();
  float beta = 2 * M_PI * rand01();
  
  return Vec(cos(alpha)*cos(beta), sin(alpha)*cos(beta));
}

  struct Particle {
        Vec x; // position
        Vec v; // velocity
        Mat C; // affine momentum matrix, unused in this file
        // Deformation gradient
        Mat F;
        // Determinant of the deformation gradient (i.e. volume)
        real Jp;
        float mass;
        int type;  // 0: elastic   1: plastic   2: liquid
  		TColorRGBA color;

        Particle(int t=2,Vec x =Vec(0), Vec v=Vec(0)):
        x(x),
    	v(v),
          F(1),
          C(0),
          type(t),
          Jp(1){
            switch(type){
            case 2:
                color =white;
                break;
            case 1:
                color =yellow;
                break;
            case 0:
                color =red;
                break;

            }
        }
    };


    struct Parameters {
  // time step
  float dt;


  
   // environment

  // restitution
  float a;  
  float mass;

   float gravity;
  // fluid parameters
   float rest_density ;
   float dynamic_viscosity;
  // equation of state
  float eos_stiffness;
  float eos_power;

};





class MLS_MPM
{
public:
// creation parameters
  float jitter;
  float spacing;
std::string screenFilename;
int winWidth, winHeight ;
int screenFilenameNumber;
int num_cells;
Parameters params;
KdTree tree;


	MLS_MPM(){

	winWidth =800; winHeight =600;
	screenFilename = "screenVis";
	
// environment



    params.a = 0.1;
        
    // time step
    params.dt = 10e-4_f;


    params.gravity = 100.8f;
    // fluid parameters
    params.rest_density = 1.0f;
    params.dynamic_viscosity = 0.02f;
    // equation of state
    params.eos_stiffness = 1000.0f;
    params.eos_power = 1;
    params.mass=1.0;
    num_cells=(n+1)*(n+1);
    particles.clear();
	};
	~MLS_MPM(){};

	std::vector<Particle> particles;
    std::vector<std::vector<NeighborData> > neighbors;

    template<class Searcher>
    void updateSearcher(Searcher &searcher) const {
      // create search data structure
      searcher.clear();
      for (int i = 0; i < particles.size(); ++i) {
        searcher.insert(i, Vector(particles[i].x.x,particles[i].x.y));
      }
      searcher.init();
    }

	vector<Vec> sample_hex(float spacing, float jitter, Vec min, Vec max) {

    const float spacing_2 = spacing/2;
    const float yspacing = spacing*sqrt(3.0);
    const float yspacing_2 = yspacing/2;
    
    Vec pos;
    vector<Vec> positions;
    bool yraised = false;
    for (pos.x = min.x; pos.x <= max.x; pos.x += spacing_2)
    {
      yraised = !yraised;
      if (yraised)
        pos.y = min.y + yspacing_2;
      else
        pos.y = min.y;
      
      while (pos.y <= max.y)
      {
        Vec p = pos;
        
        if (jitter != 0.0f) {
          p += jitter*spacing*rand01()*randomDirection();
          
        }
        
        positions.push_back(p);
        pos.y += yspacing;
      }
    }
    
    return positions;
  }

    void initSearch()
    {
        tree.queryRadius(0.05);
        updateSearcher(tree);

        int q=25;
        // reserve memory for neighbors and compute neighbors
        neighbors.resize(n_particles());
        for (int i = 0; i < n_particles(); ++i) {
          neighbors[i].reserve(2*q);


          tree.neighbors(Vector(particles[i].x.x,particles[i].x.y), neighbors[i]);
        }
    }
  
  void init(Vec Vi,Vec Vf,int type=2,float jitter = 0.0, float spacing = 0.02) {
    this->jitter = jitter;
    this->spacing = spacing;
    
    // fill half a [0,1]^2 box with particles    
    //vector<Vec> pos = sample_hex(spacing, jitter, Vec(0.85,0.6), Vec(1.05, 1.2));
    

    //for (int i = 0; i < pos.size(); ++i) {
      //particles.push_back(Particle(0,pos[i]));
    //}

    vector<Vec> pos1 = sample_hex(spacing, jitter, Vi,Vf);


    for (int i = 0; i < pos1.size(); ++i) {
      particles.push_back(Particle(type,pos1[i]));
    }
        




    
    std::cout << "created " << particles.size() << " particles and " << num_cells << " cells." << std::endl;
  }

  void step() {
        updateSearcher(tree);
        std::memset(grid, 0, sizeof(grid));  // Reset grid
          for (auto &p : particles) {          // P2G
            Vector2i base_coord =
                (p.x * inv_dx - Vec(0.5_f)).cast<int>();  // element-wise floor
            Vec fx = p.x * inv_dx - base_coord.cast<real>();
            // Quadratic kernels  [http://mpm.graphics   Eqn. 123, with x=fx, fx-1,fx-2]
            Vec w[3]{Vec(0.5) * sqr(Vec(1.5) - fx), Vec(0.75) - sqr(fx - Vec(1.0)),
                     Vec(0.5) * sqr(fx - Vec(0.5))};

            real J = determinant(p.F);  //                         Current volume

            Mat cauchy;
            if (p.type == 2) {
              cauchy = Mat(0.2_f * E * (pow<1>(p.Jp) - 1));
            } else {
                auto e = std::exp(hardening * (1.0_f - p.Jp)), mu = mu_0 * e,
                     lambda = lambda_0 * e;
                Mat r, s;
                polar_decomp(p.F, r, s);  // Polar decomp. for fixed corotated model
              cauchy = 2 * mu * (p.F - r) * transposed(p.F) + lambda * (J - 1) * J;
            }
            auto stress =  // Cauchy stress times dt and inv_dx
                -4 * inv_dx * inv_dx * dt * vol * cauchy;
            auto affine = stress + particle_mass * p.C;
            for (int i = 0; i < 3; i++)
              for (int j = 0; j < 3; j++) {  // Scatter to grid
                auto dpos = (Vec(i, j) - fx) * dx;
                Vector3 mv(p.v * particle_mass,
                           particle_mass);  // translational momentum
                grid[base_coord.x + i][base_coord.y + j] +=
                    w[i].x * w[j].y * (mv + Vector3(affine * dpos, 0));
              }
          }
          for (int i = 0; i <= n; i++)
            for (int j = 0; j <= n; j++) {  // For all grid nodes
              auto &g = grid[i][j];
              if (g[2] > 0) {                   // No need for epsilon here
                g /= g[2];                      //        Normalize by mass
                g += dt * Vector3(0, -9.8, 0);  //                  Gravity
                real boundary = 0.05, x = (real)i / n,
                     y = real(j) / n;  // boundary thick.,node coord
                real factor =2.0;
                if (  y > 1 - boundary)
                  g = Vector3(0);  // Sticky
                if(x > 1 - boundary)
                {
                    g[0] *=-params.a; //std::max(0.0_f, g[0]);  //"Separate"
                    g[1] *=factor*params.a; //std::max(0.0_f, g[0]);  //"Separate"
                }
                if (y < boundary)
                {
                    g[0] *=factor*params.a; //std::max(0.0_f, g[0]);  //"Separate"
                    g[1] *=-params.a;  //"Separate"
                }
                if(x < boundary )
                {
                    g[0] *=-params.a;  //"Separate"
                    g[1] *=factor*params.a; //std::max(0.0_f, g[0]);  //"Separate"
                }
              }
            }
          for (auto &p : particles) {  // Grid to particle
            Vector2i base_coord =
                (p.x * inv_dx - Vec(0.5_f)).cast<int>();  // element-wise floor
            Vec fx = p.x * inv_dx - base_coord.cast<real>();
            Vec w[3]{Vec(0.5) * sqr(Vec(1.5) - fx), Vec(0.75) - sqr(fx - Vec(1.0)),
                     Vec(0.5) * sqr(fx - Vec(0.5))};
            p.C = Mat(0);
            p.v = Vec(0);
            for (int i = 0; i < 3; i++)
              for (int j = 0; j < 3; j++) {
                auto dpos = (Vec(i, j) - fx),
                     grid_v = Vec(grid[base_coord.x + i][base_coord.y + j]);
                auto weight = w[i].x * w[j].y;
                p.v += weight * grid_v;  // Velocity
                p.C +=
                    4 * inv_dx * Mat::outer_product(weight * grid_v, dpos);  // APIC C
              }
            p.x += dt * p.v;                       // Advection
            if (p.type <= 1) {                     // plastic
              auto F = (Mat(1) + dt * p.C) * p.F;  // MLS-MPM F-update
              if (p.type == 1) {
                Mat svd_u, sig, svd_v;
                svd(F, svd_u, sig, svd_v);
                for (int i = 0; i < 2; i++)  // Snow Plasticity
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
          }

    // enforce boundaries (reflect at boundary, with restitution a)
      // 2D only!
      /*if (pi.x.x > 1.75) {
        float penetration = (pi.x.x - 1.75);
        pi.x.x = (1.75 - params.a * penetration );
        pi.x.y = ox.y + (1 + (params.a-1) * penetration) * params.dt * pi.v.y;
        
        pi.v.x *= -params.a;
        pi.v.y *= params.a;
      } else if (pi.x.x < 0) {
        float penetration = -pi.x.x;
        pi.x.x = params.a * penetration  ;
        pi.x.y = ox.y + (1 + (params.a-1) * penetration) * params.dt * pi.v.y;
        
        pi.v.x *= -params.a;
        pi.v.y *= params.a;        
      }
      
      if (pi.x.y < 0) {
        float penetration = -pi.x.y;
        pi.x.x = ox.x + (1 + (params.a-1) * penetration) * params.dt * pi.v.x;
        pi.x.y = params.a * penetration;
        
        pi.v.x *= params.a;
         pi.v.y *= -params.a;
      }*/


    }

  inline int n_particles() const {
    return particles.size();
  }
  
  inline Particle const &particle(int i) const {
    return particles[i];
  }

  inline Particle &particle(int i) {
    return particles[i];
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
    char convert[256];
    char dele[256];
    sprintf(convert,"convert %s %s_%04d_time_%.4f.png ", fname,this->screenFilename.c_str(),this->screenFilenameNumber,time);
    system(convert);
    sprintf(dele,"rm %s ", fname);
    system(dele);
	/* Clean up */
	screenFilenameNumber++;
	// free(image);
	//return true;
}


	
};
