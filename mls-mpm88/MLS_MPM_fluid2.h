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

const auto vol = 1.0_f;        // Particle Volume
const auto hardening = 10.0_f; // Snow hardening factor
const auto E = 1e4_f;          // Young's Modulus
const auto nu = 0.2_f;         // Poisson ratio
const bool plastic = true;
//Grid Resolution
 const int gridRes =128;
Vector3 grid[gridRes + 1][gridRes + 1];
// Initial Lamé parameters
const real mu_0 = E / (2 * (1 + nu));
const real lambda_0 = E * nu / ((1+nu) * (1 - 2 * nu));


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

  		TColorRGBA color;

        Particle(Vec x, Vec v=Vec(0)):
        x(x),
    	v(v),
          F(1),
          C(0),
          Jp(1){}
    };

    struct Cell {
        Vec v; // velocity
        float mass;
        float padding; // just for performance
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
float dx;
float dy;
float inv_dx;
float inv_dy;
Parameters params;
KdTree tree;


	MLS_MPM(){

	winWidth =800; winHeight =600;
	screenFilename = "screenVis";
	
// environment



    params.a = 0.3;
        
    // time step
    params.dt = 0.005;


    params.gravity = 9.8f;
    // fluid parameters
    params.rest_density = 1.0f;
    params.dynamic_viscosity = 0.002f;
    // equation of state
    params.eos_stiffness = 1000.0f;
    params.eos_power = 4;
    params.mass=1.0;

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
  
  void init(float jitter = 0.0, float spacing = 0.02) {
    this->jitter = jitter;
    this->spacing = spacing;
    
    // fill half a [0,1]^2 box with particles    
    vector<Vec> pos = sample_hex(spacing, jitter, Vec(0.02,0.3), Vec(0.6, 1.75));
    
    particles.clear();
    for (int i = 0; i < pos.size(); ++i) {
      particles.push_back(Particle(pos[i]));
    }
        
    num_cells = gridRes * gridRes;
    dx = (2.0-(-0.1))/gridRes;
    dy = (2.0-(-0.1))/gridRes;
    inv_dx = 1.0/dx;
    inv_dy = 1.0/dy;
    tree.queryRadius(0.05);
    updateSearcher(tree);

    int q=25;
    // reserve memory for neighbors and compute neighbors
    neighbors.resize(n_particles());
    for (int i = 0; i < n_particles(); ++i) {
      neighbors[i].reserve(2*q);


      tree.neighbors(Vector(particles[i].x.x,particles[i].x.y), neighbors[i]);
    }

    
    std::cout << "created " << particles.size() << " particles and " << num_cells << " cells." << std::endl;
  }

  void step() {
        updateSearcher(tree);
        std::memset(grid, 0, sizeof(grid));
  	// reset grid scratchpad

        // P2G
        for (int i = 0; i < n_particles(); ++i) {
            Particle &pi = particles[i];
            pi.mass=params.mass;
            // element-wise floor
            Vector2i base_coord = (pi.x * inv_dx - Vec(0.5f)).cast<int>();

            Vec fx = pi.x * inv_dx - base_coord.cast<real>();

            // Quadratic kernels [http://mpm.graphics Eqn. 123, with x=fx, fx-1,fx-2]
            Vec w[3] = {
              Vec(0.5) * sqr(Vec(1.5) - fx),
              Vec(0.75) - sqr(fx - Vec(1.0)),
              Vec(0.5) * sqr(fx - Vec(0.5))
            };
            auto affine =  pi.mass * pi.C;

            int gx, gy;
           for (gx = 0; gx < 3; gx++) {
               for (gy = 0; gy < 3; gy++) {
                   auto dpos = (Vec(gx, gy) - fx) * dx;
                   // Translational momentum
                   Vector3 mass_x_velocity(pi.v * pi.mass, pi.mass);
                   grid[base_coord.x + gx][base_coord.y + gy] += (
                     w[gx].x*w[gy].y * (mass_x_velocity + Vector3(affine * dpos, 0))
                   );
               }
           }


        }

        //cfd contribuition


        for (int i = 0; i < n_particles(); ++i) {
            Particle &pi = particles[i];
            // element-wise floor
            Vector2i base_coord = (pi.x * inv_dx - Vec(0.5f)).cast<int>();

            Vec fx = pi.x * inv_dx - base_coord.cast<real>();

            // Quadratic kernels [http://mpm.graphics Eqn. 123, with x=fx, fx-1,fx-2]
            Vec w[3] = {
              Vec(0.5) * sqr(Vec(1.5) - fx),
              Vec(0.75) - sqr(fx - Vec(1.0)),
              Vec(0.5) * sqr(fx - Vec(0.5))
            };
            // estimating particle volume by summing up neighbourhood's weighted mass contribution
            // MPM course, equation 152
            float density = 0.0f;

            int gx, gy;
           for (gx = 0; gx < 3; gx++) {
               for (gy = 0; gy < 3; gy++) {
                   float weight = w[gx].x * w[gy].y;
                    auto &grid_v = grid[base_coord.x + gx][base_coord.y + gy];



                   density +=grid_v[2]*weight;
               }
           }

           float volume = pi.mass / density;

           // end goal, constitutive equation for isotropic fluid:
           // stress = -pressure * I + viscosity * (velocity_gradient + velocity_gradient_transposed)

           // Tait equation of state. i clamped it as a bit of a hack.
           // clamping helps prevent particles absorbing into each other with negative pressures
           float pressure =  std::max(-20.0f,params.eos_stiffness * (std::pow(density / params.rest_density, params.eos_power) - 1));
           Mat stress;
           stress(0,0)=-pressure;stress(0,1)=0.0; stress(1,0)=0.0; stress(1,1)=-pressure;


           // velocity gradient - CPIC eq. 17, where deriv of quadratic polynomial is linear
           Mat dudv = pi.C;
           Mat strain = dudv;

           float trace = strain(0,0) + strain(1,1);
           strain(0,1) = strain(1,0) = trace;

           Mat viscosity_term = params.dynamic_viscosity * strain;
           stress += viscosity_term;

           Mat eq_16_term_0 = -volume*params.dt * 4 * stress  ;

           for (gx = 0; gx < 3; gx++) {
               for (gy = 0; gy < 3; gy++) {
                   float weight = w[gx].x * w[gy].y;

                   //Vec dpos(base_coord.x * dx-pi.x.x,base_coord.y * dx-pi.x.y);
                   auto dpos = (Vec(gx, gy) - fx) * dx;

                   auto &grid_v =grid[base_coord.x + gx][base_coord.y + gy];
                   Vec momentum =  (eq_16_term_0*weight) *dpos;
                   grid_v[0] += momentum.x; grid_v[1] += momentum.y;
               }
           }

        }




        // For all grid nodes
        for(int i = 0; i <= gridRes; i++) {
          for(int j = 0; j <= gridRes; j++) {
            auto &g = grid[i][j];
            // No need for epsilon here
            if (g[2] > 0) {
              // Normalize by mass
              g /= g[2];
              // Gravity
              g += params.dt * Vector3(0, -params.gravity, 0);

              // boundary thickness
              real boundary = 0.0005;
              // Node coordinates
              real x = (real) i / gridRes;
              real y = real(j) / gridRes;

              // Sticky boundary
              if (x < boundary || x > 1.75-boundary || y > 2.0-boundary) {
                g = Vector3(0);
              }
              // Separate boundary
              if (y < boundary) {
                g[1] = std::max(0.0f, g[1]);
              }
            }
          }
        }

Vec ox;
  	for (int i = 0; i < particles.size(); ++i) {
      Particle &pi = particles[i];

      Vector2i base_coord = (pi.x * inv_dx - Vec(0.5f)).cast<int>();
      Vec fx = pi.x * inv_dx - base_coord.cast<real>();
      Vec w[3] = {
                  Vec(0.5) * sqr(Vec(1.5) - fx),
                  Vec(0.75) - sqr(fx - Vec(1.0)),
                  Vec(0.5) * sqr(fx - Vec(0.5))
      };

      //pi.C = Mat(0);
      pi.v = Vec(0);
        int gx, gy;
        Mat B =Mat(0);
      for (gx = 0; gx < 3; gx++) {
          for (gy = 0; gy < 3; gy++) {
              float weight = w[gx].x * w[gy].y;
              auto dpos = (Vec(gx, gy) - fx) * dx;

              auto grid_v = Vec(grid[base_coord.x + gx][base_coord.y + gy]);
              Vec weighted_velocity =  weight * grid_v ;

              // APIC paper equation 10, constructing inner term for B
              Mat term = inv_dx * Mat::outer_product(weighted_velocity, dpos);//Mat(weighted_velocity * dpos.x, weighted_velocity * dpos.y);

              B += term;

              pi.v += weighted_velocity;
          }
      }
      pi.C = B ;


        ox=pi.x;
      // Advection
      pi.x += params.dt * pi.v;



    // enforce boundaries (reflect at boundary, with restitution a)
      // 2D only!
      if (pi.x.x > 1.75) {
        float penetration = (pi.x.x - 1.75);
        pi.x.x = (1.75 - params.a * penetration - 0.001*rand01());
        pi.x.y = ox.y + (1 + (params.a-1) * penetration) * params.dt * pi.v.y;
        
        pi.v.x *= -params.a;
        pi.v.y *= params.a;
      } else if (pi.x.x < 0) {
        float penetration = -pi.x.x;
        pi.x.x = params.a * penetration + 0.001*rand01();
        pi.x.y = ox.y + (1 + (params.a-1) * penetration) * params.dt * pi.v.y;
        
        pi.v.x *= -params.a;
        pi.v.y *= params.a;        
      }
      
      if (pi.x.y < 0) {
        float penetration = -pi.x.y;
        pi.x.x = ox.x + (1 + (params.a-1) * penetration) * params.dt * pi.v.x;
        pi.x.y = params.a * penetration + 0.001*rand01();
        
        pi.v.x *= params.a;
         pi.v.y *= -params.a;
      }
    }

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

	/* Clean up */
	screenFilenameNumber++;
	// free(image);
	//return true;
}


	
};
