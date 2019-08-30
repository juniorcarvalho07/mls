#include "ColorRGBA.hpp"
#include "Cores.h"
#include "taichi.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>


using namespace taichi;
using Vec = Vector2;
using Mat = Matrix2;

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
        float mass;
        float padding; // just for performance
  		TColorRGBA color;

        Particle(Vec x, Vec v=Vec(0)):
        x(x),
    	v(v),
    	C(0){}
    };

    struct Cell {
        Vec v; // velocity
        float mass;
        float padding; // just for performance
    };

    struct Parameters {
  // time step
  float dt;

//Grid Resolution
  int gridRes;  
  
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
Vec weights[3];


	MLS_MPM(){

	winWidth =800; winHeight =600;
	screenFilename = "screenVis";
	
// environment

    params.gridRes=64;

    params.a = 0.5;
        
    // time step
    params.dt = 0.01;


    params.gravity = 0.98f;
    // fluid parameters
    params.rest_density = 1.0f;
    params.dynamic_viscosity = 0.1f;
    // equation of state
    params.eos_stiffness = 100.0f;
    params.eos_power = 1;
    params.mass=1.0;
	};
	~MLS_MPM(){};

	std::vector<Particle> particles;
	std::vector<Cell> cells;

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
  
  void init(float jitter = 0.0, float spacing = 0.015) {
    this->jitter = jitter;
    this->spacing = spacing;
    
    // fill half a [0,1]^2 box with particles    
    vector<Vec> pos = sample_hex(spacing, jitter, Vec(0,0.2), Vec(0.55, 1.25));
    
    particles.clear();
    for (int i = 0; i < pos.size(); ++i) {
      particles.push_back(Particle(pos[i]));
    }
        
    num_cells = params.gridRes * params.gridRes;
    dx = (2.0-(-0.2))/params.gridRes;
    dy = (2.0-(-0.2))/params.gridRes;
    inv_dx = 1.0/dx;
    inv_dy = 1.0/dy;
    cells.resize(num_cells);


    
    std::cout << "created " << particles.size() << " particles and " << cells.size() << " cells." << std::endl;
  }

  void step() {


  	// reset grid scratchpad
        for (int i = 0; i < num_cells; ++i) {
            Cell &c = cells[i];

            c.mass = 0;
            c.v = Vec(0);

            }

        // P2G
        for (int i = 0; i < n_particles(); ++i) {
      			Particle &pi = particles[i];
                pi.mass=params.mass;
            // quadratic interpolation weights
            Vector2i base_coord (pi.x.x * inv_dx-0.5*dx,pi.x.y * inv_dy-0.5*dy);
            //Vector2i cell_idx;
            Vec fx(pi.x.x  - base_coord.cast<real>().x*dx-0.5*dx,pi.x.y  - base_coord.cast<real>().y*dy-0.5*dy);

    		// Quadratic kernels [http://mpm.graphics Eqn. 123, with x=fx, fx-1,fx-2]
            Vec w[3] = {
              Vec(0.5) * sqr(Vec(0.5) - fx),
              Vec(0.75) - sqr(fx ),
              Vec(0.5) * sqr(fx + Vec(0.5))
            };

            // for all surrounding 9 cells
            for (int gx = 0; gx < 3; ++gx) {
                for (int gy = 0; gy < 3; ++gy) {
                    float weight = w[gx].x * w[gy].y;

                    Vector2i cell_x(std::max(0,base_coord.x + gx - 1),std::max(0, base_coord.y + gy - 1));
                    Vec cell_dist (cell_x.x*dx - pi.x.x+0.5*dx,cell_x.y*dy - pi.x.y+0.5*dy);
                    Vec Q = pi.C*cell_dist;

                    // MPM course, equation 172
                    float mass_contrib = weight * pi.mass;

                    // converting 2D index to 1D
                    int cell_index = (int)cell_x.x * params.gridRes + (int)cell_x.y;
                    Cell &cell = cells[cell_index];

                    // scatter mass to the grid
                    cell.mass += mass_contrib;
                    
                    cell.v += mass_contrib * (pi.v + Q);
                                       // note: currently "cell.v" refers to MOMENTUM, not velocity!
                    // this gets corrected in the UpdateGrid step below.
                }
            }
        }

        // we now have 2 P2G phases as we need to ensure we have scattered particle masses to the grid,
        // in order to get our density estimate at each frame
        for (int i = 0; i < n_particles(); ++i) {
                Particle &pi = particles[i];
                // quadratic interpolation weights
                Vector2i base_coord = (pi.x * inv_dx - Vec(dx*0.5f)).cast<int>();
                //Vector2i cell_idx;
                Vec fx(pi.x.x  - base_coord.cast<real>().x*dx-0.5*dx,pi.x.y  - base_coord.cast<real>().y*dy-0.5*dy);

                // Quadratic kernels [http://mpm.graphics Eqn. 123, with x=fx, fx-1,fx-2]
                Vec w[3] = {
                  Vec(0.5) * sqr(Vec(0.5) - fx),
                  Vec(0.75) - sqr(fx ),
                  Vec(0.5) * sqr(fx + Vec(0.5))
                };
                // estimating particle volume by summing up neighbourhood's weighted mass contribution
                // MPM course, equation 152
                float density = 0.0f;
                 int gx, gy;
                for (gx = 0; gx < 3; ++gx) {
                    for (gy = 0; gy < 3; ++gy) {
                        float weight = w[gx].x * w[gy].y;

                   Vector2i cell_xx(std::max(0,base_coord.x + gx - 1),std::max(0, base_coord.y + gy - 1));
                   // converting 2D index to 1D
                   int cell_index = (int)cell_xx.x * params.gridRes + (int)cell_xx.y;
                   Cell &cell = cells[cell_index];
                   density +=cell.mass*weight;
                    }
                }

                float volume = pi.mass / density;

                // end goal, constitutive equation for isotropic fluid:
                // stress = -pressure * I + viscosity * (velocity_gradient + velocity_gradient_transposed)

                // Tait equation of state. i clamped it as a bit of a hack.
                // clamping helps prevent particles absorbing into each other with negative pressures
                float pressure =  params.eos_stiffness * (std::pow(density / params.rest_density, params.eos_power) - 1)+params.rest_density;
                Mat stress;
                stress(0,0)=-pressure;stress(0,1)=0.0; stress(1,0)=0.0; stress(1,1)=-pressure;


                // velocity gradient - CPIC eq. 17, where deriv of quadratic polynomial is linear
                Mat dudv = pi.C;
                Mat strain = dudv;

                float trace = strain(0,0) + strain(1,1);
                strain(0,1) = strain(1,0) = trace;

                Mat viscosity_term = params.dynamic_viscosity * strain;
                stress += viscosity_term;

                Mat eq_16_term_0 = -volume * 4 * stress * params.dt;

                for (gx = 0; gx < 3; ++gx) {
                    for (gy = 0; gy < 3; ++gy) {
                        float weight = w[gx].x * w[gy].y;

                        Vector2i cell_x(std::max(0,base_coord.x + gx - 1),std::max(0, base_coord.y + gy - 1));
                        Vec cell_dist (cell_x.x*dx - pi.x.x+0.5*dx,cell_x.y*dy - pi.x.y+0.5*dy);
                        // converting 2D index to 1D
                        int cell_index = (int)cell_x.x * params.gridRes + (int)cell_x.y;
                        Cell &cell = cells[cell_index];

                        // fused force + momentum contribution from MLS-MPM
                        Vec momentum =  weight*eq_16_term_0 *cell_dist;
                        cell.v += momentum;


                    }
                }


        }




// grid velocity update
        for (int i = 0; i < cells.size(); ++i) {
            Cell &cell = cells[i];

            if (cell.mass > 0) {
                // convert momentum to velocity, apply gravity
                cell.v /= cell.mass;
                cell.v += params.dt * Vec(0, -params.gravity);  //gravity

                // boundary conditions
                int x = i / params.gridRes;
                int y = i % params.gridRes;
                if (x < 0 || x > params.gridRes - 3) { cell.v.x = 0; }
                if (y < 0 || y > params.gridRes - 3) { cell.v.y = 0; }

                
            }

            
        }

Vec ox;
  	for (int i = 0; i < particles.size(); ++i) {
      Particle &pi = particles[i];


        ox=pi.x;
      // reset particle velocity. we calculate it from scratch each step using the grid
            pi.v = Vec(0);

            // quadratic interpolation weights
            Vector2i base_coord = (pi.x * inv_dx - Vec(dx*0.5f)).cast<int>();
            //Vector2i cell_idx;
            Vec fx(pi.x.x  - base_coord.cast<real>().x*dx-0.5*dx,pi.x.y  - base_coord.cast<real>().y*dy-0.5*dy);

            // Quadratic kernels [http://mpm.graphics Eqn. 123, with x=fx, fx-1,fx-2]
            Vec w[3] = {
              Vec(0.5) * sqr(Vec(0.5) - fx),
              Vec(0.75) - sqr(fx ),
              Vec(0.5) * sqr(fx + Vec(0.5))
            };

            // constructing affine per-particle momentum matrix from APIC / MLS-MPM.
            // see APIC paper (https://web.archive.org/web/20190427165435/https://www.math.ucla.edu/~jteran/papers/JSSTS15.pdf), page 6
            // below equation 11 for clarification. this is calculating C = B * (D^-1) for APIC equation 8,
            // where B is calculated in the inner loop at (D^-1) = 4 is a constant when using quadratic interpolation functions
            Mat B(0);
            for (int gx = 0; gx < 3; ++gx) {
                for (int gy = 0; gy < 3; ++gy) {
                    float weight = w[gx].x * w[gy].y;

                    Vector2i cell_x(std::max(0,base_coord.x + gx - 1),std::max(0, base_coord.y + gy - 1));
                    Vec dist (cell_x.x*dx - pi.x.x+0.5*dx,cell_x.y*dy - pi.x.y+0.5*dy);
                    int cell_index = (int)cell_x.x * params.gridRes + (int)cell_x.y;
                     Cell &c = cells[cell_index];

                    Vec weighted_velocity = weight*c.v ;

                    // APIC paper equation 10, constructing inner term for B
                    Mat term= Mat(weighted_velocity * dist.x, weighted_velocity * dist.y);

                    B += term;

                    pi.v += weighted_velocity;
                }
            }
            pi.C = B * 4;

            // advect particles
            pi.x += pi.v * params.dt;


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
