#ifndef SPH_H
#define SPH_H
#include "ColorRGBA.hpp"
#include "Cores.h"

#include "vector.hh"
#include "neighbordata.hh"
#include "kdtree.hh"
#include "hashgrid.hh"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#define soundwater 1450.0
//##################################################################//


struct Particle {
  // position
  Vector x;
  // velocity
  Vector v;
  
  // acceleration acting on this particle in this step
  Vector a;
  // smoothed velocity for artificial viscosity
  Vector v_smoothed;
  // density
  float rho;
  // pressure
  float p;
    // Temperature
  float T;
  // temperature gradient
  float dt;
  TColorRGBA C;
  
  Particle(Vector const &pos, float Tini=25.0): x(pos), v(0,0,0), a(0,0,0) {
   T=Tini;
  }
};

inline float rand01()
{
  return float(rand())/RAND_MAX;
}

inline Vector randomDirection()
{
  float alpha = 2 * M_PI * rand01();
  float beta = 2 * M_PI * rand01();
  
  return Vector(cos(alpha)*cos(beta), sin(alpha)*cos(beta), sin(beta));
}

struct std_kernel {
  float h, h_squared, h_third, h_fourth;
  
  std_kernel(float h_): h(h_), h_squared(h*h), h_third(h_squared*h), h_fourth(h_squared*h_squared) {
  }
  
  inline float value(float d, float d_squared) const {
    if (d_squared >= h_squared)
      return 0.f;
    else {
      float x = 1.f - d_squared/h_squared;
      return 4.f/(M_PI*h_squared) * x * x * x;
    }
  }
  
  inline float first_derivative(float d, float d_squared) const {
    if (d >= h)
      return 0.f;
    else {
      float x = 1.f - d_squared/h_squared;
      return -24.f*d/(M_PI*h_fourth) * x * x;
    }
  }
  
  inline Vector gradient(NeighborData const &nd) const {
    return - first_derivative(nd.d, nd.d_squared) * nd.d_normalized;
  }

  inline float second_derivative(float d, float d_squared) const {
    if (d_squared >= h_squared)
      return 0.f;
    else {
      float x = d_squared/h_squared;
      return 24.f / (M_PI*h_fourth) * (1 - x) * (5 * x - 1);
    }
  }
};

struct spiky_kernel {
  float h, h_squared, h_third, h_fourth;
  
  spiky_kernel(float h_): h(h_), h_squared(h*h), h_third(h_squared*h), h_fourth(h_squared*h_squared) {
  }

  inline float value(float d, float d_squared) const {
    if (d >= h)
      return 0.f;
    else {
      float x = 1.f - d/h;
      return 10.f / (M_PI*h_squared) * x * x * x;
    }
  }
  
  inline float first_derivative(float d, float d_squared) const {
    if (d >= h)
      return 0.f;
    else {
      float x = 1.f - d/h;
      return -30.f / (M_PI*h_third) * x * x;
    }
  }
  
  inline Vector gradient(NeighborData const &nd) const {
    return - first_derivative(nd.d, nd.d_squared) * nd.d_normalized;
  }
  
  inline float second_derivative(float d, float d_squared) const {
    if (d >= h)
      return 0.f;
    else {
      float x = 1.f - d/h;
      return 60.f / (M_PI*h_fourth) * x;
    }
  }
};

struct Parameters {
  // time step
  float dt;
  
  // sph internal
  float rel_smoothing_radius;
  
  // constants for compressibility model
  int gamma;
  float K;
  
  // rest density of fluid
  float rho0;
  
  // fraction of pressure to apply for negative pressures
  float zeta;
  
  // viscosity parameters
  float nu;
  // artificial viscosity
  float xi;
  bool real_xsph;
  
   // environment
  Vector gravity;
  // restitution
  float a;  
  //thermal diffusion constant
  float alpha;
  int steps;
};

class SPH {
  // creation parameters
  float jitter;
  float spacing;
  // effective smoothing radius (computed from spacing)
  float h;
  // mass of each particle (computed from density and particle spacing)
  float m;
  
  Parameters params;
  
  // cell size for surface extraction
  float isovalue;
  int xsurfacecells, ysurfacecells;
  float cellsize;

  // particle store
  
  
  // spatial search data structures
  
  std::vector<std::vector<NeighborData> > neighbors;
    
  // function mapping density to pressure
  inline float pressure(float rho) {
    float p = params.K * (pow((rho/params.rho0 ), params.gamma)- 1.f)+params.rho0;      
    //if (p < 0)
      //p *= params.zeta;
    return p;
  }
  
public:
std::string screenFilename;
int winWidth, winHeight ;
int screenFilenameNumber;
 KdTree tree;
  //Hashgrid;
  // kernels
  typedef spiky_kernel pressure_kernel;
  typedef std_kernel viscosity_kernel;
  float Tmax,Tmin;
  std::vector<Particle> particles;
  pressure_kernel pkernel;
  viscosity_kernel vkernel;
  
  SPH(): h(0), pkernel(h), vkernel(h) { 
	screenFilenameNumber=0;
winWidth =800; winHeight =600;
screenFilename = "screenVis";
    // set default parameter values
    
    // sph internals
    params.rel_smoothing_radius = 2.5;

    // material parameters
    params.rho0 = 998.6;
    params.K = 2000	;
    params.nu = 0.002; // 0.002 is a good value if used alone
    params.xi = 0.025; // 0.01 is a good value if used alone
    params.real_xsph = false; // real XSPH (smoothed velocity only used in integration) is much weaker
    params.steps=0;
    // pressure computation
    params.zeta = 1.0;
    params.gamma = 7;
    
    // environment
    params.gravity = Vector(0, -9.81, 0);
    params.a = 0.5;
        
    // time step
    params.dt = 0.001;
    
    // surface extraction
    isovalue = 0.5;
    surfaceCells(50);
    params.alpha = 0.0001;
  }  

  inline void parameters(Parameters const &pm) {
    bool set_r = params.rel_smoothing_radius != pm.rel_smoothing_radius;
    bool set_m = params.rho0 != pm.rho0;
    params = pm;
    if (set_r) 
      smoothing_radius(spacing * params.rel_smoothing_radius);
    else if (set_m) 
      compute_mass();
  }
  
  vector<Vector> sample_hex(float spacing, float jitter, Vector min, Vector max) {
    const float spacing_2 = spacing/2;
    const float yspacing = spacing*sqrt(3.0);
    const float yspacing_2 = yspacing/2;
    
    Vector pos;
    vector<Vector> positions;
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
        Vector p = pos;
        
        if (jitter != 0.0f) {
          p += jitter*spacing*rand01()*randomDirection();
          p.z = 0;
        }
        
        positions.push_back(p);
        pos.y += yspacing;
      }
    }
    
    return positions;
  }
  
  void init(float jitter = 0.02, float spacing = 0.02) {
    this->jitter = jitter;
    this->spacing = spacing;
    
    // fill half a [0,1]^2 box with particles    
    vector<Vector> pos = sample_hex(spacing, jitter, Vector(0,0,0), Vector(0.55, 1.05, 0));
    
    particles.clear();
    for (int i = 0; i < pos.size(); ++i) {
      particles.push_back(Particle(pos[i]));
    }
        
    int q = smoothing_radius(spacing * params.rel_smoothing_radius);
 
    // update search data structures
    updateSearcher(tree);
    //updateSearcher(grid);

    // reserve memory for neighbors and compute neighbors
    neighbors.resize(n_particles());
    for (int i = 0; i < n_particles(); ++i) {
      neighbors[i].reserve(2*q);
      particles[i].T=25.0; particles[i].dt=0.0;
      
      tree.neighbors(particles[i].x, neighbors[i]);
    }
    Tmax=25.0;Tmin=25.0;      
    std::cout << "created " << n_particles() << " particles." << std::endl;
  }
  void NormalizedColor(float T,TColorRGBA *C)
  {
           if((Tmax - Tmin)>0.01)
	   {
            float normalizedV = 2*((T-Tmin)/(Tmax-Tmin))-1.0;
             float jump =floor((1.0+normalizedV)/0.05);
                        if(normalizedV<0.0)
                        {
                                C->R=15/255;
                                C->G = (15+6*jump)/255;
                                C->B = (255-6*jump)/255;
                                
                        }
                        else
                        {
                                C->R = (15+6*(jump))/255;
                                C->G =(255- 6*(jump))/255;
                                C->B=15/255;
                        }
	   }
	   else
	   {
	     C->R = 0.0;
	     C->G = 0.0;
	     C->B=1.0;
	   }
  
  }
  
  float P_ij(int i, int j)
  {
    float a,alpha = 1.0;
    Vector v_ij = particles[i].v - particles[neighbors[i][j].idx].v;
    Vector x_ij = particles[i].x - particles[neighbors[i][j].idx].x;
    a = v_ij * x_ij;
    if((neighbors[i][j].d<0.00001)||(a >=0.0))
      return 0.0;
    else
    {
      
      float nu_ij = h*a*(neighbors[i][j].d_squared + 0.01*h*h);
      float resp =-alpha*nu_ij*soundwater/(0.5*(particles[i].rho + particles[neighbors[i][j].idx].rho));
      //printf ("viscosity = %f\n",resp);
      return  resp;
    }
	
  }
  
  template<class Searcher>
  void updateSearcher(Searcher &searcher) const {
    // create search data structure 
    searcher.clear();
    for (int i = 0; i < n_particles(); ++i) {
      searcher.insert(i, particles[i].x);
    }
    searcher.init();    
  }
  
  void step() {
    // 0th pass: update search data structure (rebuild from scratch)
    updateSearcher(tree);
    //updateSearcher(grid);
      
    // first pass: compute neighbors, compute densities and pressures
    for (int i = 0; i < n_particles(); ++i) {
      tree.neighbors(particles[i].x, neighbors[i]);
      float normw=0.0;
      float invnormw;  
      particles[i].rho = 0;
      for (int j = 0; j < neighbors[i].size(); ++j) {
        particles[i].rho += m * pkernel.value(neighbors[i][j].d, neighbors[i][j].d_squared);
      }
         
        
        
        if(params.steps==40)
        {
            for (int j = 0; j < neighbors[i].size(); ++j) {
                normw+= pkernel.value(neighbors[i][j].d, neighbors[i][j].d_squared)*(m/particles[neighbors[i][j].idx].rho);
            }       
            if(normw>0.0)
           invnormw = 1.0/normw;
           else
            invnormw=1.0;
        
            //printf("invnormw = %f  normw = %f\n",invnormw, normw);
            particles[i].rho*=invnormw;
            
        }
        
      
      particles[i].p = pressure(particles[i].rho);
    }
    if(params.steps==40)
        params.steps=0;
        else
        params.steps++;
    
    
    float p1,p2,p3;
    
    // second pass: compute accelerations and smoothed velocity, add gravity
    for (int i = 0; i < n_particles(); ++i) {
      Particle &pi = particles[i];

      pi.v_smoothed = Vector(0,0,0);
      float w = 0;
      pi.dt=0.0;
      for (int j = 0; j < neighbors[i].size(); ++j) {
       if(neighbors[i][j].idx==i)
        continue;
        Particle &pj = particles[neighbors[i][j].idx];

        // pressure forces
        pi.a -= m * (pi.p/(pi.rho*pi.rho) + pj.p/(pj.rho*pj.rho)) * pkernel.gradient(neighbors[i][j]);

        // viscosity
        //pi.a += params.nu * (pj.v - pi.v) * m/(pi.rho*pj.rho) * vkernel.second_derivative(neighbors[i][j].d, neighbors[i][j].d_squared);
	    pi.a+=(2.0*params.nu *  m)/(pj.rho) *(((pi.x-pj.x)*vkernel.gradient(neighbors[i][j]))/(neighbors[i][j].d_squared+0.01*h*h))*(pi.v - pj.v);

        // artificial viscosity
        
        //(teste viscosidade artificial)
	pi.a-=((m/(pi.rho))*P_ij(i,j)*vkernel.gradient(neighbors[i][j]));
        
        
        float wj = m/pj.rho * vkernel.value(neighbors[i][j].d, neighbors[i][j].d_squared);
        w += wj;
        pi.v_smoothed += pj.v * wj;
        //update temperature gradient;
	
	//pi.dt += (m/pj.rho)*(pi.T-pj.T)*pkernel.second_derivative(neighbors[i][j].d, neighbors[i][j].d_squared);
//         p1=((m/pj.rho)*(4.0*pi.rho/(pi.rho+pj.rho)));
//         p2 = (pi.T-pj.T);
//         p3 = (((pi.x-pj.x)*pkernel.gradient(neighbors[i][j]))/(neighbors[i][j].d_squared- 0.001*h*h));
//         pi.dt+= p1*p2*p3;
      }
      //std::cout <<  " dT = " << pi.T << std::endl;
      //pi.dt*=params.alpha;
      
      // normalize velocity estimate
      pi.v_smoothed /= w;

      // gravity
      pi.a += params.gravity;
    }
    
    // third pass: apply artificial viscosity, integrate, clear transient fields
    for (int i = 0; i < n_particles(); ++i) {
      Particle &pi = particles[i];
      
      // integrate
      Vector ox = pi.x, dx;        
      if (params.real_xsph) {
        pi.v += params.dt * pi.a;
        pi.v_smoothed += params.dt * pi.a;
        dx = params.dt * ((1-params.xi) * pi.v + params.xi * pi.v_smoothed);
      } else {
        pi.v = (1-params.xi) * pi.v + params.xi * pi.v_smoothed;
        pi.v += params.dt * pi.a;
        dx = params.dt * pi.v;
      }
      /*pi.T+= params.dt*pi.dt;
      if(pi.T>Tmax)
       Tmax = pi.T;
      if(pi.T<Tmin)
       Tmin = pi.T;*/
      pi.x += dx;

      // clear forces
      pi.a = Vector(0,0,0);

      // enforce boundaries (reflect at boundary, with restitution a)
      // 2D only!
      if (pi.x.x > 1.75) {
        float penetration = (pi.x.x - 1.75);
        pi.x.x = (1.75 - params.a * penetration - 0.001*rand01());
        pi.x.y = ox.y + (1 + (params.a-1) * penetration/fabs(dx.x)) * params.dt * pi.v.y;
        
        pi.v.x *= -params.a;
        pi.v.y *= params.a;
      } else if (pi.x.x < 0) {
        float penetration = -pi.x.x;
        pi.x.x = params.a * penetration + 0.001*rand01();
        pi.x.y = ox.y + (1 + (params.a-1) * penetration/fabs(dx.x)) * params.dt * pi.v.y;
        
        pi.v.x *= -params.a;
        pi.v.y *= params.a;        
      }
      
      if (pi.x.y < 0) {
        float penetration = -pi.x.y;
        pi.x.x = ox.x + (1 + (params.a-1) * penetration/fabs(dx.y)) * params.dt * pi.v.x;
        pi.x.y = params.a * penetration + 0.001*rand01();
        
        pi.v.x *= params.a;
        pi.v.y *= -params.a;        
      }
    }
    //std:: cout << "Tmin = " << Tmin << " Tmax = " << Tmax << std::endl;
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


  inline float smoothing_radius() {
    return h;
  }
  
  // returns the approximate number of neighbors
  inline int smoothing_radius(float newh) {
    h = newh;
    pkernel = pressure_kernel(h);
    vkernel = viscosity_kernel(h);
    //grid.queryRadius(h);
    tree.queryRadius(h);
    
    return compute_mass();
  }
  
  // returns the approximate number of neighbors
  inline int compute_mass() {
    // set mass s.t. sampling approximately yields density rho0 
    std::vector<Vector> pos = sample_hex(spacing, 0, Vector(-h, -h, 0), Vector(h, h, 0));
    float rho_a = 0;
    int q = 0;
    for (int i = 0; i < pos.size(); ++i) {
      float d = pos[i].norm();
      float k = pkernel.value(d, d*d);
      rho_a += k;
      if (k > 0)
        q++;
    }
    // normalize mass according to particle density
    m = params.rho0/rho_a;    
    
    std::cout << "rho0 = " << params.rho0 << ", h = " << h << ", q = " << q << ", m = " << m << std::endl;
    
    return q;
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
  
  inline KdTree const &kdtree() const {
    return tree;
  }

  inline HashGrid const &hashgrid() const {
    //return grid;
  }
  
  // surface extraction
  inline void surfaceCells(int cs) {
    xsurfacecells = cs;
    ysurfacecells = cs * 1.1 + 1;
    cellsize = 1.f/cs;
  }
  
  inline void drawSurfaceGrid() const {
    glLineWidth(1);
    glColor3f(0.6, 0.6, 0);
    glBegin(GL_LINES);
    for (float x = 0; x <= 1+1e-6; x += cellsize) {
      glVertex2f(x, 0);
      glVertex2f(x, 1.1);
    }
    for (float y = 0; y <= 1.1+1e-6; y += cellsize) {
      glVertex2f(0, y);
      glVertex2f(1, y);
    }
    glEnd();    
  }
  
  inline void drawSurface() const {
    // Do marching squares, iterate over all cells. 
    // If the fluid is "sparse", ie if there are lots of cells and particles in only few of them,
    // it makes sense to do fast marching stating at each particle (but only once per cell). 
    std::vector<float> value((xsurfacecells+1) * (ysurfacecells+1));
    std::vector<NeighborData> nbs;
    for (int x = 0; x <= xsurfacecells; ++x) {
      for (int y = 0; y <= ysurfacecells; ++y) {
        Vector p(x*cellsize, y*cellsize, 0);
        float &rho = value[y * (xsurfacecells+1) + x];
        tree.neighbors(p, nbs);
        
        // compute density at each point
        rho = 0;
        for (int i = 0; i < nbs.size(); ++i) {
          rho += m * pkernel.value(nbs[i].d, nbs[i].d_squared);
        }
      }   
    } 

    /* 
    // visualize grid values
    glPointSize(3);
    glBegin(GL_POINTS);
    for (int x = 0; x <= xsurfacecells; ++x) {
      for (int y = 0; y <= ysurfacecells; ++y) {    
        // get the four corner values
        // order is x y
        float v00 = value[y * (xsurfacecells+1) + x] - isovalue;
        if (v00 >= 0)
          glColor3f(1, 0, 0);
        else
          glColor3f(0, 1, 0);
            
        glVertex2f(x*cellsize, y*cellsize);
      }
    }        
    glEnd();
    */
    
    glColor3f(1, 1, 1);
    glBegin(GL_LINES);
    for (int x = 0; x < xsurfacecells; ++x) {
      for (int y = 0; y < ysurfacecells; ++y) {    
        // get the four corner values
        // order is x y
        
        // v10 - yhi - v11
        //  |           |
        // xlo         xhi
        //  |           |
        // v00 - ylo - v10

        struct intersector {
          // coordinates
          float xlo, xhi, ylo, yhi;
          // values
          float v00, v10, v01, v11;
          
          // find zero crossing by interpolation
          inline float zero(float v0, float v1, float xlo, float xhi) const {
            float a = v0/(v0-v1);     
            return (1-a) * xlo + a * xhi;
          }
          
          // get marching squares code
          inline char code() const {
            return ((v00 < 0) * 1) | ((v10 < 0) * 2) | ((v01 < 0) * 4) | ((v11 < 0) * 8);            
          }
          
          // get crossing vertices
          inline void vxlo() const {
            if (v00 * v01 > 0) throw;
            glVertex2f(xlo, zero(v00, v01, ylo, yhi));
          }
          inline void vxhi() const {
            if (v10 * v11 > 0) throw;
            glVertex2f(xhi, zero(v10, v11, ylo, yhi));
          }          
          inline void vylo() const {
            if (v00 * v10 > 0) throw;
            glVertex2f(zero(v00, v10, xlo, xhi), ylo);
          }
          inline void vyhi() const {
            if (v01 * v11 > 0) throw;
            glVertex2f(zero(v01, v11, xlo, xhi), yhi);
          }
        };
        
        intersector intersect;
        
        intersect.v00 = value[y * (xsurfacecells+1) + x] - isovalue;
        intersect.v10 = value[y * (xsurfacecells+1) + x+1] - isovalue;
        intersect.v01 = value[(y+1) * (xsurfacecells+1) + x] - isovalue;
        intersect.v11 = value[(y+1) * (xsurfacecells+1) + x+1] - isovalue;

        intersect.xlo = x * cellsize;
        intersect.xhi = intersect.xlo + cellsize;
        intersect.ylo = y * cellsize;
        intersect.yhi = intersect.ylo + cellsize;
        
        // marching squares, draw directly
        // code(binary) = v11 v01 v10 v00
    
        switch (intersect.code()) {
          case 0:
          case 0xf:
            // no intersections
            break;
          case 1: // 0001
          case 0xe: // 1110
            intersect.vxlo();
            intersect.vylo();
            break;
          case 2: // 0010
          case 0xd: // 1101
            intersect.vxhi();
            intersect.vylo();
            break;
          case 3: // 0011
          case 0xc: // 1100
            intersect.vxlo();
            intersect.vxhi();
            break;
          case 4: // 0100
          case 0xb: // 1011
            intersect.vxlo();
            intersect.vyhi();
            break;
          case 5: // 0101
          case 0xa: // 1010
            intersect.vyhi();
            intersect.vylo();
            break;
          case 6: // 0110
          case 9: // 1001
            intersect.vxlo();
            intersect.vylo();
            intersect.vxhi();
            intersect.vyhi();
            break;
          case 7: // 0111
          case 8: // 1000
            // intersections on yhi and xhi
            intersect.vxhi();
            intersect.vyhi();
            break;
        }
      }
    }
    glEnd();
  }
};


#endif
