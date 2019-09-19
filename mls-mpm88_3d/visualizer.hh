#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <GL/glew.h>
#include <GL/glut.h>
#include "MarchingCubes.h"
//#include "vector.hh"
#include "neighbordata.hh"
#include "hashgrid.hh"
#include "particle.hh"
#include "spaceMesh.hh"
#define sphereRadius 1.0

typedef std::vector<std::vector<NeighborData>> Tneighbors;


struct Intersector {

 std::vector<Vector> *auxVector;
 std::vector< std::vector<unsigned int> > polys;
   //coordinates
  taichi::real xlo, xhi, ylo, yhi,zlo,zhi;
  // values
  taichi::real v000, v100, v010, v110 ,v001, v101, v011, v111;
  Vector p000, p100, p010, p110 ,p001, p101, p011, p111;
  // find zero crossing by interpolation
  inline taichi::real zero(taichi::real v0, taichi::real v1, taichi::real xlo, taichi::real xhi) const {
    if ((v0-v1)>0.000000001)
    {
    taichi::real a = v0/(v0-v1);
    return (1-a) * xlo + a * xhi;
    }
    else return xlo;
  }
};


  struct bounding_box{
    Vec bmin,bmax;
  public:
    inline void Print(){std::cout <<" xmin = " << bmin.x << " ymin = " << bmin.y << " zmin = " << bmin.z <<" xmax = " << bmax.x << " ymax = " << bmax.y << " zmax = " << bmax.z << std::endl;};
  };




  class Visualizer{

  public:

      //Thomas Lewinner Marching Cubes

      MarchingCubesL mc ;
      SpaceMesh SM;
      std::vector<HashGrid> v_hash;
      std::vector<bounding_box> v_bbox;
      std::vector<Tneighbors> v_neighbors;
      bounding_box bbox;
      int xsurfacecells, ysurfacecells,zsurfacecells;
      taichi::real cellsizex,cellsizey,cellsizez;
      taichi::real isovalue,Wmax,Wmin;
      Intersector intersect;
      int SurfaceCells;
       std::vector<taichi::real> value;
      Visualizer(){
          SurfaceCells=90;
          Wmax=1.5;
          Wmin=0.0;
      }
      ~Visualizer(){}
      inline void incSurfaceCells()
      {
          SurfaceCells*=1.1;
      }
      inline void decSurfaceCells()
      {
          SurfaceCells*=0.9;
      }
      template<class Searcher>
      void updateSearcher(Searcher &searcher,std::vector<Particle> &particles)
      {
        // create search data structure
        searcher.clear();
        bounding_box bbox;
        bbox.bmin=particles[0].x;bbox.bmax=particles[0].x;
        for (int i = 0; i < particles.size(); ++i) {
          searcher.insert(i, Vector(particles[i].x.x,particles[i].x.y,particles[i].x.z));
          if(particles[i].x.x<bbox.bmin.x) bbox.bmin.x = particles[i].x.x;
          if(particles[i].x.y<bbox.bmin.y) bbox.bmin.y = particles[i].x.y;
          if(particles[i].x.z<bbox.bmin.z) bbox.bmin.z = particles[i].x.z;
          if(particles[i].x.x>bbox.bmax.x) bbox.bmax.x = particles[i].x.x;
          if(particles[i].x.y>bbox.bmax.y) bbox.bmax.y = particles[i].x.y;
          if(particles[i].x.z>bbox.bmax.z) bbox.bmax.z = particles[i].x.z;
        }
       v_bbox.push_back(bbox);
        searcher.init();
        /*Tneighbors neighbors;
        neighbors.resize(particles.size());
        for (int i = 0; i < particles.size(); i++) {
          neighbors[i].reserve(100);
          searcher.neighbors(Vector(particles[i].x.x,particles[i].x.y,particles[i].x.z), neighbors[i]);
        }
        v_neighbors.push_back(neighbors);*/

      }


      inline void init(int mode,std::vector<std::vector<Particle>> &v_particles, float h)
      {

          int i;
          for (i=0;i<v_particles.size();i++)
          {
              HashGrid hash;
              hash.queryRadius(h);
              if(mode==1)
                updateSearcher(hash,v_particles[i]);
              v_hash.push_back(hash);
          }
      }


      /*-----------------------------------------------------------------*/
      //Surface Reconstruction from Onderik 2012 - begin
      /*-----------------------------------------------------------------*/

      inline taichi::real g(taichi::real w)
      {
          if(w>Wmax)
             return 1.0;

          if(w<Wmin)
              return 0.0;

        taichi::real w1 = (w-Wmax)*(w-Wmax);
        taichi::real w2 = (Wmax-Wmin)*(Wmax-Wmin);
        return (1-w1/w2)*(1-w1/w2);
      }


      inline taichi::real ker(taichi::real dd,taichi::real sr)
      {
          taichi::real ss = sr*sr;
          if(dd<=ss)
          {
        taichi::real aux =(1-dd/ss);
        return aux*aux*aux;
          }
          else
              return 0.0;
      }

      inline taichi::real phi_o(HashGrid &hash,Vector p, std::vector<NeighborData> &nbs, std::vector<Particle> &particles,taichi::real sr, taichi::real R)
      {
        taichi::real den=0.0;
        taichi::real wj,invwj,a,f,r;
        Vector Paux(0,0,0);
        int i,j,k,s= nbs.size();
        std::vector<NeighborData> nbsj;
        for(i=0;i<s;i++)
        {
          j = nbs[i].idx;

          hash.neighbors(Vector(particles[j].x.x,particles[j].x.y,particles[j].x.z),nbsj);
          wj=0.0;
          for(k=0;k<nbsj.size();k++)
          {
              if(nbsj[k].idx==j)
                  continue;
            wj+=ker(nbsj[k].d_squared,sr);//wij(neighbors[j][k].d,smoothing_radius());//
          }
          if(wj>0.15)
        invwj=1/wj;
          else
             invwj=0.0;
          a=invwj*ker((p-Vector(particles[j].x.x,particles[j].x.y,particles[j].x.z)).sqrnorm(),sr);//wij((p-particles[j].x).norm(),smoothing_radius());//
          Paux+=a*Vector(particles[j].x.x,particles[j].x.y,particles[j].x.z);
          den+=a;

        }
        if(den==0.0)
            return 1.0;
        Paux = (1/den)*Paux;
        std::vector<NeighborData> nbsk;
        hash.neighbors(Paux,nbsj);
        s= nbsj.size();

          a=0.0;
          for(i=0;i<s;i++)
          {      j = nbsj[i].idx;
            hash.neighbors(Vector(particles[j].x.x,particles[j].x.y,particles[j].x.z),nbsk);
        wj=0.0;
        for(k=0;k<nbsk.size();k++)
        {
            if(nbsk[k].idx==j)
            continue;
          wj+=ker(nbsk[k].d_squared,sr);//wij(neighbors[j][k].d,smoothing_radius());//
        }
            if(wj>0.15)
          invwj=1/wj;
        else
          invwj=0.0;

        a+=invwj*ker((Paux - Vector(particles[j].x.x,particles[j].x.y,particles[j].x.z)).sqrnorm(),sr);//wij((Paux - particles[j].x).norm(),smoothing_radius());//
          }
            if(a>10.0)
                printf("stop\n");
          f=g(a);


        r = (p-Paux).norm() -0.875*R*f;

        return r;


      }





    /*-----------------------------------------------------------------*/
      //Surface Reconstruction from Onderik 2012 - end
      /*-----------------------------------------------------------------*/






      inline taichi::real ks(taichi::real s)
      {
        taichi::real s2=s*s;
        taichi::real aux= (1.0-s2)*(1.0-s2)*(1.0-s2) ;
        if(aux>0.0) return aux;
        else return 0.0;
      }

      inline taichi::real phi_z(Vec x,std::vector<NeighborData> &nbs,std::vector<Particle> &particles,taichi::real r)
      {
        // Yongning Zhu sibgrapi 2005


        taichi::real invden,den, R= 3.0*r;

        int j,i,k,s; den=0.0;
        for ( j = 0; j < nbs.size(); j++) {
              den +=ks(nbs[j].d/R);
            }
         if(den<0.00000001)
             return 1.0;
         else
           invden = 1/den;
        Vec xa(0,0,0);taichi::real ra=0.0;
        taichi::real wi;
        taichi::real wj,invwj,a;
        for(i=0;i<nbs.size();i++)
        {
          wi = ks(nbs[i].d/R);
          xa+= wi*particles[nbs[i].idx].x;
          ra+=wi*r*2.0;
        }
        xa=invden*xa;
        ra=invden*ra;

        return (x-xa).length() -ra;
      }


      void DrawMesh(std::vector<Vector> &MCMeshPoints,std::vector<Vector> &normals,std::vector<std::vector<unsigned int> > &polys,TColorRGBA c, bool edgeok=false)
     {

         int i,nt =polys.size(),nv = MCMeshPoints.size();

          std::vector<GLfloat> vertices;
         std::vector<GLfloat> normalsO;
         std::vector<GLfloat> color;
         std::vector<GLuint> indices;
         vertices.resize(nv * 3);
             normalsO.resize(nv * 3);
             color.resize(nv * 4);

         for(i= 0; i<nv ;i++)
         {
           vertices[3*i] = MCMeshPoints[i].x;
           vertices[3*i+1] = MCMeshPoints[i].y;
           vertices[3*i+2] = MCMeshPoints[i].z;
           normalsO[3*i] = normals[i].x;
           normalsO[3*i+1] = normals[i].y;
           normalsO[3*i+2] = normals[i].z;
           color[4*i]=c.R;
           color[4*i+1]=c.G;
           color[4*i+2]=c.B;
           color[4*i+3]=c.A;
         }

         indices.resize(nt*3);
         for(  i = 0; i <nt; i++){
             int id0, id1, id2;
             id0 = polys[i][0];
             id1 = polys[i][1];
             id2 = polys[i][2];

         indices[3*i]=id0;
         indices[3*i+1]=id1;
         indices[3*i+2]=id2;


         }


             glEnableClientState(GL_VERTEX_ARRAY);
             glEnableClientState(GL_NORMAL_ARRAY);
             glEnableClientState(GL_COLOR_ARRAY);

             glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
             glNormalPointer(GL_FLOAT, 0, &normalsO[0]);
             glColorPointer(4, GL_FLOAT, 0, &color[0]);
             glDrawElements(GL_TRIANGLES, 3*nt, GL_UNSIGNED_INT, &indices[0]);
             glPopMatrix();

         glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
         glDisableClientState(GL_NORMAL_ARRAY);
         glDisableClientState(GL_COLOR_ARRAY);

         //glDisable(GL_BLEND);

         if(edgeok)
         {
             glColor3f(0.4, 0.4, 0.4);



             for(unsigned int i = 0; i < nt; i++){
                 int id0, id1, id2;
                 id0 = polys[i][0];
                 id1 = polys[i][1];
                 id2 = polys[i][2];
                 glBegin(GL_LINE_LOOP);


                 glVertex3f(MCMeshPoints[id0].x,MCMeshPoints[id0].y,MCMeshPoints[id0].z);



                 glVertex3f(MCMeshPoints[id1].x,MCMeshPoints[id1].y,MCMeshPoints[id1].z);



                 glVertex3f(MCMeshPoints[id2].x,MCMeshPoints[id2].y,MCMeshPoints[id2].z);
                 glEnd();

             }


         }

         vertices.clear();
         color.clear();
         normalsO.clear();
         indices.clear();

     }

inline void execute(std::vector<std::vector<Particle>> &v_particles,float h,unsigned int mode =1,unsigned int impf=1,bool savepovFile=false, bool boxok=false)
{
    int i;
    taichi::real radius=2.5;
    SurfaceCells=80;
    if (impf==1)
    {
            radius=3.5;
            SurfaceCells=90;
    }
    if((mode==1)&&(v_hash.size()>0))
    {
        for(i=0;i<v_particles.size();i++)
        {
            v_hash[i].clear();

        }
        v_hash.clear();
        v_bbox.clear();

    }
    init(mode,v_particles,radius*h);

    for(i=v_particles.size()-1;i>-1;i--)
    {
        drawSurface(v_particles[i][0].color,h,v_particles[i],v_hash[i],v_bbox[i],SurfaceCells, mode,impf,savepovFile,boxok);
    }
}


inline void drawAllSurfaceGrid(std::vector<std::vector<Particle>> &v_particles)
{
    int i;
    if (v_bbox.size()>0)
    {
        for(i=0;i<v_bbox.size();i++)
            drawSurfaceGrid(v_particles[i][0].color,v_bbox[i]);
    }
}

inline void drawSurfaceGrid(TColorRGBA c,bounding_box &bbox) {
  float mx = (bbox.bmax.x + bbox.bmin.x)*0.5;
  float my = (bbox.bmax.y + bbox.bmin.y)*0.5;
  float mz = (bbox.bmax.z + bbox.bmin.z)*0.5;
  float maxdimx = (bbox.bmax.x - bbox.bmin.x);
  float maxdimy = (bbox.bmax.y - bbox.bmin.y);
  float maxdimz = (bbox.bmax.z - bbox.bmin.z);
  maxdimx *= 1.5; maxdimy *= 1.5; maxdimz *= 1.5;
  float bx,by,bz,ex,ey,ez;
  bx=(mx-maxdimx*0.5); by=(my-maxdimy*0.5);bz=(mz-maxdimz*0.5);
  ex=(mx+maxdimx*0.5); ey=(my+maxdimy*0.5);ez=(mz+maxdimz*0.5);

  glLineWidth(1);
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINES);
  for(float z=bz;z<=ez;z+=cellsizez)
  {
  for (float x = bx; x <= ex; x += cellsizex) {
    glVertex3f(x, by,z);
    glVertex3f(x, ey,z);
  }
  for (float y = by; y <= ey; y += cellsizey) {
    glVertex3f(bx, y,z);
    glVertex3f(ex, y,z);
  }
  }
  glEnd();



  /*float color;
  glBegin(GL_POINTS);
  for (int x = 0; x <= xsurfacecells; ++x) {
    for (int y = 0; y <= ysurfacecells; ++y) {
      for(int z = 0; z<= zsurfacecells;++z){
          Vector p(bx+x*cellsizex, by+y*cellsizey, bz+z*cellsizez);

          glColor3f(1, 0, 0);
          glVertex3f(p.x, p.y,p.z);
      }
    }
  }

  glEnd();*/
}


  inline int drawSurface(TColorRGBA c,float h,std::vector<Particle> &particles,HashGrid &hash, bounding_box &bbox,int cs, unsigned int mode =1,unsigned int impf=1,bool savepovFile=false, bool boxok=false)  {
    // Do marching tetrahedra, iterate over all cells.
    // If the fluid is "sparse", ie if there are lots of cells and particles in only few of them,
    // it makes sense to do fast marching stating at each particle (but only once per cell).
    //  mode 0 : marching tetrahedra without normals; 1 marching cubes with normals; 2 marching tetrahedra with normals; mode3 : thomas Lewinwer Marching cubes



    switch (mode){

    case 1:
{
        taichi::real mx = (bbox.bmax.x + bbox.bmin.x)*0.5;
        taichi::real my = (bbox.bmax.y + bbox.bmin.y)*0.5;
        taichi::real mz = (bbox.bmax.z + bbox.bmin.z)*0.5;
        taichi::real maxdimx = (bbox.bmax.x - bbox.bmin.x);
        taichi::real maxdimy = (bbox.bmax.y - bbox.bmin.y);
        taichi::real maxdimz = (bbox.bmax.z - bbox.bmin.z);
        maxdimx *= 1.5; maxdimy *= 1.5; maxdimz *= 1.5;
        taichi::real bx,by,bz,ex,ey,ez;
        bx=(mx-maxdimx*0.5); by=(my-maxdimy*0.5);bz=(mz-maxdimz*0.5);
        ex=(mx+maxdimx*0.5); ey=(my+maxdimy*0.5);ez=(mz+maxdimz*0.5);
        xsurfacecells = 1.0*(ex - bx)*cs ;
        ysurfacecells = cs * (ey - by)*1.0 ;
        zsurfacecells = cs * (ez - bz)*1.0   ;
        cellsizex = (ex - bx)/(xsurfacecells);
        cellsizey = (ey - by)/(ysurfacecells);
        cellsizez = (ez - bz)/(zsurfacecells);

         mc.set_resolution( xsurfacecells,ysurfacecells,zsurfacecells,cellsizex,cellsizey,cellsizez,bx,by,bz) ;
         mc.init_all();
        //ofstream dataStream;
     std::vector<Vector> MCMeshPoints;
     std::vector<Vector> normals;

    taichi::real sum,aux;
    value.clear();

   value.resize((xsurfacecells+1) * (ysurfacecells+1)*(zsurfacecells+1));
    std::vector<NeighborData> nbs;
    //nbs.reserve(100);
    for (int x = 0; x <= xsurfacecells; ++x) {
      for (int y = 0; y <= ysurfacecells; ++y) {
        for(int z = 0; z<= zsurfacecells;++z){
        Vector p(bx+x*cellsizex, by+y*cellsizey, bz+z*cellsizez);
        taichi::real &rho = value[z*((ysurfacecells+1)*(xsurfacecells+1))+ y * (xsurfacecells+1) + x];
        rho=-1.0;
        hash.neighbors(p, nbs);

            switch (impf){
            case 1:
              isovalue=0.0;
            rho=phi_z(Vec(p.x,p.y,p.z),nbs,particles,0.4*h);
            //std::cout << "rho = " << rho << std::endl;
            break;
            case 2:
              isovalue=0.0;
            rho=phi_o(hash,p,nbs,particles,2.5*h,h);
            //std::cout << "rho = " << rho << std::endl;
            break;


              }


        // compute density at each point


        //rho = rho;
      }
    }
    }
    //tree.queryRadius(h);

    intersect.auxVector = new std::vector<Vector>;
    intersect.xlo = bx;
    for (int x = 0; x < xsurfacecells; ++x) {
      intersect.ylo = by;
      for (int y = 0; y < ysurfacecells; ++y) {
    intersect.zlo = bz;
    for (int z=0;z< zsurfacecells;++z) {






        intersect.v000 = value[z*((ysurfacecells+1)*(xsurfacecells+1))+y * (xsurfacecells+1) + x] ;
        intersect.v100 = value[z*((ysurfacecells+1)*(xsurfacecells+1))+y * (xsurfacecells+1) + x+1] ;
        intersect.v010 = value[z*((ysurfacecells+1)*(xsurfacecells+1))+(y+1) * (xsurfacecells+1) + x] ;
        intersect.v110 = value[z*((ysurfacecells+1)*(xsurfacecells+1))+(y+1) * (xsurfacecells+1) + x+1] ;
    intersect.v001 = value[(z+1)*((ysurfacecells+1)*(xsurfacecells+1))+y * (xsurfacecells+1) + x] ;
        intersect.v101 = value[(z+1)*((ysurfacecells+1)*(xsurfacecells+1))+y * (xsurfacecells+1) + x+1] ;
        intersect.v011 = value[(z+1)*((ysurfacecells+1)*(xsurfacecells+1))+(y+1) * (xsurfacecells+1) + x] ;
        intersect.v111 = value[(z+1)*((ysurfacecells+1)*(xsurfacecells+1))+(y+1) * (xsurfacecells+1) + x+1] ;

        intersect.xlo = bx+x * cellsizex;
        intersect.xhi = intersect.xlo + cellsizex;
        intersect.ylo = by+y * cellsizey;
        intersect.yhi = intersect.ylo + cellsizey;
    intersect.zlo = bz+z*cellsizez;
        intersect.zhi = intersect.zlo + cellsizez;
        intersect.p000.x = intersect.xlo; intersect.p000.y = intersect.ylo;intersect.p000.z = intersect.zlo;
    intersect.p100.x = intersect.xhi; intersect.p100.y = intersect.ylo;intersect.p100.z = intersect.zlo;
    intersect.p010.x = intersect.xlo; intersect.p010.y = intersect.yhi;intersect.p010.z = intersect.zlo;
    intersect.p110.x = intersect.xhi; intersect.p110.y = intersect.yhi;intersect.p110.z = intersect.zlo;
    intersect.p001.x = intersect.xlo; intersect.p001.y = intersect.ylo;intersect.p001.z = intersect.zhi;
    intersect.p101.x = intersect.xhi; intersect.p101.y = intersect.ylo;intersect.p101.z = intersect.zhi;
    intersect.p011.x = intersect.xlo; intersect.p011.y = intersect.yhi;intersect.p011.z = intersect.zhi;
    intersect.p111.x = intersect.xhi; intersect.p111.y = intersect.yhi;intersect.p111.z = intersect.zhi;
    if (mode==1)
    {
      mc.set_data(value[z*((ysurfacecells+1)*(xsurfacecells+1))+y * (xsurfacecells+1) + x],x,y,z);
    }
    }
      }
    }





        if (mode==1)
        {

      mc.run();
      mc.clean_temps() ;

      mc.CopyPolyhedron(intersect.polys,MCMeshPoints,&normals);
      mc.clean_all() ;


      //if(savepovFile)
       //generatePovRayFile(MCMeshPoints,normals,intersect.polys,boxok);

        DrawMesh(MCMeshPoints,normals,intersect.polys,c,false);
    }


     intersect.auxVector->clear();
     intersect.polys.clear();
     delete intersect.auxVector;
    }
        break;
     case 2:
    {

        SM.setupDephMap(particles,3.0*h,true,false,false,false,false);
    }

}
     return 0;
    //std::cout << "Wmin = " << Wmin << " Wmax = " << Wmax << std::endl;
   }

  };
#endif // VISUALIZER_H
