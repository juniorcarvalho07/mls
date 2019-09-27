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



typedef std::vector<std::vector<NeighborData>> Tneighbors;


struct Intersector {
int type;
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
      int povRayFileNumber;
      MarchingCubesL mc ;
      SpaceMesh SM;
      std::vector<HashGrid> v_hash;
      std::vector<bounding_box> v_bbox;
      std::vector<Tneighbors> v_neighbors;
      bounding_box bbox;
      int xsurfacecells, ysurfacecells,zsurfacecells;
      taichi::real cellsizex,cellsizey,cellsizez;
      taichi::real isovalue,Wmax,Wmin;
      int SurfaceCells;
       std::vector<taichi::real> value;
       std::vector<std::vector<Vector>> MCMeshPoints_vector;
       std::vector<std::vector<Vector>> normals_vector;
       std::vector<Intersector> intersector_vector;
      Visualizer(){
          SurfaceCells=90;
          Wmax=1.5;
          Wmin=0.0;
          povRayFileNumber=0;
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

      inline void generatePovRayFile(std::vector<std::vector<Vector>> &MCMeshPoints,std::vector<std::vector<Vector>> &normals,std::vector<Intersector> &I,bool boxok=false)
      {
        char fname[32];
        int i,j;
        sprintf(fname, "%s_%04d.pov", "povRayScreen",
                this->povRayFileNumber);
        ofstream dataStream;
        dataStream.open(fname, ios::out);
        dataStream << "#version 3.6;\n"
                "global_settings{assumed_gamma 1.0}\n"
                "#default{ finish{ ambient 0.1 diffuse 0.9 }}\n";

        dataStream << "#include \"colors.inc\"\n"
                "#include \"textures.inc\"\n"
                "#include \"woods.inc\"\n";

    //background

        dataStream << "background { color White }\n";

    //camera
        dataStream << "camera{ location   < -0.9, 2.5 ,-5.0>\n"
                "look_at <0.5 , 0.85 , 0.0>\n"
                "right x*image_width/image_height\n"
                "angle 25 }\n";

    // light
        dataStream << "light_source\n"
    "{ <200, 200, -250>/50, 1\n"
      "fade_distance 5 fade_power 2\n"
      "area_light x*3, y*3, 12, 12 circular orient adaptive 0\n"
    "}\n"
    "light_source\n"
    "{ <-500, 250, -150>/50, <1,.8,.4>\n"
      "fade_distance 6 fade_power 2\n"
      "area_light x*3, y*3, 12, 12 circular orient adaptive 0\n"
    "}\n"
    "light_source\n"
    "{ <250, 300, 500>/50, <.3,.8,1>\n"
      "fade_distance 5 fade_power 2\n"
      "area_light x*3, y*3, 12, 12 circular orient adaptive 0\n"
    "}\n";
    // Floor
      dataStream <<"  #declare Floor_Texture =\n"
        "texture { pigment { P_WoodGrain18A color_map { M_Wood18A }}}\n"
        "texture { pigment { P_WoodGrain12A color_map { M_Wood18B }}}\n"
        "texture {\n"
            "pigment { P_WoodGrain12B color_map { M_Wood18B }}\n"
            "finish { reflection 0.3 }\n"
        "}\n";

     dataStream << "#declare T_Glass = texture {   pigment { color red 0.85 green 0.45 blue 0.45 filter 0.95 }\n"
       "finish {\n"
          "ambient 0.0\n"
          "diffuse 0.0\n"
          "reflection 0.3\n"
          "phong 0.3\n"
          "phong_size 90\n"
       "}\n"
    "}\n";

     dataStream << "#declare Tc = texture {\n"
     "pigment{color rgb<184.0/255.0,183.0/255.0,153.0/255.0>}\n"
        "finish { diffuse 0.9 phong 0.0 reflection 0.5} \n"
    "}\n";
    //water


    dataStream << "#declare Skin =\n"
     "texture {\n"
                    "pigment { Yellow }\n"
                    "normal {\n"
                        "wood ramp_wave\n"
                        "slope_map {\n"
                            "[ 0.00 <0,0> ]\n"
                            "[ 0.40 <0.5,1> ]\n"
                            "[ 0.80 <1,0> ]\n"
                            "[ 0.90 <0.1,-1> ]\n"
                            "[ 1.00 <0,0> ]\n"
                        "}\n"
                        "bump_size 0.03\n"
                        "turbulence 0.1\n"
                        "rotate x*-75\n"
                        "translate z*-10\n"
                        "rotate y*30\n"
                        "scale .05\n"
                    "}\n"
                "}\n";


     //blood
    dataStream << "#declare Wine = pigment{color red 1.0 filter 0.65};\n";

    //needle
    //dataStream << "#declare Wine = pigment{color red 0.65 green 0.65 blue 0.95 filter 0.65};\n";

    dataStream << "#declare Liquid = finish { reflection 0.05 }\n";

    dataStream << "#declare PlankNormal =\n"
      "normal\n"
      "{ gradient x 2 slope_map { [0 <0,1>][.05 <1,0>][.95 <1,0>][1 <0,-1>] }\n"
        "scale 0.2\n"
      "};\n";

    dataStream << "#declare C_White_Wine = <221,241,251"
                  ">/255;\n"
        "#declare T_wine =\n"
        "texture {\n"
            "pigment {\n"
                "color rgbf <C_White_Wine.x, C_White_Wine.y, C_White_Wine.z, 0.62>\n"
            "}\n"
            "finish {\n"
                "specular 0.6\n"
                "roughness 0.002\n"
                "ambient 0\n"
                "diffuse 0\n"
                   "reflection {\n"
                        "0.1*C_White_Wine, 0.4*C_White_Wine\n"
                        "fresnel on\n"
                        "metallic 1\n"
                    "}\n"
                    "conserve_energy\n"
               "}\n"
        "}\n";

    dataStream << "plane\n"
    "{ y, -0.006\n"
      "material\n"
      "{ texture{Floor_Texture}}\n"
      //"}\n"
      //"finish { specular .4 reflection .1 }\n"
    "}\n";


    //obstacle


           if(boxok)
           {
         dataStream << "mesh {\n";
         for( i=2;i< malha->getNumberOfCells();i++)
         {
           dataStream << "triangle { <" << malha->getVertex(malha->getCell(i)->getVertexId(0))->getCoord(0) << ", "<< malha->getVertex(malha->getCell(i)->getVertexId(0))->getCoord(1) << ", " << malha->getVertex(malha->getCell(i)->getVertexId(0))->getCoord(2) << ">,\n"
           "<" << malha->getVertex(malha->getCell(i)->getVertexId(1))->getCoord(0) << ", "<< malha->getVertex(malha->getCell(i)->getVertexId(1))->getCoord(1) << ", " << malha->getVertex(malha->getCell(i)->getVertexId(1))->getCoord(2) << ">,\n"
           "<" << malha->getVertex(malha->getCell(i)->getVertexId(2))->getCoord(0) << ", "<< malha->getVertex(malha->getCell(i)->getVertexId(2))->getCoord(1) << ", " << malha->getVertex(malha->getCell(i)->getVertexId(2))->getCoord(2) << ">}\n";
         }

         dataStream << "texture {\n"
                       "pigment {\n"
                       "color rgbt <0.6, 0.6, 0.6, 0.8>\n"
                       "}\n"
        "}\n";
         dataStream << "}\n";
           }



            if(MalhaObst!=NULL)
            {
             dataStream << "mesh2 {\n";

         dataStream << "vertex_vectors {\n";
          dataStream <<  MalhaObst->getNumberOfVertices() << ",\n";

          for( i=0;i< MalhaObst->getNumberOfVertices()-1;i++)
            {


            dataStream << "< " <<MalhaObst->getVertex(i)->getCoord(0) << ", " << MalhaObst->getVertex(i)->getCoord(1) << ", " << MalhaObst->getVertex(i)->getCoord(2) << " >,\n";


        }
        dataStream << "< " <<MalhaObst->getVertex(i)->getCoord(0) << ", " << MalhaObst->getVertex(i)->getCoord(1) << ", " << MalhaObst->getVertex(i)->getCoord(2) << " >\n";
        dataStream << "}\n";

          dataStream << "normal_vectors {\n";
          dataStream <<  MalhaObst->getNumberOfVertices() << ",\n";

          for( i=0;i< MalhaObst->getNumberOfVertices()-1;i++)
            {


            dataStream << "< " <<MalhaObst->getVertex(i)->getNormalCoord(0) << ", " << MalhaObst->getVertex(i)->getNormalCoord(2) << ", " << MalhaObst->getVertex(i)->getNormalCoord(2) << " >,\n";


        }
        dataStream << "< " <<MalhaObst->getVertex(i)->getNormalCoord(0) << ", " << MalhaObst->getVertex(i)->getNormalCoord(2) << ", " << MalhaObst->getVertex(i)->getNormalCoord(2) << " >\n";
        dataStream << "}\n";

        dataStream << "face_indices {\n";
          dataStream <<  MalhaObst->getNumberOfCells() << ",\n";

          for( i=0;i< MalhaObst->getNumberOfCells()-1;i++)
            {


            dataStream << "< " <<MalhaObst->getCell(i)->getVertexId(0) << ", " << MalhaObst->getCell(i)->getVertexId(1) << ", " << MalhaObst->getCell(i)->getVertexId(2) << " >,\n";


        }
        dataStream << "< " <<MalhaObst->getCell(i)->getVertexId(0) << ", " << MalhaObst->getCell(i)->getVertexId(1) << ", " << MalhaObst->getCell(i)->getVertexId(2) << " >\n";
        dataStream << "}\n";



            /*for( i=0;i< MalhaObst->getNumberOfCells();i++)
            {
                    dataStream << "triangle {\n";
                    dataStream << "< " << MalhaObst->getVertex(MalhaObst->getCell(i)->getVertexId(0))->getCoord(0) << ", " << MalhaObst->getVertex(MalhaObst->getCell(i)->getVertexId(0))->getCoord(1) << ", " << MalhaObst->getVertex(MalhaObst->getCell(i)->getVertexId(0))->getCoord(2) << " >,\n";
                    dataStream << "< " << MalhaObst->getVertex(MalhaObst->getCell(i)->getVertexId(1))->getCoord(0) << ", " << MalhaObst->getVertex(MalhaObst->getCell(i)->getVertexId(1))->getCoord(1) << ", " << MalhaObst->getVertex(MalhaObst->getCell(i)->getVertexId(1))->getCoord(2) << " >,\n";
                    dataStream << "< " << MalhaObst->getVertex(MalhaObst->getCell(i)->getVertexId(2))->getCoord(0) << ", " << MalhaObst->getVertex(MalhaObst->getCell(i)->getVertexId(2))->getCoord(1) << ", " << MalhaObst->getVertex(MalhaObst->getCell(i)->getVertexId(2))->getCoord(2) << " > }\n";
            }*/
            dataStream << "texture {\n";
       dataStream << " T_Glass } \n"; //for blood
        //dataStream << " Skin } \n"; //for water

         dataStream << "}\n";
        }



    // fluid
    for (int i =0;i<MCMeshPoints.size();i++)
    {
          dataStream << "mesh2 {\n";

          dataStream << "vertex_vectors {\n";
          dataStream <<  MCMeshPoints[i].size() << ",\n";
          int Msize = MCMeshPoints[i].size();
          for( j=0;j< Msize-1;j++)
            {


            dataStream << "< " <<MCMeshPoints[i][j].x << ", " << MCMeshPoints[i][j].y << ", " << MCMeshPoints[i][j].z << " >,\n";


        }
        dataStream << "< " <<MCMeshPoints[i][j].x << ", " << MCMeshPoints[i][j].y << ", " << MCMeshPoints[i][j].z << " >\n";
        dataStream << "}\n";

          dataStream << "normal_vectors {\n";
          dataStream <<  normals[i].size() << ",\n";

          for( j=0;j< normals[i].size()-1;j++)
            {


            dataStream << "< " <<normals[i][j].x << ", " << normals[i][j].y << ", " << normals[i][j].z << " >,\n";


        }
        dataStream << "< " <<normals[i][j].x << ", " << normals[i][j].y << ", " << normals[i][j].z << " >\n";
        dataStream << "}\n";

        dataStream << "face_indices {\n";
          dataStream <<  I[i].polys.size() << ",\n";

          for( j=0;j< I[i].polys.size()-1;j++)
            {


            dataStream << "< " <<I[i].polys[j][0] << ", " << I[i].polys[j][1] << ", " << I[i].polys[j][2] << " >,\n";


        }
        dataStream << "< " <<I[i].polys[j][0] << ", " << I[i].polys[j][1] << ", " << I[i].polys[j][2] << " >,\n";
        dataStream << "}\n";


switch (I[i].type){

case -1:
       {dataStream << "material{\n";
            dataStream << "texture{\n";
            dataStream << "Wine}\n"; // end of texture T_wine fro water -- Wine for blood

            dataStream << "interior{\n"; // ior 1.33\n";
            dataStream << "fade_power 1001\n";
            dataStream << "fade_distance 0.25\n";
            dataStream << "fade_color <0.7,0.7,1.0>\n";
            dataStream << "caustics 0.16\n";
            dataStream << "}\n"; // end of interior
            dataStream << "}\n"; // end of material

    }
    break;
    case 1:
           {
                dataStream << "material{\n";
                dataStream << "texture{\n";
                dataStream << "Skin}\n"; // end of texture T_wine fro water -- Wine for blood
                dataStream << "}\n";

           }
        break;

    case 0:
           {
                dataStream << "material {\n";
                dataStream << "texture {\n";
                dataStream << "pigment {\n";
                dataStream << "color rgbt <0.8, 0.8, 0.8, 0.9>\n";
                dataStream << "}\n";
                dataStream << "finish {\n";
                dataStream << "ambient 0.0\n";
                dataStream << "diffuse 0.0\n";
                dataStream << "reflection {\n";
                dataStream << "0.0, 1.0\n";
                dataStream << "fresnel on\n";
                dataStream << "}\n";
                dataStream << "specular 0.4\n";
                dataStream << "roughness 0.1\n";
                dataStream << "}\n";
                dataStream << "}\n";
                dataStream << "interior {\n";
                dataStream << "ior 1.3\n";
                dataStream << "}\n";
                dataStream << "}\n";

           }
        break;

    case 2:
           {
                dataStream << "material{\n";
                dataStream << "texture{\n";
                dataStream << "pigment{rgb <1,0,0>}\n";
                dataStream << "finish{\n";
                  dataStream << "conserve_energy\n";
                  dataStream << "diffuse 0.6\n";
                  dataStream << "ambient 0\n";
                  dataStream << "specular 0.5\n";
                  dataStream << "roughness 0.05\n";
                  dataStream << "reflection{0 1 fresnel on metallic 0}\n";
                dataStream << "}\n";
                dataStream << "}\n";
                dataStream << "interior{ior 1.16}\n";
                dataStream << "}\n";

           }
        break;
    }
    dataStream << "}\n"; //end of file
     }

        dataStream.close();
        this->povRayFileNumber++;
  }

inline void execute(std::vector<std::vector<Particle>> &v_particles,float h,unsigned int mode =1,unsigned int impf=1,bool savepovFile=false, bool boxok=false)
{
    int i;
    taichi::real radius=2.5;
    SurfaceCells=90;
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

    if(savepovFile)
        generatePovRayFile(MCMeshPoints_vector,normals_vector,intersector_vector,boxok);
    MCMeshPoints_vector.clear();
    normals_vector.clear();
    intersector_vector.clear();
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
     Intersector intersect;

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
        MCMeshPoints_vector.push_back(MCMeshPoints);
        normals_vector.push_back(normals);
        intersect.type=particles[0].type;
        intersector_vector.push_back(intersect);

    }


     //intersect.auxVector->clear();
     //intersect.polys.clear();
     //delete intersect.auxVector;
    }
        break;
     case 2:
    {

        SM.setupDephMap(particles,1.0*h,true,false,false,false,false);
    }

}
     return 0;
    //std::cout << "Wmin = " << Wmin << " Wmax = " << Wmax << std::endl;
   }

  };
#endif // VISUALIZER_H
