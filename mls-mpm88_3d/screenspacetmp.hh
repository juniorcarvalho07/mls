#ifndef SCREENSPACETMP_HH
#define SCREENSPACETMP_HH

typedef std::pair<int,int> TcP;
#include "vector.hh"
#include "sph.hh"

#include <math.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>

class cell
{
public:
  std::vector<int> pIds;
  std::vector<float> zValues;
};

class SpaceMesh{
public:
  float screenSpace;
  int Nx,Ny,iNx,iNy,Li;
  std::vector<cell> cells;

  std::vector<TcP> cPlist;
  void initCells()
  {
    cPlist.clear();
    cells.clear();
    screenSpace=1;
    iNx=0;
    iNy=0;
    Li=0;
    Nx=(int) (screenW/screenSpace) +1;
    Ny=(int) (screenH/screenSpace) +1;
    int i,j;
    for(i=0;i<Nx;i++)
      for(j=0;j<Ny;j++)
      {
    cell c;
    cells.push_back(c);
      }

  }

  Vector worldScreenSpace(int x, int y) {
    int vp[4];
    double MV[16], P[16];
    static unsigned short tmp_mz;
    static GLfloat mz;
    double ox, oy, oz,
                posX,
                posY,
                posZ;

    glGetIntegerv(GL_VIEWPORT, vp);
    glGetDoublev(GL_MODELVIEW_MATRIX, MV);
    glGetDoublev(GL_PROJECTION_MATRIX, P);
    int realy =vp[3]-y;
    glReadBuffer(GL_FRONT);
    glReadPixels(x, realy,1, 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT,
                            &tmp_mz);

            mz = (float) tmp_mz / 65535.0;  // maximum value for unsigned_short

            gluUnProject(x, realy, mz, MV, P,
                            vp, &posX, &posY, &posZ);

    ox = posX,
    oy = posY;
    oz = posZ;
    oz = 0.0;

    Vector o(ox, oy, oz);

    return o;
  }

  void  MultiM(float M1[16],float M2[16],float res[16])
  {

      int i,j,k;
      for(i=0;i<4;i++)
          for(j=0;j<4;j++)
          {
              res[i*4+j]=0.0;
              for(k=0;k<4;k++)
              {
                  res[i*4+j]+=M1[i*4+k]*M2[k*4+j];
              }
          }
  }

  inline void MultiV(float objx,float objy,float objz,float objw, float M[16],float *camx, float *camy,float *camz,float *camw)
  {
    *camx = M[0]*objx+M[1]*objy+M[2]*objz+M[3]*objw;
    *camy = M[4]*objx+M[5]*objy+M[6]*objz+M[7]*objw;
    *camz = M[8]*objx+M[9]*objy+M[10]*objz+M[11]*objw;
    *camw = M[12]*objx+M[13]*objy+M[14]*objz+M[15]*objw;
  }

  int DrawOneCellWithParticles( SPH &sph)
  {

    Vector v1,v2,v3,v4;
    double *P,*Po;
    P = new double[3];
      if(Li<cPlist.size())
      {
        if(cells[cPlist[Li].first*Nx+cPlist[Li].second].pIds.size()>0)
        {
          glColor4f(0.8, 0.8, 0.8, 0.2);
          glBegin(GL_POLYGON);
          v1= worldScreenSpace(cPlist[Li].first*screenSpace,screenH-cPlist[Li].second*screenSpace);
          v1 = sph.particles[cells[cPlist[Li].first*Nx+cPlist[Li].second].pIds[0]].x;
          v1.z=cells[cPlist[Li].first*Nx+cPlist[Li].second].zValues[0];
        //  P[0]=v1.x; P[1]=v1.y; P[2]=v1.z; Po=auxMatrix::Mult_Matrix_Point(Interactor->matrix,P); v1.x=Po[0];v1.y=Po[1];v1.z=Po[2];
          //delete Po;
        glVertex3f(v1.x,v1.y,v1.z);
        v2= worldScreenSpace((cPlist[Li].first+1)*screenSpace,screenH-cPlist[Li].second*screenSpace);
        v2.z=cells[cPlist[Li].first*Nx+cPlist[Li].second].zValues[0];
        //P[0]=v2.x; P[1]=v2.y; P[2]=v2.z; Po=auxMatrix::Mult_Matrix_Point(Interactor->matrix,P); v2.x=Po[0];v2.y=Po[1];v2.z=Po[2];
        //delete Po;
        glVertex3f(v2.x,v2.y,v2.z);
        v3= worldScreenSpace((cPlist[Li].first+1)*screenSpace,screenH-(cPlist[Li].second+1)*screenSpace);
        v3.z = cells[cPlist[Li].first*Nx+cPlist[Li].second].zValues[0];
        //P[0]=v3.x; P[1]=v3.y; P[2]=v3.z; Po=auxMatrix::Mult_Matrix_Point(Interactor->matrix,P); v3.x=Po[0];v3.y=Po[1];v3.z=Po[2];
        //delete Po;
        glVertex3f(v3.x,v3.y,v3.z);
        v4= worldScreenSpace(cPlist[Li].first*screenSpace,screenH-(cPlist[Li].second+1)*screenSpace);
        v4.z = cells[cPlist[Li].first*Nx+cPlist[Li].second].zValues[0];
        //P[0]=v4.x; P[1]=v4.y; P[2]=v4.z; Po=auxMatrix::Mult_Matrix_Point(Interactor->matrix,P); v4.x=Po[0];v4.y=Po[1];v4.z=Po[2];
        //delete Po;
        glVertex3f(v4.x,v4.y,v4.z);
        glEnd();
        glColor4f(0.0,0.0,0.0,0.5);
        glBegin(GL_LINE_LOOP);
        glVertex3f(v1.x,v1.y,v1.z);
        glVertex3f(v2.x,v2.y,v2.z);
        glVertex3f(v3.x,v3.y,v3.z);
        glVertex3f(v4.x,v4.y,v4.z);
        glEnd();
        Li++;
        delete P;
        return 1;
        }
        Li++;
    }
      delete P;
      return 0;

  }

  void DrawCellsWithParticles()
  {
    int i;
    Vector v1,v2,v3,v4;
    for(i=0;i<cPlist.size();i++)
    {

        if(cells[cPlist[i].first*Nx+cPlist[i].second].pIds.size()>0)
    {
      glColor4f(0.8, 0.8, 0.8, 0.2);
      glBegin(GL_POLYGON);
          v1= worldScreenSpace(cPlist[i].first*screenSpace,screenH-cPlist[i].second*screenSpace);
          v1.z=cells[cPlist[i].first*Nx+cPlist[i].second].zValues[0];
        glVertex3f(v1.x,v1.y,v1.z);
        v2= worldScreenSpace((cPlist[i].first+1)*screenSpace,screenH-cPlist[i].second*screenSpace);
        v2.z=cells[cPlist[i].first*Nx+cPlist[i].second].zValues[0];
        glVertex3f(v2.x,v2.y,v2.z);
        v3= worldScreenSpace((cPlist[i].first+1)*screenSpace,screenH-(cPlist[i].second+1)*screenSpace);
        v3.z = cells[cPlist[i].first*Nx+cPlist[i].second].zValues[0];
        glVertex3f(v3.x,v3.y,v3.z);
        v4= worldScreenSpace(cPlist[i].first*screenSpace,screenH-(cPlist[i].second+1)*screenSpace);
        v4.z = cells[cPlist[i].first*Nx+cPlist[i].second].zValues[0];
        glVertex3f(v4.x,v4.y,v4.z);
        glEnd();
        glColor4f(0.0,0.0,0.0,0.5);
        glBegin(GL_LINE_LOOP);
        glVertex3f(v1.x,v1.y,v1.z);
        glVertex3f(v2.x,v2.y,v2.z);
        glVertex3f(v3.x,v3.y,v3.z);
        glVertex3f(v4.x,v4.y,v4.z);
        glEnd();

        }

    }
  }

  inline int MygluProjectf(float objx, float objy, float objz, float modelview[16], float projection[16], int *viewport, float *wx, float *wy, float *wz, float *ww)
    {
      float in0, in1,in2,in3;
          float out0,out1,out2,out3;

          in0=objx;
          in1=objy;
          in2=objz;
          in3=1.0;
          MultiV(in0,in1,in2,in3,modelview, &out0,&out1,&out2,&out3);
          MultiV(out0,out1,out2,out3,projection, &in0,&in1,&in2,&in3);
          if (in3 == 0.0) return 0;
          *ww=in3;
          in0 /= in3;
          in1 /= in3;
          in2 /= in3;
          /* Map x, y and z to range 0-1 */
          in0 = in0 * 0.5 + 0.5;
          in1 = in1 * 0.5 + 0.5;
          in2 = in2 * 0.5 + 0.5;

          /* Map x,y to viewport */
          in0 = in0 * viewport[2] + viewport[0];
          in1 = in1 * viewport[3] + viewport[1];

          *wx=in0;
          *wy=in1;
          *wz=in2;
            return 1;

    }



  void setupDephMap( SPH &sph)
  {

    initCells();
    float P[16];
    float x,y,z,w,rp,iR,rp2,f;
    float wx,wy,wz;
    float MV[16];
    float MVP[16];
    int vp[4];
    int xp,yp, res;

    glutPostRedisplay();
    glGetIntegerv(GL_VIEWPORT, vp);
    glGetFloatv(GL_MODELVIEW_MATRIX, MV);
    glGetFloatv(GL_PROJECTION_MATRIX, P);
    MultiM(P,MV,MVP);
    int i,j,pi;

    for(pi=0;pi< sph.particles.size();pi++)
    {
    Particle &p =sph.particles[pi];
        res=MygluProjectf(p.x.x,p.x.y,p.x.z,MV,P,vp,&wx,&wy,&wz,&w);
        rp = 1.0*sphereRadius*screenW*sqrt(MVP[0]*MVP[0]+MVP[1]*MVP[1]+MVP[2]*MVP[2])/w;
    rp2=rp*rp;
        //xp=screenW*(0.5+0.5*x/w);
        //yp=screenH*(0.5+0.5*y/w);
        xp =(int)wx;
        yp =(int)wy;

        for(i=xp-1;i<xp+1;i++)
          for(j=yp-1;j<yp+1;j++)
      {

            iR =(float) ((i*screenSpace)-xp)*(i*screenSpace-xp)+(j*screenSpace-yp)*(j*screenSpace-yp);
            if(iR<=(rp2))
        {
              TcP cp;
              cp.first=i;cp.second=j;
              cPlist.push_back(cp);
          f= 1.0 - iR/rp2;
              if(cells[i*Nx+j].zValues.size()>0)
              {
                  if(cells[i*Nx+j].zValues[0] < p.x.z)
                  {
                      cells[i*Nx+j].pIds[0]=pi;
                      cells[i*Nx+j].zValues[0]=p.x.z;
                  }
              }
              else
              {
                  cells[i*Nx+j].pIds.push_back(pi);
                  cells[i*Nx+j].zValues.push_back(p.x.z);
              }
        }
      }
    }
    DrawOneCellWithParticles(sph);
  }


};


#endif // SCREENSPACETMP_HH

