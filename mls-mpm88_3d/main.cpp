#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glew.h>
#include <GL/glut.h>
#endif

#include <stdlib.h>
#include <iostream>

#include "MLS_MPM_fluid.hh"
//#include "spaceMesh.hh"
//#include "BoundaryParticles.hh"
#define         REFRESH_DELAY           1               //ms
using namespace std;
clock_t current_ticks, delta_ticks;
float FPSf;
class Sphere3D
{

protected:
    std::vector<GLfloat> vertices;
    std::vector<GLfloat> normals;
    std::vector<GLfloat> color;
    std::vector<GLushort> indices;
    unsigned int sectors_, rings_;

public:

  void SetColor (float R, float G, float B, float A)
  {
    int r,s;
    std::vector<GLfloat>::iterator c = color.begin();
    for(r = 0; r < rings_; r++) for(s = 0; s < sectors_; s++)
    {
      *c++ = R;
      *c++ = G;
      *c++ = B;
      *c++ = A;
    }
  }

    void SolidSphere(float radius, unsigned int rings, unsigned int sectors)
    {
        sectors_ = sectors;
    rings_ = rings;
        float const R = 1./(float)(rings-1);
        float const S = 1./(float)(sectors-1);
        int r, s;

        vertices.resize(rings * sectors * 3);
        normals.resize(rings * sectors * 3);
        color.resize(rings * sectors * 4);
        std::vector<GLfloat>::iterator v = vertices.begin();
        std::vector<GLfloat>::iterator n = normals.begin();

        for(r = 0; r < rings; r++) for(s = 0; s < sectors; s++) {
                float const y = sin( -M_PI_2 + M_PI * r * R );
                float const x = cos(2*M_PI * s * S) * sin( M_PI * r * R );
                float const z = sin(2*M_PI * s * S) * sin( M_PI * r * R );



                *v++ = x * radius;
                *v++ = y * radius;
                *v++ = z * radius;

                *n++ = x;
                *n++ = y;
                *n++ = z;
        //std::cout <<  " v["<< r*sectors+s << "] = ( " << vertices[r*sectors+s] << " , " << vertices[r*sectors+s+1] << " , " << vertices[r*sectors+s+2] << " )" << std::endl;
        }

        indices.resize(rings * sectors * 4);
        std::vector<GLushort>::iterator i = indices.begin();
         for(r = 0; r < rings-1; r++) for(s = 0; s < sectors-1; s++) {
                *i++ = r * sectors + s;
                *i++ = r * sectors + (s+1);
                *i++ = (r+1) * sectors + (s+1);
                *i++ = (r+1) * sectors + s;
        //std::cout <<  " q["<< r*sectors+s << "] = ( " << indices[r*sectors+s] << " , " << indices[r*sectors+s+1] << " , " << indices[r*sectors+s+2] << " , " << indices[r*sectors+s+3]<< " )" << std::endl;
        }
    }

    void draw(GLfloat x, GLfloat y, GLfloat z)
    {
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glTranslatef(x,y,z);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);

        glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
        glNormalPointer(GL_FLOAT, 0, &normals[0]);
        glColorPointer(4, GL_FLOAT, 0, &color[0]);
        glDrawElements(GL_QUADS, indices.size(), GL_UNSIGNED_SHORT, &indices[0]);
        glPopMatrix();

    glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    }
};

void DrawGround()
{

    static const GLfloat difw[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    static const GLfloat spec[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    static const GLfloat ambi[] = { 0.1f, 0.1f, 0.1f, 1.0f };
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


    glMaterialfv(GL_FRONT, GL_SPECULAR, spec);
    glMaterialfv(GL_FRONT, GL_AMBIENT,  ambi);
    glMaterialf(GL_FRONT, GL_SHININESS, 30);

    glMaterialfv(GL_FRONT, GL_DIFFUSE, difw);
    glPushMatrix();
    //glTranslatef(0, -0.6, 0);
    glColor3fv(difw);
    glNormal3f(0.0, 1.0, 0.0);
    glBegin(GL_POLYGON);
    glVertex3d(-10.0, -0.006, -10.0);
    glVertex3d(-10.0, -0.006,  10.0);
    glVertex3d( 10.0, -0.006,  10.0);
    glVertex3d( 10.0, -0.006, -10.0);
    glEnd();
    glPopMatrix();
};

static int ox, oy;
bool buttondown[3] = {false, false, false};
float   mouse_radius = 0.1;
float mouse_force = 200,normalizedV;
int jump;
Vec mouse_point(std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity());



MLS_MPM mpm;

scrInteractor *Interactor = new scrInteractor(screenW, screenH);

bool running = false, drawgrid = false, drawsurface = false, drawhash = false, drawtree = false, saveImageFile=false, saveFile=false,vcolorok=false,showFPSok=false,screenMeshok=false,savePovFile=false;
Sphere3D pSphere;
int niter=0;
int impf=1;
void showFPS(float fps)
{
    static std::stringstream ss;

    // backup current model-view matrix
   /* glPushMatrix();                     // save current modelview matrix
    glLoadIdentity();                   // reset modelview matrix

    // set to 2D orthogonal projection
    glMatrixMode(GL_PROJECTION);        // switch to projection matrix
    glPushMatrix();                     // save current projection matrix
    glLoadIdentity();                   // reset projection matrix
    gluOrtho2D(0, 400, 0, 300);         // set to orthogonal projection*/

   // float color[4] = {0, 0, 0, 1};

    // update fps every second
    ss.str("");
    ss << std::fixed << std::setprecision(1);
    ss << fps << " FPS" << std::ends; // update fps string
    //ss << std::resetiosflags(std::ios_base::fixed | std::ios_base::floatfield);
    glRasterPos2f(1.7, 1.8);
    //drawString(ss.str().c_str(), 700, 500, color, font);
    Interactor->Draw_String(ss.str().c_str(),1.0f-Interactor->getBGColor(0),1.0f-Interactor->getBGColor(1),1.0f-Interactor->getBGColor(2));
    //drawString(report[0].c_str(), 15, 273, color, font);
    //drawString(report[1].c_str(), 15, 260, color, font);

    // restore projection matrix
    /*glPopMatrix();                      // restore to previous projection matrix

    // restore modelview matrix
    glMatrixMode(GL_MODELVIEW);         // switch to modelview matrix
    glPopMatrix();*/                      // restore to previous modelview matrix
}
void RenderScene(void)
{
  allCommands->Execute();





if((!drawsurface) && (!screenMeshok))
{
TColorRGBA caux;
    pSphere.SolidSphere(dx*0.5,4,4);
  glClearColor(1.0, 1.0, 1.0, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
/*  glBegin(GL_QUADS);
  glColor3f(0, 0, 0.2);
  glVertex2f(0, 0);
  glVertex2f(0, 10);
  glVertex2f(1.75, 10);
  glVertex2f(1.75, 0);
  glEnd();*/
  
    pSphere.SetColor(0.0,0.0,1.0,1.0);
      //glColor3f(1, 1, 0);
      glPointSize(5);
     // glEnable(GL_POINT_SMOOTH);
      //glBegin(GL_POINTS);

      glEnable(GL_BLEND);
          glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      // draw particles
      for (int i = 0; i < mpm.n_particles(); i++)
        for (int j=0;j<mpm.v_particles[i].size();j++){
        Vec const &p = mpm.v_particles[i][j].x;
        Vector vi(mpm.v_particles[i][j].v.x,mpm.v_particles[i][j].v.y,mpm.v_particles[i][j].v.z);
       // glVertex2f(p.x, p.y);
        caux.R=mpm.v_particles[i][j].color.R;
         caux.G=mpm.v_particles[i][j].color.G;
         caux.B=mpm.v_particles[i][j].color.B;
         caux.A=mpm.v_particles[i][j].color.A;
     if(vcolorok)
     {
             Interactor->ShowLookupTable(true,mpm.NormMinV,mpm.NormMaxV,"Velocity",mpm.Acc_total_time);
          normalizedV = 2.0*((vi.norm()-mpm.NormMinV)/(mpm.NormMaxV-mpm.NormMinV))-1.0;
          //std::cout << "NormMinA = " << sph.NormMinA << " NormMaxA = " << sph.NormMaxA << " ai.norm() = "<< ai.norm() << std::endl;
           jump =floor((1.0+normalizedV)/0.1);
           //std::cout << "normalizedV = " << normalizedV << " jump = " << jump << std::endl;
           if(normalizedV<0.0)
           {
                   caux.R=(float)(15)/255;;
                   caux.G = (float)(15+12*jump)/255;
                   caux.B = (float)(255-12*jump)/255;
           caux.A=1.0;

           }
           else
           {
                   caux.R = (float)(15+12*(jump))/255;
                   caux.G =(float)(255- 12*(jump))/255;
                   caux.B=(float)(15)/255;
           caux.A=1.0;
           }
     }
        pSphere.SetColor(caux.R,caux.G,caux.B,caux.A);
     //glColor3f(caux.R,caux.G,caux.B);
     //glVertex3f(p.x, p.y,p.z);
        pSphere.draw(p.x, p.y,p.z);
      }
 glDisable(GL_BLEND);
  // glEnd();
    
}
    
    


  // if requested, draw search data structures
  if (drawhash) {
   // mpm.hashgrid().draw(mouse_point);
  }
  
  if (drawtree) {
    //mpm.kdtree().draw(mouse_point);
  }
  
  if (drawgrid) {
    mpm.Vis.drawAllSurfaceGrid(mpm.v_particles);
  }
  
  if (drawsurface) {
      glEnable(GL_BLEND);
          glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    mpm.Vis.execute(mpm.v_particles,1.0*dx,1,impf,savePovFile,true);
    glDisable(GL_BLEND);
  }
  if (screenMeshok) {
      glEnable(GL_BLEND);
          glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    mpm.Vis.execute(mpm.v_particles,1.0*dx,2,impf);
    glDisable(GL_BLEND);
  }
  



  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  Print->Face(0,whiteo);
  Print->Face(1,whiteo);
 Print->Face(2,whiteo);
 Print->Face(3,whiteo);
 //Print->Face(4,whiteo);
 //Print->Face(5,whiteo);
 Print->Face(6,whiteo);
 Print->Face(7,whiteo);
 Print->Face(8,whiteo);
 Print->Face(9,whiteo);
 //DrawGround();

 glDisable(GL_BLEND);

 if(MalhaObst!=NULL)
 {
     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
     PrintObst->SmoothFaces(redb);
     PrintObst->Edges(lgrey);
     glDisable(GL_BLEND);
 }


if(showFPSok)
    showFPS(FPSf);




  glFinish();
  glutSwapBuffers();
}  

void reshape(int width, int height)
{
  glViewport(0, 0, width, height);
  mpm.winWidth = width;
  mpm.winHeight = height;
  
  glMatrixMode(GL_PROJECTION);
  
  glLoadIdentity();

  double r = (float)width/height;
  double xw = 1.2 * r;
  double yw = 1.2 / r;
  
  if (width < height)
    glOrtho(-.1, 1.1, -(yw-1)/2, (yw+1)/2, -1, 1);
  else
    glOrtho(-(xw-1)/2, (xw+2.5)/2, -.1, 1.5, -1.5, 1.5);
    
  glMatrixMode(GL_MODELVIEW);  
  
  glutPostRedisplay();
}

Vec worldSpace(int x, int y) {
  int vp[4];
  double MV[16], P[16];
  double ox, oy, oz;
  glGetIntegerv(GL_VIEWPORT, vp);
  glGetDoublev(GL_MODELVIEW_MATRIX, MV);
  glGetDoublev(GL_PROJECTION_MATRIX, P);
  gluUnProject(x, mpm.winHeight-y, 0, MV, P, vp, &ox, &oy, &oz);
  
  oz = 0;
  Vec o(ox, oy,oz);

  return o;
}

void HandleKeyboard(unsigned char key, int x, int y){



    double coords[3];
    char *xs[10];
    allCommands->Keyboard(key);

  switch(key) {
    case '\033':
    case 'q':
      exit(0);
    case ' ':
      running = !running;
      break;

  case 'i':
      Interactor->WriteScreenImage(mpm.Acc_total_time);
      break;
    case 'S':
  {
      screenMeshok=!screenMeshok;
      drawsurface=false;
  }
      break;
    case 'G':
      drawgrid = !drawgrid;
      break;
    case 'f':
      showFPSok = !showFPSok;
      break;
    case 'x':
      saveImageFile = !saveImageFile;
    case 'Z':
  {
      drawsurface = !drawsurface;
      screenMeshok=false;
      impf=1;
  }
      break;
   case 'O':
  {
      drawsurface = !drawsurface;
      screenMeshok=false;
      impf=2;
  }
      break;
  case 'v':
      vcolorok = !vcolorok;
      break;
    case 'p':
      savePovFile = !savePovFile;
      break;
    case 'r':
      // reset
      //mpm.init_sphere( 0.02,0.045);
      break;
    case '+':
      //mpm.smoothing_radius(1.1 * mpm.smoothing_radius());
      break;
    case '-':
      //mpm.smoothing_radius(1./1.1 * mpm.smoothing_radius());
      break;
  case '.':
   mpm.Vis.SM.screenSpace+=0.5;
   break;
 case ',':
   mpm.Vis.SM.screenSpace-=0.5;
   break;
    case '0':
    case '1': 
    case '2': 
    case '3': 
    case '4': 
    case '5': 
    case '6': 
    case '7': 
    case '8': 
    case '9': 
    {
      int set = key-'1';
      if (set < 0)
        set = 9;
      // set preset parameter values
      //std::cout << "setting parameter set " << set << ": " << parameters[set].name << std::endl;
      //mpm.parameters(parameters[set].params);
      break;
    }
  }
  Interactor->Refresh_List();
  glutPostRedisplay();
}

void timerEvent(int value) {


        if (running) {

          //glutPostRedisplay();
          //if (saveFile)
             //mpm.surfaceCells(mcRes);
            current_ticks = clock();


          mpm.step();
            niter++;

           if((saveImageFile)&&((niter % int(frame_dt / dt) == 0)))
          {

             mpm.Acc_time=0.0;
           // mpm.surfaceCells(125);
            Interactor->WriteScreenImage(mpm.Acc_total_time);
          }
           if((niter % int(frame_dt / dt) == 0))
           {
                delta_ticks = clock() - current_ticks; //the time, in ms, that took to render the scene
               if(delta_ticks > 0)
               {
                   FPSf = CLOCKS_PER_SEC / delta_ticks;
                   //showFPS(FPSf);
               }
              Interactor->Refresh_List();
              glutPostRedisplay();
           }
          }

glutTimerFunc(REFRESH_DELAY, timerEvent, 0);
}


void idle()
{

    if (running) {
    mpm.step();
    if(saveImageFile)
    {
     // mpm.surfaceCells(50);
      mpm.WriteScreenImage();
    }
    glutPostRedisplay();
  }
}

void constructBox()
{
  double vertex[3];
  int cell[3];
  //face frontal

  vertex[0]=-0.006;vertex[1]=-0.006;vertex[2]=-0.006;
  malha ->addVertex(vertex);
  vertex[0]=1.506;vertex[1]=-0.006;vertex[2]=-0.006;
  malha ->addVertex(vertex);
  vertex[0]=1.506;vertex[1]=1.506;vertex[2]=-0.006;
  malha ->addVertex(vertex);
  vertex[0]=-0.006;vertex[1]=1.506;vertex[2]=-0.006;
  malha ->addVertex(vertex);

  //chao

  vertex[0]=-0.006;vertex[1]=-0.006;vertex[2]=1.006;
  malha ->addVertex(vertex);
  vertex[0]=1.506;vertex[1]=-0.006;vertex[2]=1.006;
  malha ->addVertex(vertex);
  cell[0]=0;cell[1]=4;cell[2]=1;
  malha->addCell(cell);
  cell[0]=1;cell[1]=4;cell[2]=5 ;
  malha->addCell(cell);

//traz

  vertex[0]=-0.006;vertex[1]=1.506;vertex[2]=1.006;
  malha ->addVertex(vertex);
  vertex[0]=1.506;vertex[1]=1.506;vertex[2]=1.006;
  malha ->addVertex(vertex);
  cell[0]=6;cell[1]=5;cell[2]=4;
  malha->addCell(cell);
  cell[0]=6;cell[1]=7;cell[2]=5 ;
  malha->addCell(cell);

  //frente
  cell[0]=0;cell[1]=1;cell[2]=3;
  malha->addCell(cell);
  cell[0]=3;cell[1]=1;cell[2]=2 ;
  malha->addCell(cell);


//lateral esquerda


  cell[0]=4;cell[1]=0;cell[2]=6;
  malha->addCell(cell);
  cell[0]=6;cell[1]=0;cell[2]=3 ;
  malha->addCell(cell);


  //lateral direita


  cell[0]=1;cell[1]=5;cell[2]=2;
  malha->addCell(cell);
  cell[0]=2;cell[1]=5;cell[2]=7 ;
  malha->addCell(cell);

}



int main(int argc, char ** argv)
{

    Interactor->setDraw(RenderScene);
    glutInit(&argc, argv);




    malha = new TMesh();

      meshHandler.Set(malha);

    Print = new TPrintOf(meshHandler);

   constructBox();
   //std::cout<< std::endl<< "aqui"<<std::endl<<std::endl;
  MalhaObst = new TMesh();
   if(MalhaObst!=NULL)
   {
    //Reader.read(MalhaObst,"/home/helton/mls/mls-mpm88_3d/off/splineit2corteTri.off");
        //Reader.read(MalhaObst,"/home/helton/Gdrive/mpm/off/splineit2corteTriRef.off");
     Reader.read(MalhaObst,"/home/helton/SPH/mls/mls-mpm88_3d/off/mushroom.off");
     //Reader.read(MalhaObst,"/home/helton/Gdrive/mpm/off/splineit2s.off");
     //Reader.read(MalhaObst,"/home/helton/Gdrive/mpm/off/bunny200Points1.off");
      // Reader.read(MalhaObst,"/home/helton/Dropbox/mpm/off/cross.off");
     //Reader.read(MalhaObst,"/home/helton/Dropbox/mpm/off/elephant500Points.off");
     //Reader.read(MalhaObst,"/home/helton/projetos/mls-mpm88_3d/off/cup.off");
     //Reader.read(MalhaObst,"/home/helton/Dropbox/mpm/off/needlecorte.off");
     //Reader.read(MalhaObst,"/home/helton/Gdrive/mpm/off/cup2.off");
     //Reader.read(MalhaObst,"/home/helton/projetos/codigo3D/off/bunny200Points1.off");
     //Writer.write(MalhaObst,"/home/helton/Dropbox/mpm/off/needle05_a.off");
      meshHandlerObst.Set(MalhaObst);
      PrintObst = new TPrintOf(meshHandlerObst);

   }
  allCommands = new TMyCommands(Print, Interactor);
  double a,x1,x2,y1,y2,z1,z2;

        of::ofVerticesIterator<TTraits> iv(&meshHandler);

        iv.initialize();
        x1 = x2 = iv->getCoord(0);
        y1 = y2 = iv->getCoord(1);
        z1 = z2 = iv->getCoord(2);

        for(iv.initialize(); iv.notFinish(); ++iv){
                if(iv->getCoord(0) < x1) x1 = a = iv->getCoord(0);
                if(iv->getCoord(0) > x2) x2 = a = iv->getCoord(0);
                if(iv->getCoord(1) < y1) y1 = a = iv->getCoord(1);
                if(iv->getCoord(1) > y2) y2 = a = iv->getCoord(1);
                if(iv->getCoord(2) < z1) z1 = a = iv->getCoord(2);
                if(iv->getCoord(2) > z2) z2 = a = iv->getCoord(2);
        }

        float maxdim;
        maxdim = fabs(x2 - x1);
        if(maxdim < fabs(y2 - y1)) maxdim = fabs(y2 - y1);
        if(maxdim < fabs(z2 - z1)) maxdim = fabs(z2 - z1);

        maxdim *= 1.2;

        Point center((x1+x2)/2.0, (y1+y2)/2.0, (z1+z2)/2.0 );
        Interactor->Init(center[0]-maxdim, center[0]+maxdim,
                                        center[1]-maxdim, center[1]+maxdim,
                                        center[2]-maxdim, center[2]+maxdim);

        if(MalhaObst!=NULL)
        {
        of::ofVerticesIterator<TTraits> ivo(&meshHandlerObst);

        ivo.initialize();
        x1 = x2 = ivo->getCoord(0);
        y1 = y2 = ivo->getCoord(1);
        z1 = z2 = ivo->getCoord(2);

        for(ivo.initialize(); ivo.notFinish(); ++ivo){
                if(ivo->getCoord(0) < x1) x1 = a = ivo->getCoord(0);
                if(ivo->getCoord(0) > x2) x2 = a = ivo->getCoord(0);
                if(ivo->getCoord(1) < y1) y1 = a = ivo->getCoord(1);
                if(ivo->getCoord(1) > y2) {
                 y2 = a = ivo->getCoord(1);
                 Vymax = &ivo;
                }
                if(ivo->getCoord(2) < z1) z1 = a = ivo->getCoord(2);
                if(ivo->getCoord(2) > z2) z2 = a = ivo->getCoord(2);
        }

        Point centerObst((x1+x2)/2.0, (y1+y2)/2.0, (z1+z2)/2.0 );
        maxdim = fabs(x2 - x1);
        if(maxdim < fabs(y2 - y1)) maxdim = fabs(y2 - y1);
        if(maxdim < fabs(z2 - z1)) maxdim = fabs(z2 - z1);
        for(ivo.initialize(); ivo.notFinish(); ++ivo){
          ivo->setCoord(0,(ivo->getCoord(0)/(maxdim*1.5))); // 0.7 needle  1.25 othewise
          ivo->setCoord(1,(ivo->getCoord(1)/(maxdim*1.5)));
          ivo->setCoord(2,(ivo->getCoord(2)/(maxdim*1.5))); //0.7 needle 1.25 othewise
        }
        ivo.initialize();
        x1 = x2 = ivo->getCoord(0);
        y1 = y2 = ivo->getCoord(1);
        z1 = z2 = ivo->getCoord(2);

        for(ivo.initialize(); ivo.notFinish(); ++ivo){
                if(ivo->getCoord(0) < x1) x1 = a = ivo->getCoord(0);
                if(ivo->getCoord(0) > x2) x2 = a = ivo->getCoord(0);
                if(ivo->getCoord(1) < y1) y1 = a = ivo->getCoord(1);
                if(ivo->getCoord(1) > y2) y2 = a = ivo->getCoord(1);
                if(ivo->getCoord(2) < z1) z1 = a = ivo->getCoord(2);
                if(ivo->getCoord(2) > z2) z2 = a = ivo->getCoord(2);
        }
        centerObst[0]=(x1+x2)/2.0; centerObst[1]=(y1+y2)/2.0; centerObst[2]=(z1+z2)/2.0;
        maxdim = fabs(x2 - x1);
        if(maxdim < fabs(y2 - y1)) maxdim = fabs(y2 - y1);
        if(maxdim < fabs(z2 - z1)) maxdim = fabs(z2 - z1);

        std::cout<< std::endl<< "center[0]= "<< center[0] << " center[1]= "<< center[1]<< " center[2]= "<< center[2] << std::endl<<std::endl;

        std::cout<< std::endl<< "centerObst[0]= "<< centerObst[0] << " centerObst[1]= "<< centerObst[1]<< " centerObst[2]= "<< centerObst[2] << std::endl<<std::endl;

        std:: cout << " x1 = " << x1 << " x2 = " << x2 << " maxdim = " << maxdim <<  std::endl;
        std:: cout << " y1 = " << y1 << " y2 = " << y2 << " maxdim = " << maxdim <<  std::endl;
        std:: cout << " z1 = " << z1 << " z2 = " << z2 << " maxdim = " << maxdim <<  std::endl;


        for(ivo.initialize(); ivo.notFinish(); ++ivo){
          ivo->setCoord(0,(ivo->getCoord(0)+(center[0]-centerObst[0])));
          ivo->setCoord(1,(ivo->getCoord(1)+(center[1]-centerObst[1])-0.5)); // -0.1 vaso -0.6 bunny -0.25 cup 0.0 cup2 +0.1 needle  0.0 cross
          ivo->setCoord(2,(ivo->getCoord(2)+(center[2]-centerObst[2])));
        }
        //Writer.write(MalhaObst,"/home/helton/Dropbox/mpm/off/needleref_scale.off");
        }
       //mpm.init_sphere( 0.02,0.02);

  //mpm.init_coord_sphere( 0.02,0.1);
    //

        //mpm.init(Vec(0.0+0.15,0.0+0.5,0.0+0.15),Vec(0.0+0.5,0.0+1.0,0.0+0.85),0.0,0.025,0); //Dambreak
        //mpm.init_sphere(Vec(0.25,0.3,0.35),Vec(0.40, 0.8, 0.65), 0.02,0.0225,1);
        mpm.init_sphere(Vec(0.9,0.55,0.25),Vec(1.12, 1.3, 0.80), 0.02,0.02,2);
        //mpm.init_sphere(Vec(0.25,0.8,0.35),Vec(0.4, 1.1, 0.65), 0.02,0.0225,1);
        //mpm.init_sphere(Vec(0.65,0.8,0.35),Vec(0.8, 1.1, 0.65), 0.02,0.0225,2);
        //mpm.init_sphere(Vec(0.7,0.8,0.35),Vec(0.92, 1.4, 0.65), 0.02,0.0225,0);
        //mpm.init(Vec(0.0+0.67,0.0+0.5,0.0+0.35),Vec(0.0+0.97,0.0+1.4,0.0+0.65),0.0,0.02,0);
        //mpm.init(Vector(meshHandlerObst->getVertex(Vymax)->getCoord(0)+0.01,meshHandlerObst->getVertex(Vymax)->getCoord(1)-0.15,meshHandlerObst->getVertex(Vymax)->getCoord(2)+0.0055),Vector(meshHandlerObst->getVertex(Vymax)->getCoord(0)+0.045,meshHandlerObst->getVertex(Vymax)->getCoord(1)-0.02,meshHandlerObst->getVertex(Vymax)->getCoord(2)+0.055),0.02,0.006); //simpleit3corte
        //mpm.init(Vec(meshHandlerObst->getVertex(Vymax)->getCoord(0)+0.055,meshHandlerObst->getVertex(Vymax)->getCoord(1)-0.155,meshHandlerObst->getVertex(Vymax)->getCoord(2)-0.0555),Vec(meshHandlerObst->getVertex(Vymax)->getCoord(0)+0.165,meshHandlerObst->getVertex(Vymax)->getCoord(1)+0.45,meshHandlerObst->getVertex(Vymax)->getCoord(2)+0.015),0.02,0.01,-1); // splineit2corteTri
        //mpm.init(Vector(meshHandlerObst->getVertex(Vymax)->getCoord(0)+0.015,meshHandlerObst->getVertex(Vymax)->getCoord(1)-0.11,meshHandlerObst->getVertex(Vymax)->getCoord(2)-0.0255),Vector(meshHandlerObst->getVertex(Vymax)->getCoord(0)+0.05,meshHandlerObst->getVertex(Vymax)->getCoord(1)-0.04,meshHandlerObst->getVertex(Vymax)->getCoord(2)+0.0125),0.02,0.005); // splineit3corteTri
        //mpm.init(Vector(meshHandlerObst->getVertex(Vymax)->getCoord(0)-0.00,meshHandlerObst->getVertex(Vymax)->getCoord(1)-0.10,meshHandlerObst->getVertex(Vymax)->getCoord(2)-0.1),Vector(meshHandlerObst->getVertex(Vymax)->getCoord(0)+0.09,meshHandlerObst->getVertex(Vymax)->getCoord(1)-0.05,meshHandlerObst->getVertex(Vymax)->getCoord(2)-0.04),0.02,0.004);//needle
        //mpm.init(Vector(0.0-0.13,0.0+0.2,0.0-0.08),Vector(0.0+0.15,0.0+2.5,0.0+0.08),0.02,0.0261); //cup 2
        //mpm.init(Vector(0.0-0.13,0.0+0.7,0.0-0.08),Vector(0.0+0.15,0.0+2.5,0.0+0.08),0.02,0.0325); //cup 1
    //mpm.init(Vector(0.0-0.2,0.0+0.2,0.0-0.1),Vector(0.0+0.2,0.0+2.5,0.0+0.1),0.02,0.028); //cup 1
    //mpm.init("/home/helton/Dropbox/sph/off/splineit3s_scale3dPoints_6k.txt");
    //mpm.init("/home/helton/Dropbox/sph/off/splineit2s_scale3dPoints_8k.txt");
    //mpm.init("/home/helton/Dropbox/sph/off/needle_scale_points_4k.txt");
    of::ofCellsIterator<TTraits> ico(&meshHandler);
            for(ico.initialize(); ico.notFinish(); ++ico)
            {
             mpm.ComputeTriangleMeshNormal(malha,&ico);
        }
    if(MalhaObst!=NULL)
    {
      of::ofVerticesIterator<TTraits> ivo(&meshHandlerObst);
        int cc=0;

        of::ofCellsIterator<TTraits> ico1(&meshHandlerObst);
            for(ico1.initialize(); ico1.notFinish(); ++ico1)
            {
             mpm.ComputeTriangleMeshNormal(MalhaObst,&ico1,1);
             //if(cc==1)
              //std::cout << " n[0] = " << ico->getNormalCoord(0) << " n[1] = " << ico->getNormalCoord(1) << " n[2] = " << ico->getNormalCoord(2) << std::endl;
             cc++;

            }
           // Vector nv;
            for(ivo.initialize(); ivo.notFinish(); ++ivo){
          //nv.x=ivo->getNormalCoord(0);nv.y=ivo->getNormalCoord(1);nv.z=ivo->getNormalCoord(2);
         // nv*=(1/ivo->getNumberofcells());
          //nv.normalize();
          ivo->setNormalCoord(0,ivo->getNormalCoord(0)/ivo->getNumberofcells());ivo->setNormalCoord(1,ivo->getNormalCoord(1)/ivo->getNumberofcells());ivo->setNormalCoord(2,ivo->getNormalCoord(2)/ivo->getNumberofcells());
        }

        PrintObst->SetOpenglObjects(redb);
        mpm.FillGridObst(MalhaObst);

        }




 AddKeyboard(HandleKeyboard);
 //allCommands->Help(std::cout);
std::cout<< std::endl<< "Press \"?\" key for help"<<std::endl<<std::endl;

  glutTimerFunc(REFRESH_DELAY, timerEvent, 0);
  Init_Interactor();
 Interactor->Refresh_List();

  return 0;
}


