#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdlib.h>
#include <iostream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>
#include "MLS_MPM_fluid.hh"

using namespace std;
clock_t current_ticks, delta_ticks;

static int ox, oy;
bool buttondown[3] = {false, false, false};
float   mouse_radius = 0.1;
float mouse_force = 200;
Vec mouse_point(std::numeric_limits<double>::infinity(), 
                   std::numeric_limits<double>::infinity());
std::vector<NeighborData> selected;
int niter=0;

MLS_MPM mpm;

bool running = false, drawgrid = false, drawsurface = false, drawhash = false, drawtree = false, saveImageFile=false,showFPSok=false;

float FPSf;
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
    glRasterPos2f(1.3, 1.4);
    //drawString(ss.str().c_str(), 700, 500, color, font);
    glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
     glDisable(GL_LIGHTING);
   glColor3f(1,1,1);
   for (unsigned int i = 0; i < strlen(ss.str().c_str()); i++)
         glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, ss.str().c_str()[i]);

    glEnable(GL_LIGHTING);
     glPopAttrib();

    //Interactor->Draw_String(ss.str().c_str(),1.0f-Interactor->getBGColor(0),1.0f-Interactor->getBGColor(1),1.0f-Interactor->getBGColor(2));
    //drawString(report[0].c_str(), 15, 273, color, font);
    //drawString(report[1].c_str(), 15, 260, color, font);

    // restore projection matrix
    /*glPopMatrix();                      // restore to previous projection matrix

    // restore modelview matrix
    glMatrixMode(GL_MODELVIEW);         // switch to modelview matrix
    glPopMatrix();*/                      // restore to previous modelview matrix
}

void redraw(void)
{
  glClearColor(1.0, 1.0, 1.0, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_QUADS);
  glColor3f(0, 0, 0.3);
  glVertex2f(0, 0);
  glVertex2f(0, 1.75);
  glVertex2f(1.75, 1.75);
  glVertex2f(1.75, 0);
  glEnd();

glLineWidth(3);
  glBegin(GL_LINE_STRIP);
  glColor3f(0.6, 0.2, 0.0);
  glVertex2f(0.06, 0.06);
  glVertex2f(1.69, 0.06);
  glVertex2f(1.69, 1.69);
  glVertex2f(0.06, 1.69);
  glVertex2f(0.06, 0.06);

  glEnd();
  

      glPointSize(5);
      glEnable(GL_POINT_SMOOTH);
      glBegin(GL_POINTS);
      // draw particles

      for (int i = 0; i < mpm.n_particles(); ++i) {
        Vec const &p = mpm.particle(i).x;
        glColor3f(mpm.particle(i).color.R, mpm.particle(i).color.G, mpm.particle(i).color.B);
        glVertex2f(p.x, p.y);
      }
   
   glEnd();
    
   
    
    
  glPointSize(10);
      glEnable(GL_POINT_SMOOTH);
      glBegin(GL_POINTS);
  // selected particles in red
  glColor3f(1, 0, 0);
  for (int i = 0; i < selected.size(); ++i) {
    Vec const &p = mpm.particle(selected[i].idx).x;
    glVertex2f(p.x, p.y);
  }
  glEnd();
 
  // if requested, draw search data structures
  if (drawhash) {
   // mpm.hashgrid().draw(mouse_point);
  }
  
  if (drawtree) {
    //mpm.kdtree().draw(mouse_point);
  }
  
  if (drawgrid) {
    //mpm.drawSurfaceGrid();
  }
  
  if (showFPSok) {
    showFPS(FPSf);
  }
  
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
  Vec o(ox, oy);

  return o;
}

void keyboard(unsigned char key, int x, int y)
{
  switch(key) {
    case '\033':
    case 'q':
      exit(0);
    case ' ':
      running = !running;
      break;
    case 't':
      drawtree = !drawtree;
      break;
    case 'h':
      drawhash = !drawhash;
      break;
    case 's':
      drawsurface = !drawsurface;
      break;
    case 'f':
      showFPSok = !showFPSok;
    case 'g':
      drawgrid = !drawgrid;
      break;
  case 'x':
    saveImageFile = !saveImageFile;
    break;
    case 'r':
      // reset
      //mpm.init();
      break;
    case '+':
      //mpm.smoothing_radius(1.1 * mpm.smoothing_radius());
      break;
    case '-':
      //mpm.smoothing_radius(1./1.1 * mpm.smoothing_radius());
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
  glutPostRedisplay();
}

void motion(int x, int y)
{
  Vec point = worldSpace(x,y);

  if(buttondown[GLUT_LEFT_BUTTON])
  { 
    mpm.tree.neighbors(Vector(point.x,point.y), selected);

    // drag fluid
    for (int i = 0; i < selected.size(); ++i) {
      Particle &p = mpm.particle(selected[i].idx);

      float k =0.2;
      p.v += mouse_force * k * (point - mouse_point);
    }
  }
  else if (buttondown[GLUT_RIGHT_BUTTON])
  {
  }
  
  mouse_point = point;
  glutPostRedisplay();
}

void button(int b, int state, int x, int y)
{
  ox = x;
  oy = y;
  
  // collect neighhoring particles
  if (state == GLUT_DOWN) {
    mouse_point = worldSpace(x,y);
    mpm.tree.neighbors(Vector(mouse_point.x,mouse_point.y), selected);
     buttondown[b] = true;
  } else {
    selected.clear();
    buttondown[b] = false;
  }
  
  glutPostRedisplay();
}

void idle()
{

    if (running) {
        current_ticks = clock();
    mpm.step();


    niter++;
    if((saveImageFile)&&((niter % int(frame_dt / dt) == 0)))
    {
     // mpm.surfaceCells(50);
      mpm.WriteScreenImage();
    }
    if (niter % int(frame_dt / dt) == 0)
    {


        glutPostRedisplay();
        delta_ticks = clock() - current_ticks; //the time, in ms, that took to render the scene
        if(delta_ticks > 0)
        {
            FPSf = CLOCKS_PER_SEC / delta_ticks;
            //showFPS(FPSf);
        }
    }
  }
}

int main(int argc, char ** argv) 
{
  glutInitWindowSize(mpm.winWidth, mpm.winHeight);
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA | GLUT_DOUBLE);
  glutCreateWindow("mpm");
mpm.init(Vec(1.05,0.8), Vec(1.25, 1.0),0);
mpm.init(Vec(1.35,0.8), Vec(1.55, 1.0),1);
  mpm.init(Vec(0.25,0.8), Vec(0.75, 1.4));
  mpm.initSearch();

  // GL init
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  glutDisplayFunc(redraw);
  glutMotionFunc(motion);
  glutPassiveMotionFunc(motion);
  glutMouseFunc(button);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);
  
  glutMainLoop();

  /*
  GUI gui("MLS-MPM", mpm.winWidth, mpm.winHeight);
  mpm.init();
  auto &canvas = gui.get_canvas();
  int f = 0;
  for (int i = 0;; i++) {               //              Main Loop
    mpm.step();                        //     Advance simulation
    if (i % int(frame_dt / dt) == 0) {  //        Visualize frame
      canvas.clear(0x112F41);           //       Clear background
      canvas.rect(Vec(0.04,0.04), Vec(0.96,0.96))
          .radius(2)
          .color(0x4FB99F)
          .close();  // Box
      for (auto p : mpm.particles)
        canvas.circle(p.x).radius(3).color(0xF2B134);  // Particles
      gui.update();                               // Update image
      // canvas.img.write_as_image(fmt::format("tmp/{:05d}.png", f++));
    }
  }*/
  
  return 0;
}

