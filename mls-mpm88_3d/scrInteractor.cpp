#include "scrInteractor.h"
#include <stdio.h>
#include <iostream>

using namespace std;

bool scrInteractor::Normal_Change = 0;
float* scrInteractor::Centroide = new float[3];
;

/*====================== glButton ===================================*/

/*===================================================================*/
void glButton::Draw_Label() /* Escreve no botão */
{
	glColor3fv(this->text_color);
	for (unsigned int i = 0; i < strlen(this->label); i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, this->label[i]);
}

/*==============================================================*/
void glButton::Draw() /* Desenha o botão */
{

	if (this->visible) {
		glColor3fv(this->btn_color);
		glBegin(GL_POLYGON);
		glVertex2f(this->edge.left, this->edge.top);
		glVertex2f(this->edge.left, this->edge.bottom);
		glVertex2f(this->edge.right, this->edge.bottom);
		glVertex2f(this->edge.right, this->edge.top);
		glEnd();

		glColor3f(1.0f, 1.0f, 1.0f);

		glBegin(GL_LINE_LOOP);
		glVertex2f(this->edge.left, this->edge.top);
		glVertex2f(this->edge.left, this->edge.bottom);
		glVertex2f(this->edge.right, this->edge.bottom);
		glVertex2f(this->edge.right, this->edge.top);
		glEnd();

		glColor3fv(this->text_color);
		glRasterPos2f(this->edge.left + 0.005, this->edge.top + 0.1); // pocisiona onde será escrito no botão
		this->Draw_Label();
	}

}

/*==============================================================*/
void glButton::init(auxRect<float> edge_values, float * clr_edge,
		float * clr_btn, char * text) /* inicializa as variaveis dos botões*/
		{
	this->btn_color = new float[3];
	this->text_color = new float[3];

	this->setColor(clr_edge, clr_btn);

	this->edge = edge_values;

	this->label = new char[strlen(text) + 1];

	this->visible = true;

	strcpy(this->label, text);
	//
}
/*==============================================================*/

void glButton::setColor(float * text, float * btn) /* define as cores do texto e do botão*/
{
	for (int i = 0; i < 3; i++) {
		this->text_color[i] = text[i];
		this->btn_color[i] = btn[i];
	}
}

/*================ scrInteractor ===================================*/

scrInteractor::scrInteractor(int width, int height) {

	this->rec_screen.setValues(0, width, 0, height);

	this->render = RUSER;

	this->mouse_left = this->mouse_right = this->user_button = false;

	this->point_size = 2;

	this->vec_translation = new float[3];
	this->vec_translation[0] = this->vec_translation[1] =
			this->vec_translation[2] = 0.0f;

	this->scale = 1.0;
	this->LookupTable=false;

}

/*==============================================================*/

bool scrInteractor::Get_Normal_Change() // Devolve a variavel que diz se o sinal das normais deve ser mudado ou não
{
	return Normal_Change;
}
/*===================================================================*/
bool scrInteractor::Init(float xmin, float xmax, float ymin, float ymax,
		float near_z, float far_z) {
	auxRect<float> rec_ext;
	rec_ext.left = xmin;
	rec_ext.right = xmax;
	rec_ext.top = ymin;
	rec_ext.bottom = ymax;
	Init(rec_ext, near_z, far_z);
	simulTime=0.0;
	return true;
}
;

/*===================================================================*/
bool scrInteractor::Init(auxRect<float> rec_extents, float near_z,
		float far_z) {

	this->rec_initial = rec_extents;

	this->aux_z = (far_z + near_z) / 2;
	this->far_z = (far_z - this->aux_z) * 10.0;
	this->near_z = (near_z - this->aux_z) * 10.0;
	this->bg_color = new float[3];
	this->Normal_Change = false;
	this->Centroide = new float[3];
	this->Centroide[0] = 0;
	this->Centroide[1] = 0;
	this->Centroide[2] = 0;

	this->bg_state = USER;

	this->setBGColor();
	simulTime=0.0;
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

	glutInitWindowSize(this->rec_screen.Width(), this->rec_screen.Height());

	glutInitWindowPosition(10, 10);

	this->main_winid = glutCreateWindow("Visualizador");

	glPointSize(this->point_size);

	this->Init_Menu();   // create the buttons

	this->Init_List();      // create the display list

	this->matrix = new float[16];
	auxMatrix::Zero_Matrix(this->matrix); //identidade

	this->axis_matrix = new float[16];
	auxMatrix::Zero_Matrix(this->axis_matrix);

	this->pt_mouse.setValues(0, 0);
	this->pt_mouse_old.setValues(0, 0);

	this->Init_Lights();

	this->Calc_Rotation();
        //this->Initial_Rotation();
	this->Initial_Translation();

	this->setLights(true);

	glClearColor(getBGColor(0), getBGColor(1), getBGColor(2), 0.0);
	//glClear(GL_COLOR_BUFFER_BIT);

	// enable depth test - zbuffer
	glEnable(GL_DEPTH_TEST);
	
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	// Clear the screen
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	return true;
}

/*===================================================================*/
// function called from display list
/*===================================================================*/

void scrInteractor::setDraw(void (*func)()) {
	this->functionDraw = func;
}

/*===================================================================*/
void scrInteractor::Reshape(int width, int height) {

	this->rec_screen.bottom = height;
	this->rec_screen.right = width;

	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

    if (width <= height)
        glFrustum(rec_initial.left, rec_initial.right, rec_initial.top,
                rec_initial.bottom * width / height, near_z + aux_z,
				far_z + aux_z);
	else
        glFrustum(rec_initial.left, rec_initial.right * width / height,
                rec_initial.top, rec_initial.bottom, near_z + aux_z,
                far_z + aux_z);


	glutSetWindow(this->sub_winid);
	glutReshapeWindow(width, height / SUB_SIZE);
	glutPositionWindow(0, 0);
	glutSetWindow(this->main_winid);

}

/*===================================================================*/
void scrInteractor::MouseMotion(int x, int y) /* é acessada com o movimento do mouse, com o botão clicado*/
{
	pt_mouse_old = pt_mouse;
	pt_mouse.setValues(x, y);

	if (this->mouse_left && this->mouse_right)
		this->Calc_Translation();
	else if (this->mouse_left && !this->mouse_right)
		this->Calc_Rotation();
	else if (!this->mouse_left && this->mouse_right)
		this->Calc_Scale();

	glutPostRedisplay();
}

/*===================================================================*/
void scrInteractor::Mouse(int button, int state, int x, int y) /* coordena */
{
	int mod_keys;
	static char names[3] = { 'X', 'Y', 'Z' };
//	static float wz;
//	static GLint viewport[4];
// static GLint my;

	if (state == GLUT_DOWN) {
		if (pt_mouse.getZero())
			pt_mouse.setValues(x, y);

		pt_mouse_old = pt_mouse;
		pt_mouse.setValues(x, y);

		if (button == GLUT_RIGHT_BUTTON)
			this->mouse_right = true;

		if (button == GLUT_LEFT_BUTTON) {
			mod_keys = glutGetModifiers();
			if (mod_keys == GLUT_ACTIVE_SHIFT) {
				double point[3];
				char aux_label[7];
				char btn_label[25];

				this->ScreenToPoint(this->pt_mouse, point);
				int i;
				for (i = 0; i < 2; i++) {
					if (this->buttons[i + 2].getState() == true)
						strcpy(aux_label, "locked");
					else
						strcpy(aux_label, "free");

					sprintf(btn_label, "%c : %4.5f    (%s)", names[i], point[i],
							aux_label);
					strcat(btn_label, "\0");

					this->buttons[i + 2].setLabel(btn_label);
					strcpy(btn_label, "");
				}

				sprintf(btn_label, "%c : %4.5f     (%s)", names[i], point[i],
						"free");

				this->buttons[4].setLabel(btn_label);

				glutSetWindow(this->sub_winid);
				glutPostRedisplay();

			} else
				this->mouse_left = true;
		}

		glutPostRedisplay();
	} else {
		if (button == GLUT_LEFT_BUTTON)
			mouse_left = false;

		if (button == GLUT_RIGHT_BUTTON)
			mouse_right = false;
	}

}

/*===================================================================*/
bool scrInteractor::Keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case '+':
		/*if (this->point_size < 20)
		 this->point_size ++;
		 
		 glPointSize(this->point_size);*/
		this->scale += 0.5;

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		if (rec_screen.Width() <= rec_screen.Height())
            glFrustum(rec_initial.left, rec_initial.right, rec_initial.top,
                    rec_initial.bottom * rec_screen.Width()
                            / rec_screen.Height(), (near_z * scale) + aux_z,
                    (far_z * scale) + aux_z);
		else
            glFrustum(rec_initial.left,
                    rec_initial.right * rec_screen.Width()
                            / rec_screen.Height(), rec_initial.top,
                    rec_initial.bottom, (near_z * scale) + aux_z,
                    (far_z * scale) + aux_z);
		glutPostRedisplay();
		return true;
		break;

	case '-':
		/*if (this->point_size > 0)
		 this->point_size --;
		 
		 glPointSize(this->point_size);*/
		this->scale += -0.5;

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		if (rec_screen.Width() <= rec_screen.Height())
            glFrustum(rec_initial.left, rec_initial.right, rec_initial.top,
                    rec_initial.bottom * rec_screen.Width()
                            / rec_screen.Height(), (near_z * scale) + aux_z,
                    (far_z * scale) + aux_z);
		else
            glFrustum(rec_initial.left,
                    rec_initial.right * rec_screen.Width()
                            / rec_screen.Height(), rec_initial.top,
                    rec_initial.bottom, (near_z * scale) + aux_z,
                    (far_z * scale) + aux_z);
		glutPostRedisplay();
		return true;
		break;

	case 'z':
		this->Calc_Scale();
		glutPostRedisplay();
		return true;
		break;

	}

	return false;
}

/*===================================================================*/
void scrInteractor::Init_List() // placa de video, nope processador
{
	this->view_list = glGenLists(1);

	glNewList(this->view_list, GL_COMPILE);
	this->functionDraw();
	glEndList();

}

/*===================================================================*/
void scrInteractor::Display() {
	glutSetWindow(this->main_winid);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Clear the window with current clearing color
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glPushAttrib(GL_LIGHTING_BIT);

	glScalef(this->scale, this->scale, this->scale);
	glTranslatef(this->vec_translation[0], this->vec_translation[1],
			this->vec_translation[2]);   // rotation point
	glTranslatef(this->val_tx, this->val_ty, this->val_tz); // normal translation

	glMultMatrixf(this->matrix);        // rotation

	glTranslatef(-this->vec_translation[0], -this->vec_translation[1],
			-this->vec_translation[2]); // back from rotation point

	if (this->render == RSHADING_WIRE) {
		// needed to add an offset to the depth values of a polygon's fragments before the 
		//     depth comparison is performed
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0, 1.0);

		// depth test to make zbuffer
		glEnable(GL_DEPTH_TEST);

		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		glColor3f(0.45f, 0.45f, 0.45f);

		// wont change the RGB, just the alpha
		glColorMask(false, false, false, true);

		glCallList(this->view_list);   // display the object

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glColorMask(true, true, true, true);

		glCallList(this->view_list);  // display the object in FLAT_SHADING mode

        /*glColor3f(1.0, 0.0, 0.0);
		glPointSize(3);

        glBegin(GL_POINTS);    // display the rotation point
		glVertex3f(this->vec_translation[0], this->vec_translation[1],
				this->vec_translation[2]);
        glEnd();

        glPointSize(this->point_size);*/

		glDisable(GL_POLYGON_OFFSET_FILL);

        //glDisable(GL_DEPTH_TEST);  // disable to improve performance

	} else if (this->render == RUSER) {

		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0, 1.0);
		glEnable(GL_DEPTH_TEST);
		//glPolygonMode(GL_FRONT_AND_BACK,  GL_LINE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glColorMask(true, true, true, true);
		glCallList(this->view_list);

		glDisable(GL_DEPTH_TEST);
		glDisable(GL_POLYGON_OFFSET_FILL);
	} else {

		glCallList(this->view_list);
	}

    //this->Display_Point();

	glGetDoublev(GL_MODELVIEW_MATRIX, this->model_view_matrix);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf(this->matrix);
	this->Draw_Axis();

	glPopAttrib();   // used with setLights(true)

	glFlush();
	glutSwapBuffers();

}

/*===================================================================*/
void scrInteractor::Reset() {
	this->trackball.Reset();
	auxMatrix::Zero_Matrix(this->matrix);
	auxMatrix::Zero_Matrix(this->axis_matrix);

	this->val_tx = -this->initial_tx;
	this->val_ty = -this->initial_ty;
	this->val_tz = -this->initial_tz;
	this->scale = 1;

	glutSetWindow(this->main_winid);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	if (rec_screen.right <= rec_screen.bottom)
        glFrustum(rec_initial.left, rec_initial.right, rec_initial.top,
				rec_initial.bottom * rec_screen.right / rec_screen.bottom,
				near_z + aux_z, far_z + aux_z);
	else
        glFrustum(rec_initial.left,
				rec_initial.right * rec_screen.right / rec_screen.bottom,
				rec_initial.top, rec_initial.bottom, near_z + aux_z,
				far_z + aux_z);

	glutPostRedisplay();

	auxVector::vecZero(this->vec_translation);

	this->render = RUSER;
	this->setRender_Mode();

}

/*===================================================================*/
void scrInteractor::setRender_Mode() {
	char label[30];
	switch (this->render) {
	case RUSER:
		sprintf(label, "Render : User");
		this->buttons[0].setLabel(label);

		glutSetWindow(this->sub_winid);
		glutPostRedisplay();

		glutSetWindow(this->main_winid);
       // gluPerspective(90.f, 1.f, 1.f, 500.f);

        //glMatrixMode(GL_MODELVIEW);
        //glLoadIdentity();
		glutPostRedisplay();

		glDisable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		this->setLights(true);

		break;

	case RWIRE:
		sprintf(label, "Render : Wireframe");
		this->buttons[0].setLabel(label);

		glutSetWindow(this->sub_winid);
		glutPostRedisplay();

		glutSetWindow(this->main_winid);
		glutPostRedisplay();

		glDisable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		this->setLights(false);

		break;

	case RPOINT:
		sprintf(label, "Render : Points");
		this->buttons[0].setLabel(label);

		glPointSize(this->point_size);

		glutSetWindow(this->sub_winid);
		glutPostRedisplay();

		glutSetWindow(this->main_winid);
		glutPostRedisplay();

		glDisable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		this->setLights(false);
		break;

	case RSHADING:
		sprintf(label, "Render : Shading");
		this->buttons[0].setLabel(label);

		glutSetWindow(this->sub_winid);
		glutPostRedisplay();

		glutSetWindow(this->main_winid);
		glutPostRedisplay();

		glEnable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		this->setLights(true);
		break;


	case RSHADING_WIRE:
		sprintf(label, "Render : Shading+");
		this->buttons[0].setLabel(label);

		glColor3f(0.0, 0.0, 0.0);

		glutSetWindow(this->sub_winid);
		glutPostRedisplay();

		glutSetWindow(this->main_winid);
		glutPostRedisplay();

		glDisable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		this->setLights(true);

		break;
	}
}


/*===================================================================*/
void scrInteractor::Initial_Rotation() {
        float sx, sy, ex, ey;

        /*sx = 2.0f * this->pt_mouse_old.getX() / this->rec_screen.Width() - 1.0f;

        sy = 1.0f - 2.0f * this->pt_mouse_old.getY() / this->rec_screen.Height();

        ex = 2.0f * this->pt_mouse.getX() / this->rec_screen.Width() - 1.0f;

        ey = 1.0f - 2.0f * this->pt_mouse.getY() / this->rec_screen.Height();*/

        TrackBall temp_trackball(2, 0, 0, 0, this->buttons[2].getState(),
                        this->buttons[3].getState());

        this->trackball.Add_Quats(temp_trackball.getQuats());

        auxMatrix::Mult_Matrix(&this->matrix, this->trackball.getMatrix());
}


/*===================================================================*/
void scrInteractor::Calc_Rotation() {
	float sx, sy, ex, ey;

	sx = 2.0f * this->pt_mouse_old.getX() / this->rec_screen.Width() - 1.0f;

	sy = 1.0f - 2.0f * this->pt_mouse_old.getY() / this->rec_screen.Height();

	ex = 2.0f * this->pt_mouse.getX() / this->rec_screen.Width() - 1.0f;

	ey = 1.0f - 2.0f * this->pt_mouse.getY() / this->rec_screen.Height();

	TrackBall temp_trackball(sx, sy, ex, ey, this->buttons[2].getState(),
			this->buttons[3].getState());

	this->trackball.Add_Quats(temp_trackball.getQuats());

	auxMatrix::Mult_Matrix(&this->matrix, this->trackball.getMatrix());
}

/*===================================================================*/
void scrInteractor::Calc_Translation() {
	static float temp_tx, temp_ty;

	if (this->buttons[2].getState() == true)
		temp_tx = 0;
	else
		temp_tx = ((float) this->pt_mouse.getX()
				- (float) this->pt_mouse_old.getX()) / this->rec_screen.Width();

	if (this->buttons[3].getState() == true)
		temp_ty = 0;
	else
		temp_ty = ((float) this->pt_mouse_old.getY()
				- (float) this->pt_mouse.getY()) / this->rec_screen.Height();

	this->val_tx += (float) this->rec_initial.Width() * temp_tx / this->scale;
	this->val_ty += (float) this->rec_initial.Height() * temp_ty / this->scale;

}

/*===================================================================*/

void scrInteractor::Calc_Scale() {
	this->scale += ((float) this->pt_mouse.getY()
			- (float) this->pt_mouse_old.getY())
			/ (float) this->rec_screen.Height();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	if (rec_screen.Width() <= rec_screen.Height())
        glFrustum(rec_initial.left, rec_initial.right, rec_initial.top,
				rec_initial.bottom * rec_screen.Width() / rec_screen.Height(),
				(near_z * scale) + aux_z, (far_z * scale) + aux_z);
	else
        glFrustum(rec_initial.left,
				rec_initial.right * rec_screen.Width() / rec_screen.Height(),
				rec_initial.top, rec_initial.bottom, (near_z * scale) + aux_z,
				(far_z * scale) + aux_z);
}

/*===================================================================*/
void scrInteractor::Draw_String(const char* text, float r, float g, float b)
{
   glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
    glDisable(GL_LIGHTING);
  glColor3f(r,g,b);
  for (unsigned int i = 0; i < strlen(text); i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, text[i]);

   glEnable(GL_LIGHTING);
    glPopAttrib();
}

/*===================================================================*/
void scrInteractor::Draw_Axis() {
	auxPoint<int> point_screen;
	double point_world[3];
	float lsize[2];

	glGetFloatv(GL_LINE_WIDTH_RANGE, lsize);

	static float curr_size = lsize[0];
	static float coord_val = MIN(this->rec_initial.Width() / 10,
			this->rec_initial.Height() / 10);
	float coord_x = this->rec_screen.left + (this->rec_screen.right / 25);
	float coord_y = this->rec_screen.bottom - (this->rec_screen.bottom / 25);

	point_screen.setValues((int) coord_x, (int) coord_y);

	this->ScreenToPoint_Axis(point_screen, point_world);

	glLineWidth(5 * curr_size);
	glutSetWindow(this->main_winid);

	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(point_world[0], point_world[1], point_world[2]);
	glVertex3f(point_world[0] + coord_val, point_world[1], point_world[2]);

	glVertex3f(point_world[0] + (coord_val * 0.9),
			point_world[1] + (coord_val / 10), point_world[2]);
	glVertex3f(point_world[0] + coord_val, point_world[1], point_world[2]);

	glVertex3f(point_world[0] + (coord_val * 0.9),
			point_world[1] - (coord_val / 10), point_world[2]);
	glVertex3f(point_world[0] + coord_val, point_world[1], point_world[2]);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(point_world[0], point_world[1], point_world[2]);
	glVertex3f(point_world[0], point_world[1] + coord_val, point_world[2]);

	glVertex3f(point_world[0] + (coord_val / 10),
			point_world[1] + (coord_val * 0.9), point_world[2]);
	glVertex3f(point_world[0], point_world[1] + coord_val, point_world[2]);

	glVertex3f(point_world[0] - (coord_val / 10),
			point_world[1] + (coord_val * 0.9), point_world[2]);
	glVertex3f(point_world[0], point_world[1] + coord_val, point_world[2]);

	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(point_world[0], point_world[1], point_world[2]);
	glVertex3f(point_world[0], point_world[1], point_world[2] + coord_val);

	glVertex3f(point_world[0] + (coord_val / 10), point_world[1],
			point_world[2] + (coord_val * 0.9));
	glVertex3f(point_world[0], point_world[1], point_world[2] + coord_val);

	glVertex3f(point_world[0] - (coord_val / 10), point_world[1],
			point_world[2] + (coord_val * 0.9));
	glVertex3f(point_world[0], point_world[1], point_world[2] + coord_val);

	glEnd();
	glLineWidth(curr_size);
	glColor3f(1.0f, 1.0f, 1.0f);

}

/*===================================================================*/
void scrInteractor::Init_Lights() /* define a iluminação da malha*/
{
//	float amb_light[] =  {0.2f, 0.2f, 0.2f, 1.0f};
  
   float _angle = 50.0f;
   
   
         
    //Add ambient light
    GLfloat ambientColor[] = {0.25f, 0.25f, 0.25f, 1.0f}; //Color (0.2, 0.2, 0.2)
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);
    
    //Add positioned light
    GLfloat diffuseLightColor0[] = {0.5f, 0.5f, 0.5f, 1.0f}; 
    GLfloat specularLightColor0[] = {1.0f, 1.0f, 1.0f, 1.0f}; 
    GLfloat lightPos0[] = {10.5f, -5.5f, 2.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLightColor0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specularLightColor0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
    
    //Add directed light
    GLfloat lightColor1[] = {1.0f, 1.0f, 1.0f, 1.0f}; //Color (0.5, 0.2, 0.2)
    //Coming from the direction (-1, 0.5, 0.5)
    GLfloat lightPos1[] = {10.5f, -10.5f, -2.5f, 1.0f};
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor1);
    //glLightfv(GL_LIGHT1, GL_SPECULAR, specularLightColor0);
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);
    glRotatef(_angle, 0.0f, 1.0f, 0.0f);
    
   
	// Capacidade de brilho do material
	GLfloat especularidade[4] = { 0.25, 0.25, 0.25, 0.25 };
	GLint especMaterial = 5;

	// Especifica que a cor de fundo da janela será preta
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	// Habilita o modelo de colorização de Gouraud
	glShadeModel(GL_SMOOTH);

	// Define a refletância do material
	glMaterialfv(GL_FRONT, GL_SPECULAR, especularidade);
	// Define a concentração do brilho
	glMateriali(GL_FRONT, GL_SHININESS, especMaterial);

	// Ativa o uso da luz ambiente
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);

	// Define os parâmetros da luz de número 0
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLightColor0);
	glLightfv(GL_LIGHT1, GL_AMBIENT, ambientColor);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLightColor0);
	//glLightfv(GL_LIGHT1, GL_SPECULAR, luzEspecular);
	//glLightfv(GL_LIGHT1, GL_POSITION, posicaoLuz);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);

	// Habilita a definição da cor do material a partir da cor corrente
	glEnable(GL_COLOR_MATERIAL);
	//Habilita o uso de iluminação
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_LIGHTING);
	// Habilita a luz de número 0
	glEnable(GL_LIGHT0);
	// Habilita o depth-buffering
	glEnable(GL_DEPTH_TEST);

	glEnable(GL_NORMALIZE);
}

/*===================================================================*/
void scrInteractor::setLights(bool lights_on) /* habilita ou não o uso da iluminação, de acordo com o modo de desenho da malha*/
{
	if (lights_on)
		glEnable(GL_LIGHTING);
	else
		glDisable(GL_LIGHTING);

}

/*===================================================================*/
void scrInteractor::ScreenToPoint(auxPoint<int> mouse, double point[3]) {
	static GLint viewport[4], my;
	static GLdouble projection_matrix[16];
	static short tmp_mz;
	static GLfloat mz;

	glPushMatrix();
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);

	my = viewport[3] - mouse.y;

	glReadBuffer(GL_FRONT);
	glReadPixels(mouse.x, my, 1, 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT,
			&tmp_mz);

	mz = (float) tmp_mz / 65535.0;  // maximum value for unsigned_short

	gluUnProject(mouse.x, my, mz, model_view_matrix, projection_matrix,
			viewport, &point[0], &point[1], &point[2]);
	glPopMatrix();

}

/*===================================================================*/
void scrInteractor::ScreenToPoint_Axis(auxPoint<int> mouse, double point[3]) {
	static GLint viewport[4], my;
	static GLdouble model_view[16], projection_matrix[16];

	glPushMatrix();
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);

	my = viewport[3] - mouse.y;

	gluUnProject(mouse.x, my, 0.5, model_view, projection_matrix, viewport,
			&point[0], &point[1], &point[2]);
	glPopMatrix();

}

/*===================================================================*/

void scrInteractor::Set_Centroide(float x, float y, float z) {
	Centroide[0] = x;
	Centroide[1] = y;
	Centroide[2] = z;
}

/*===================================================================*/
void scrInteractor::Change_BG_Color() /* atualiza a cor do BG*/
{
	glutSetWindow(this->main_winid);
	glClearColor(this->getBGColor(0), this->getBGColor(1), this->getBGColor(2),
			0.0);
	glClear(GL_COLOR_BUFFER_BIT);

	glutPostRedisplay();
}

/*===================================================================*/

float scrInteractor::getBGColor(int rgb) /* devolve o valor r, g ou b da nova cor do BG*/
{
	return this->bg_color[rgb];
}

/*===================================================================*/

void scrInteractor::setBGColor() /* define um novo estado para o BG de acordo com o estado atual*/
{
    if (this->getBGState() == GL_WHITE) {
		this->setUserBGColor();
		this->bg_state = USER;
	} else if (this->getBGState() == USER) {
		this->bg_color[0] = 1.0;
		this->bg_color[1] = 1.0;
		this->bg_color[2] = 1.0;
        this->bg_state = GL_BLACK;
	} else {
		this->bg_color[0] = 1.0;
		this->bg_color[1] = 1.0;
		this->bg_color[2] = 1.0;
        this->bg_state = GL_WHITE;
	}
}

/*==============================================================*/

int scrInteractor::getBGState() /*retorna o estado atual do BG*/
{
	return this->bg_state;
}

/*==============================================================*/

void scrInteractor::setUserBGColor() /*onde o usuario seta a cor do BG*/
{

	/*coloque aqui os valores rgb*/
	this->bg_color[0] = 0.4; /*r*/
	this->bg_color[1] = 0.2; /*g*/
	this->bg_color[2] = 0.7; /*b*/

}

/*==============================================================*/
void scrInteractor::Initial_Translation() {
	this->initial_tx = this->rec_initial.getCenterX();
	this->initial_ty = this->rec_initial.getCenterY();
	this->initial_tz = (this->far_z + this->near_z) / 2;

	this->rec_initial.left -= this->initial_tx;
	this->rec_initial.right -= this->initial_tx;
	this->rec_initial.bottom -= this->initial_ty;
	this->rec_initial.top -= this->initial_ty;
	this->near_z -= this->initial_tz;
	this->far_z -= this->initial_tz;

	this->vec_translation[0] = this->initial_tx;
	this->vec_translation[1] = this->initial_ty;
	this->vec_translation[2] = this->initial_tz;

	this->val_tx = -this->initial_tx;
	this->val_ty = -this->initial_ty;
	this->val_tz = -this->initial_tz;

	// fit in the screen
	this->rec_initial.left += this->rec_initial.left * 0.2;
	this->rec_initial.right += this->rec_initial.right * 0.2;
	this->rec_initial.bottom += this->rec_initial.bottom * 0.2;
	this->rec_initial.top += this->rec_initial.top * 0.2;

}

/*===================================================================*/
void scrInteractor::subInit() {
	this->sub_winid = glutCreateSubWindow(this->main_winid, 0, 0,
			this->rec_screen.right, this->rec_screen.bottom / SUB_SIZE);

}

/*===================================================================*/
void scrInteractor::subReshape(int width, int height) {

	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

    glFrustum(0.0f, 0.5*width, 0.0, 1.0*height, 0.0f, 1.0f);

	glMatrixMode(GL_MODELVIEW);

	glLoadIdentity();

}

/*===================================================================*/
void scrInteractor::subMouse(int button, int state, int x, int y) /* coordena as ações dos botões de acordo com o clique do mouse*/
{
	char label[30];
	strcpy(label, "");

	if (state == GLUT_DOWN) {
		if (button == GLUT_LEFT_BUTTON) {
			int id = this->getObject_Id(x, y);
			switch (id) {
			case BTN_RENDER:

				switch (this->render) {
				case RUSER:
					this->render = RWIRE;

					break;

				case RWIRE:
					this->render = RPOINT;
					break;

				case RPOINT:
					this->render = RSHADING_WIRE;
					break;
					void Save_State();

				case RSHADING:
					this->render = RSHADING_WIRE;
					break;

				case RSHADING_WIRE:
					this->render = RUSER;
					break;
				}
				this->setRender_Mode();
				break;

			case BTN_RESET:
				this->Reset();
				glutPostRedisplay();
				break;

			case BTN_X_AXIS:
				this->buttons[2].setState();
				strncat(label, this->buttons[2].getLabel(),
						strlen(this->buttons[2].getLabel()) - 8);
				if (this->buttons[2].getState() == true)
					strcat(label, "(locked)");
				else
					strcat(label, "  (free)");

				this->buttons[2].setLabel(label);
				glutPostRedisplay();

				break;

			case BTN_Y_AXIS: {
				this->buttons[3].setState();
				strncat(label, this->buttons[3].getLabel(),
						strlen(this->buttons[3].getLabel()) - 8);
				if (this->buttons[3].getState() == true)
					strcat(label, "(locked)");
				else
					strcat(label, "  (free)");

				this->buttons[3].setLabel(label);
				glutPostRedisplay();
			}
				break;

			case BTN_Z_AXIS: {
				this->buttons[4].setState();
				strncat(label, this->buttons[4].getLabel(),
						strlen(this->buttons[4].getLabel()) - 8);
				if (this->buttons[4].getState() == true)
					strcat(label, "(locked)");
				else
					strcat(label, "  (free)");

				this->buttons[4].setLabel(label);
				glutPostRedisplay();
			}

				break;

			case BTN_COLOR: {
				//this->buttons[5].setState();
				//strncat(label, this->buttons[5].getLabel(), strlen(this->buttons[5].getLabel()) - 7);

				int state = getBGState();

				switch (state) {
                case GL_BLACK: {
					strncat(label, this->buttons[5].getLabel(),
							strlen(this->buttons[5].getLabel()) - 6);
					strcat(label, " White");
					this->buttons[5].setLabel(label);
					glutPostRedisplay();

					this->setBGColor();

					this->Change_BG_Color();

				}
					break;

                case GL_WHITE: {

					this->setBGColor();
					strcpy(label, "BackGround Color:  User");
					this->buttons[5].setLabel(label);
					glutPostRedisplay();

					this->Change_BG_Color();

				}
					break;

				case USER: {
					strncat(label, this->buttons[5].getLabel(),
							strlen(this->buttons[5].getLabel()) - 6);
					this->setBGColor();
					strcat(label, " Black");
					this->buttons[5].setLabel(label);
					glutPostRedisplay();

					this->Change_BG_Color();
				}
					break;

				}

			}
				break;

			case BTN_CHANGE_NORMAL: {

				this->Normal_Change = !this->Normal_Change; // muda a variavel que define como vai estar a normal

				glutSetWindow(this->main_winid);

				this->Refresh_List();

				glutPostRedisplay();

			}
				break;

			case BTN_CENTROIDE: {

				glutSetWindow(main_winid);

				//this->vec_translation = Calc_Centroide();	

				this->vec_translation[0] = Centroide[0];
				this->vec_translation[1] = Centroide[1];
				this->vec_translation[2] = Centroide[2];

				sprintf(label, "X(%4.3f) Y(%4.3f) Z(%4.3f)",
						this->vec_translation[0], this->vec_translation[1],
						this->vec_translation[2]);
				glRasterPos2f(0.77, 0.33);

                this->Draw_String(label,1.0,1.0,1.0);

				glutPostRedisplay();

			}
				break;

			}

		}
		if (button == GLUT_RIGHT_BUTTON)

			glutPostRedisplay();
		glFlush();
	}

}

/*===================================================================*/

void scrInteractor::ShowLookupTable(bool ok,float min, float max,std::string label,float totalTime)
{

 LookupTable = ok;

 MinLookupTable=min;
 MaxLookupTable=max;
 LookupTableLabel = label;
  simulTime = totalTime;
 
       
    
}


/*===================================================================*/

void scrInteractor::subDisplay() {
	char label[50];

	glutSetWindow(this->sub_winid);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClearColor(0.65, 0.65, 0.65, 0.0);

	// Clear the window with current clearing color
	glClear(GL_COLOR_BUFFER_BIT);

	glColor3f(0.0f, 0.2f, 0.2f);
	glInitNames();
	glPushName(0);
	glLoadName(BTN_RENDER);
	this->buttons[0].Draw();
	glEnd();

	glLoadName(BTN_RESET);
	this->buttons[1].Draw();
	glEnd();

	glLoadName(BTN_X_AXIS);
	this->buttons[2].Draw();
	glEnd();

	glLoadName(BTN_Y_AXIS);
	this->buttons[3].Draw();
	glEnd();

	glLoadName(BTN_Z_AXIS);
	this->buttons[4].Draw();
	glEnd();

	glLoadName(BTN_COLOR);
	this->buttons[5].Draw();
	glEnd();

	glLoadName(BTN_CHANGE_NORMAL);
	this->buttons[6].Draw();
	glEnd();

	glLoadName(BTN_CENTROIDE);
	this->buttons[7].Draw();
	glEnd();

	/*****************************************************************************************/
	// arrumar a localização destes caracteres.  
	/*****************************************************************************************/
	 if(LookupTable==false)
	 {
	sprintf(label, "Rotation Point");

	glColor3f(1.0f, 1.0f, 1.0f);
    glRasterPos2f(0.02, 0.33);
    this->Draw_String(label,1.0f, 1.0f, 1.0f);

	sprintf(label, "X(%1.2f) Y(%1.2f) Z(%1.2f) - stime = %1.3f", this->vec_translation[0],
			this->vec_translation[1], this->vec_translation[2],simulTime);

    glRasterPos2f(0.27, 0.33);
    this->Draw_String(label,1.0f, 1.0f, 1.0f);
  
	  }
	  else{
           //glColor3d(0.0, 0.0, 0.0);
            /* Tell OpenGL your intentions */
            glBegin(GL_POLYGON);
                /* Set the vertices */
                glColor3f(0.058823529, 0.058823529, 1.0);
                glVertex2d(0.03, -0.83);
                glColor3f(0.058823529, 0.058823529, 1.0);
                glVertex2d(0.03, 0.13);
                glColor3f(0.058823529, 0.388235294, 0.388235294);
                glVertex2d(0.33, 0.13);
                glColor3f(0.058823529, 0.388235294, 0.388235294);
                glVertex2d(0.33, -0.83);
                /* Done making polygon */
            glEnd();
            glBegin(GL_POLYGON);
                /* Set the vertices */
                glColor3f(0.058823529, 0.388235294, 0.388235294);
                glVertex2d(0.33, -0.83);
                glColor3f(0.058823529, 0.388235294, 0.388235294);
                glVertex2d(0.33, 0.13);
                glColor3f(0.717647059, 0.717647059,0.058823529f );
                glVertex2d(0.63, 0.13);
                glColor3f(0.717647059, 0.717647059,0.058823529f );
                glVertex2d(0.63, -0.83);
                /* Done making polygon */
            glEnd();
	    glBegin(GL_POLYGON);
                /* Set the vertices */
                glColor3f(0.717647059, 0.717647059,0.058823529f );
                glVertex2d(0.63, -0.83);
                glColor3f(0.717647059, 0.717647059,0.058823529f );
                glVertex2d(0.63, 0.13);
                glColor3f(1.0f,0.058823529f, 0.058823529f );
                glVertex2d(0.93, 0.13);
                glColor3f(1.0f,0.058823529f, 0.058823529f );
                glVertex2d(0.93, -0.83);
                /* Done making polygon */
            glEnd();
            sprintf(label, "%2.2f",MinLookupTable);
            glColor3f(0.0f, 0.0f, 0.0f);
            glRasterPos2f(0.03, 0.24);
            this->Draw_String(label,1.0f, 1.0f, 1.0f);
            sprintf(label, "%2.6f",MaxLookupTable);
            glColor3f(0.0f, 0.0f, 0.0f);
            glRasterPos2f(0.86, 0.24);
            this->Draw_String(label,1.0f, 1.0f, 1.0f);
            sprintf(label, "%2.2f",(MaxLookupTable+MinLookupTable)*0.5);
            glColor3f(0.0f, 0.0f, 0.0f);
            glRasterPos2f(0.35, 0.24);
            this->Draw_String(label,1.0f, 1.0f, 1.0f);
            sprintf(label, "%s - stime = %1.3f",LookupTableLabel.c_str(),simulTime);
            glColor3f(1.0f, 1.0f, 1.0f);
            glRasterPos2f(0.38, -0.55);
            this->Draw_String(label,1.0f, 1.0f, 1.0f);
          }
	glFlush();
	glutSwapBuffers();
	glutPostRedisplay();
}
/*===================================================================*/
float * scrInteractor::getTranslation() {
	return this->vec_translation;
}
/*===================================================================*/

int scrInteractor::getObject_Id(int x, int y) {

	int objects_found = 0;
	int viewport[4] = { 0 };

	unsigned int select_buffer[32] = { 0 };

	glSelectBuffer(32, select_buffer);

	glGetIntegerv(GL_VIEWPORT, viewport);

	glMatrixMode(GL_PROJECTION);

	glPushMatrix();
	glRenderMode(GL_SELECT);

	glLoadIdentity();

	gluPickMatrix(x, viewport[3] - y, 2, 2, viewport);

	glFrustum(0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f);

	glMatrixMode(GL_MODELVIEW);

	this->subDisplay();

	objects_found = glRenderMode(GL_RENDER);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);

	if (objects_found > 0) {
		unsigned int lowest_depth = select_buffer[1];

		int selected_object = select_buffer[3];

		for (int i = 1; i < objects_found; i++) {
			if (select_buffer[(i * 4) + 1] < lowest_depth) {
				lowest_depth = select_buffer[(i * 4) + 1];

				selected_object = select_buffer[(i * 4) + 3];
			}
		}

		return selected_object;
	}
	return 0;
}

/*===================================================================*/

void scrInteractor::Init_Menu() {
	auxRect<float> rec_button;

	char * label = new char[30];

	static float btn_color[3] = { 0.0f, 0.6f, 0.6f };
	float text_color[3] = { 1.0f, 1.0f, 1.0f };

	this->buttons = new glButton[8];

	// Render Button
    rec_button.setValues(-1.0, -0.75, -0.99, -0.015);
	strcpy(label, "Render : User");
	this->buttons[0].init(rec_button, text_color, btn_color, label);

	// Reset Button
    rec_button.setValues(-1.0, -0.75, 0.015, 0.99);
	strcpy(label, "Reset to User Mode");
	this->buttons[1].init(rec_button, text_color, btn_color, label);

	// X Axis Button
	text_color[1] = 0.0f;
	text_color[2] = 0.0f;
    rec_button.setValues(-0.745, -0.5, 0.015, 0.99);
	strcpy(label, "X:   (free)");
	this->buttons[2].init(rec_button, text_color, btn_color, label);

	// Y Axis Button
	text_color[0] = 0.0f;
	text_color[1] = 1.0f;
    rec_button.setValues(-0.745, -0.5, -0.99, -0.015);
	strcpy(label, "Y:   (free)");
	this->buttons[3].init(rec_button, text_color, btn_color, label);

	// Z Axis Button  
	text_color[1] = 0.0f;
	text_color[2] = 1.0f;
    rec_button.setValues(-0.495, -0.2, 0.015, 0.99);
	strcpy(label, "Z:   (free)");
	this->buttons[4].init(rec_button, text_color, btn_color, label);

	// color Button
    rec_button.setValues(-0.495, -0.2, -0.99, -0.015);
	strcpy(label, "BackGround Color: Black");
	this->buttons[5].init(rec_button, text_color, btn_color, label);

	// normal Button
    rec_button.setValues(-0.195, 0.0, -0.99, -0.015);
	strcpy(label, "Change Normal");
	this->buttons[6].init(rec_button, text_color, btn_color, label);

	// centróide Button
    rec_button.setValues(-0.195, 0.0, 0.015, 0.99);
	strcpy(label, "Calc Centroide");
	this->buttons[7].init(rec_button, text_color, btn_color, label);

}

/*===================================================================*/

void scrInteractor::Refresh_List() {
	glDeleteLists(this->view_list, 1);
	this->Init_List();
}

/*===================================================================*/

void scrInteractor::Display_Point() {
	glColor3f(1.0, 0.0, 0.0);
	glPointSize(3);
	glBegin(GL_POINTS);
	glVertex3f(this->vec_translation[0], this->vec_translation[1],
			this->vec_translation[2]);
	glEnd();
	glPointSize(this->point_size);

}

/*===================================================================*/

void scrInteractor::SetScreenImageFilename(std::string filename) {

	if (filename != this->screenFilename) {
		this->screenFilenameNumber = 0;
		this->screenFilename = filename;
	}
}

/*===================================================================*/

std::string &
scrInteractor::GetScreenImageFilename() {
	return this->screenFilename;
}

/*===================================================================*/

bool scrInteractor::WriteScreenImage(float time=0.0) {

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
	SetScreenImageFilename("ScreenVis");
	int width, height;
	width = this->rec_screen.right - this->rec_screen.left;
	height = this->rec_screen.bottom - this->rec_screen.top;

	/* Allocate our buffer for the image */
	 image = reinterpret_cast<unsigned char*>(malloc(3*width*height*sizeof(char)));
	if (image == NULL) {
		fprintf(stderr, "Failed to allocate memory for image\n");
		return (false);
	}

	glPixelStorei(GL_PACK_ALIGNMENT, 1);

	sprintf(fname, "%s_%04d_time_%.4f.ppm", this->screenFilename.c_str(),
			this->screenFilenameNumber,time);

	if ((fptr = fopen(fname, "w")) == NULL) {
		fprintf(stderr, "Failed to open file for window dump\n");
		return false;
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
	this->screenFilenameNumber++;
	// free(image);
	return true;
}

