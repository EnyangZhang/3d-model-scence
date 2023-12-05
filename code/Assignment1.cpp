//  ========================================================================
//  FILE NAME: Assignment1.cpp
//  AUTHOR: Enyang Zhang
//  ID:72004622
//  ========================================================================

#include <iostream>
#include <GL/freeglut.h>
#include <fstream>
#include <climits>
#include <math.h>
#include <cmath>
#include "loadTGA.h"
#include <GL/glut.h>
#include "loadTGA.h"
using namespace std;

#define GL_CLAMP_TO_EDGE 0x812F   //To get rid of seams between textures
//Vision mode
float angle=0, angle1=0, look_x, look_z=-1., eye_x, eye_z = 200 , temp_eye_x, temp_eye_z;//mode1
float look_x1 =-240, look_z1=-640;   //mode2
int cam_hgt = 280;
int controlview = 1;
//for loading texture
GLuint txId[9];
//Required for creating cylindrical objects
GLUquadric *q;
//cannon
bool fire = false;
float xf = 38.88;
float yf = 64;
float gravity = 0.098;
float v = 0;
float *x, *y, *z;  //vertex coordinate arrays
int *t1, *t2, *t3; //triangles
int nvrt, ntri;    //total number of vertices and triangles
//robot patrol
float alpha, armMove=2, antennaMove, moverobot=0, paramMRobt = -5, faceDirection = -90;
float lightPos= -200.0f;
//robot walk around casle in square route
float robx,roby,robz, cargox = -30, cargoz = 250;
int key=1, faceDirectionCR = -90;
//spaceship spinng angle
float spaceshipBodyspin;
//spaceship transport circle moving and shifting
float transCircleMove, circlesizex=1.0, circlesizey=1.0, circlesizez=1.0, shiftspeed=0.05;
float circlesizex1=0.3, circlesizey1=1, circlesizez1=0.3;
int key1 = 1, times, key2=1, times2;//key is switch parameter for controling circles' size
//spaceship launch
bool launch= false;
float leftoff;
//eternaltower
const int N = 10;		//Number of vertices of the base polygon
float spanangle;        //twist the tower
//ball on the tower(more like atom model)
float mainballangle;
float outerballangle;
//shadow
float spinobjectshadow;  //make shadow to spining

//---loads textures----------------------------------------------------------------
void loadGLTextures()				// Load bitmaps And Convert To Textures
{
    glGenTextures(8, txId); 		// Create texture ids
	// *** left ***
    glBindTexture(GL_TEXTURE_2D, txId[0]);
    loadTGA("skybox.tga");
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);	
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);	

    glBindTexture(GL_TEXTURE_2D, txId[1]);  //Use this texture
    loadTGA("Wall.tga");
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);	//Set texture parameters
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);

    glBindTexture(GL_TEXTURE_2D, txId[2]);  //Use this texture name for the following OpenGL texture
    loadTGA("dock.tga");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);


    glBindTexture(GL_TEXTURE_2D, txId[3]);  //Use this texture name for the following OpenGL texture
    loadTGA("marble.tga");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, txId[4]);  //Use this texture name for the following OpenGL texture
    loadTGA("mainball.tga");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);


    glBindTexture(GL_TEXTURE_2D, txId[5]);  //Use this texture name for the following OpenGL texture
    loadTGA("subball1.tga");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, txId[6]);  //Use this texture name for the following OpenGL texture
    loadTGA("subball2.tga");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, txId[7]);  //Use this texture name for the following OpenGL texture
    loadTGA("subball3.tga");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, txId[8]);  //Use this texture name for the following OpenGL texture
    loadTGA("subball4.tga");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

//---Loads mesh data in OFF format    ---------------------------------------------
void loadMeshFile(const char* fname)
{
    ifstream fp_in;
    int num, ne;

    fp_in.open(fname, ios::in);
    if(!fp_in.is_open())
    {
        cout << "Error opening mesh file" << endl;
        exit(1);
    }

    fp_in.ignore(INT_MAX, '\n');				//ignore first line
    fp_in >> nvrt >> ntri >> ne;			    // read number of vertices, polygons, edges

    x = new float[nvrt];                        //create arrays
    y = new float[nvrt];
    z = new float[nvrt];

    t1 = new int[ntri];
    t2 = new int[ntri];
    t3 = new int[ntri];

    for(int i=0; i < nvrt; i++)                         //read vertex list
        fp_in >> x[i] >> y[i] >> z[i];

    for(int i=0; i < ntri; i++)                         //read polygon list
    {
        fp_in >> num >> t1[i] >> t2[i] >> t3[i];
        if(num != 3)
        {
            cout << "ERROR: Polygon with index " << i  << " is not a triangle." << endl;  //not a triangle!!
            exit(1);
        }
    }

    fp_in.close();
    cout << " File successfully read." << endl;
}

//---Function to compute the normal vector of a triangle with index tindx ---------
void normal(int tindx)
{
    float x1 = x[t1[tindx]], x2 = x[t2[tindx]], x3 = x[t3[tindx]];
    float y1 = y[t1[tindx]], y2 = y[t2[tindx]], y3 = y[t3[tindx]];
    float z1 = z[t1[tindx]], z2 = z[t2[tindx]], z3 = z[t3[tindx]];
    float nx, ny, nz;
    nx = y1*(z2-z3) + y2*(z3-z1) + y3*(z1-z2);
    ny = z1*(x2-x3) + z2*(x3-x1) + z3*(x1-x2);
    nz = x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2);
    glNormal3f(nx, ny, nz);
}

//---draws the mesh model of the cannon--------------------------------------------
void drawCannon()
{
    glColor3f(0.4, 0.5, 0.4);

    //Construct the object model here using triangles read from OFF file
    glBegin(GL_TRIANGLES);
        glScalef(10,10,10);
        for(int tindx = 0; tindx < ntri; tindx++)
        {
           normal(tindx);
           glVertex3d(x[t1[tindx]], y[t1[tindx]], z[t1[tindx]]);
           glVertex3d(x[t2[tindx]], y[t2[tindx]], z[t2[tindx]]);
           glVertex3d(x[t3[tindx]], y[t3[tindx]], z[t3[tindx]]);
        }
    glEnd();
}

//---draws mounting bracket--------------------------------------------------------
void CannonModel(void){
    glPushMatrix();
    glTranslatef(-10, 5, 17);
    glScaled(80, 10, 6);
    glColor3f(0,0,1);
    glutSolidCube(1);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-20, 25, 17);
    glScaled(40, 30, 6);
    glColor3f(0,1,0);
    glutSolidCube(1);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-10, 5, -17);
    glScaled(80, 10, 6);
    glColor3f(0,0,1);
    glutSolidCube(1);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-20, 25, -17);
    glScaled(40, 30, 6);
    glColor3f(0,1,0);
    glutSolidCube(1);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-20, 30, 0);
    glRotatef(30, 0, 0, 1);
    glTranslatef(20, -30, 0);
    drawCannon();
    glPopMatrix();

//    glPushMatrix();
    glTranslatef(xf, yf, 0);
    glColor3f(1,0,0);
    glutSolidSphere(5, 36, 18);
//    glPopMatrix();
}

//---timer for control everything's moving and spining-----------------------------
void myTimer(int value)
{
    //cannon with physic
    if(fire == true){
        xf += 8.66;
        yf += 5-v;
        v += gravity;
}

    //launch the spaceship
    if(leftoff <= 800 && launch == true)
    {
        leftoff += 10;
    }

    //robot walks around casle with cargo in a square route
    switch(key)
    {
    case 1: robx -=10;
        if(robx==-920){
                key = 2;
                faceDirectionCR -= 90;
                cargoz -= 90;
                cargox += 90;
        }break;
    case 2:robz -= 10;
      if(robz == -1410){
      key = 3;
      faceDirectionCR -= 90;
      cargoz += 90;
      cargox += 90;
        }break;
   case 3: robx += 10;
        if(robx == 480){
            key = 4;
            faceDirectionCR -= 90;
            cargoz += 90;
            cargox -= 90;
        }break;
  case 4:robz += 10;
        if(robz == 250){
            key = 1;
            faceDirectionCR -= 90;
            cargoz -= 90;
            cargox -= 90;
        }break;
    }

    //patrol robot
    moverobot+= paramMRobt;
    if(moverobot== -600){
        paramMRobt = 5;
        faceDirection+=180;
        lightPos += 250;
    } else if (moverobot == 200){
        paramMRobt = -5;
        faceDirection-=180;
        lightPos -= 250;
    }

    //antenna spining of robot
    antennaMove += 10;

    alpha += armMove;
    if(alpha == 30){
    armMove = -3;
    } else if(alpha == -30){
        armMove = 3;
    }

    //UFO's dock spining
    spaceshipBodyspin += 10;
    //UFO transport circle
    transCircleMove -=10;
    if(transCircleMove == -800){
        transCircleMove = 0;
    }

    //for first circle model to increase and reduce size
    switch(key1)
    {
    case 1:  circlesizez -= shiftspeed;
             circlesizex -= shiftspeed;
             times += 1;
             if(times==14) key1 = 2;
        break;
    case 2:  circlesizez += shiftspeed;
             circlesizex += shiftspeed;
             times -= 1;
             if(times==0) key1 = 1;
        break;
    }

    //for second circle model to increase and reduce size
    switch(key2)
    {
    case 1:  circlesizez1 += shiftspeed;
        circlesizex1 += shiftspeed;
        times2 += 1;
        if(times2==14) key2 = 2;

        break;
    case 2: circlesizez1 -= shiftspeed;
        circlesizex1 -= shiftspeed;
        times2 -= 1;
        if(times2==0) key2 = 1;
        break;
}

    //eternal tower's atom model rotation and spining
    mainballangle +=2.5;
    outerballangle += 10;

    //shadow
    spinobjectshadow += 0.5;
    glutPostRedisplay();
    glutTimerFunc(40, myTimer, 0);

}

//---creates skybox----------------------------------------------------------------
void skybox(){
   glEnable(GL_TEXTURE_2D);

  ////////////////////// LEFT WALL ///////////////////////
  glBindTexture(GL_TEXTURE_2D, txId[0]);
      //<<<<<<<<<<<< Remove the statements that assign color to the sides of the cube
  glBegin(GL_QUADS);

  glTexCoord2f(0,0.34);
  glVertex3f(-1500,  0, 1500);
  glTexCoord2f(0.25,0.34);
  glVertex3f(-1500, 0., -1500);
  glTexCoord2f(0.25,0.66);
  glVertex3f(-1500, 1500., -1500);
  glTexCoord2f(0,0.66);
  glVertex3f(-1500, 1500, 1500);
  glEnd();

  ////////////////////// FRONT WALL ///////////////////////
  glBindTexture(GL_TEXTURE_2D, txId[0]);

  glBegin(GL_QUADS);
  glTexCoord2f(0.25,0.34);
  glVertex3f(-1500,  0, -1500);
  glTexCoord2f(0.5,0.34);
  glVertex3f(1500, 0., -1500);
  glTexCoord2f(0.5,0.66);
  glVertex3f(1500, 1500, -1500);
  glTexCoord2f(0.25,0.66);
  glVertex3f(-1500,  1500, -1500);
  glEnd();

 ////////////////////// RIGHT WALL ///////////////////////
  glBindTexture(GL_TEXTURE_2D, txId[0]);

  glBegin(GL_QUADS);
  glTexCoord2f(0.5,0.34);

  glVertex3f(1500,  0, -1500);
  glTexCoord2f(0.75,0.34);

  glVertex3f(1500, 0, 1500);
  glTexCoord2f(0.75,0.66);

  glVertex3f(1500, 1500,  1500);
  glTexCoord2f(0.5,0.66);

  glVertex3f(1500,  1500,  -1500);
  glEnd();


  ////////////////////// REAR WALL ////////////////////////
  glBindTexture(GL_TEXTURE_2D, txId[0]);

  glBegin(GL_QUADS);
  glTexCoord2f(0.75,0.34);

  glVertex3f( 1500, 0, 1500);
  glTexCoord2f(1,0.34);

  glVertex3f(-1500, 0,  1500);
  glTexCoord2f(1,0.66);


  glVertex3f(-1500, 1500,  1500);
  glTexCoord2f(0.75,0.66);

  glVertex3f( 1500, 1500, 1500);
  glEnd();
  
  /////////////////////// TOP //////////////////////////
  glBindTexture(GL_TEXTURE_2D, txId[0]);
  glColor3f(1, 0, 1);
  glBegin(GL_QUADS);
  glTexCoord2f(0.26,0.66);

  glVertex3f(-1500, 1500, -1500);
  glTexCoord2f(0.49,0.66);

  glVertex3f(1500, 1500,  -1500);
  glTexCoord2f(0.49,1);

  glVertex3f(1500, 1500,  1500);
  glTexCoord2f(0.26,1);

  glVertex3f(-1500, 1500, 1500);
  glEnd();
}

//---creates the floor-------------------------------------------------------------
void floor()
{
    glDisable(GL_TEXTURE_2D);
    float white[4] = {1., 1., 1., 1.};
    float black[4] = {0};
    glColor4f(0.7, 0.7, 0.7, 1);  //The floor is gray in colour
    glNormal3f(0.0, 1.0, 0.0);

    //The floor is made up of several tiny squares on a 200x200 grid. Each square has a unit size.
    glMaterialfv(GL_FRONT, GL_SPECULAR, black);

    glBegin(GL_QUADS);
    for(int i = -1500; i < 1500; i+=20)
    {
        for(int j = -1500;  j < 1500; j+=20)
        {
            glVertex3f(i, 0, j);
            glVertex3f(i, 0, j+20);
            glVertex3f(i+20, 0, j+20);
            glVertex3f(i+20, 0, j);
        }
    }
    glEnd();
    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
}

//---solar panels on the top of each tower-----------------------------------------
void paraboloids(void){
   glBegin(GL_QUAD_STRIP);
   glColor3f(0.8,0.8,0.);
   for(int i = -20; i < 21; i++){
     glVertex3f(i, 0,0.025*i*i);
     glVertex3f(i, 50,0.025*i*i);

   }
   glEnd();
}

//completed solar system consist of a paraboloids shaped panel and stick beneath it
void solar(void){
    glPushMatrix();
    glTranslatef(0,50,0);
    glRotatef(-90, 1,0,0);
    glRotatef(mainballangle, 0,0,1);
    glTranslatef(0,-25,0);
    paraboloids();
    glPopMatrix();

    glPushMatrix();
    glColor4f (0.7, 0.7, 0.7, 1.0);
    glRotatef(-90, 1, 0,0);
    gluCylinder(q, 5, 5, 50, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();
}

//---casle-------------------------------------------------------------------------
//casle's walls
void walls()
{
    glDisable(GL_TEXTURE_2D);

    glColor4f(1.0, 1.0, 1.0, 1.0);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, txId[1]);
    //3 large polygons (front, back, top)
    glBegin(GL_QUADS);
      glNormal3f(0.0, 0.0, 1.0);   //Facing +z (Front side)
      glTexCoord2f(0.,0.);     glVertex3f(-60.0,0,15);
      glTexCoord2f(6.,0.);     glVertex3f(60.0, 0, 15);
      glTexCoord2f(6,6);    glVertex3f(60,120,15);
      glTexCoord2f(0,6);    glVertex3f(-60, 120, 15);

      glNormal3f(0.0, 0.0, -1.0);   //Facing -z (Back side)

      glTexCoord2f(0.,0.);     glVertex3f(60.0, 0, -15);
      glTexCoord2f(6.,0.);     glVertex3f(-60.0,0,-15);
      glTexCoord2f(6,6);    glVertex3f(-60, 120, -15);
      glTexCoord2f(0,6);    glVertex3f(60,120,-15);

      glDisable(GL_TEXTURE_2D);
      glNormal3f(0.0, 1.0, 0.0);   //Facing +y (Top side)
      glTexCoord2f(0,0.45);     glVertex3f(-60.0, 120.0, 15.0);
      glTexCoord2f(1.,0.45);     glVertex3f(60.0, 120.0,  15.0);
      glTexCoord2f(1,0.83);    glVertex3f(60.0, 120.0, -15.0);
      glTexCoord2f(0,0.83);    glVertex3f(-60.0, 120.0, -15.0);
    glEnd();
    glColor4f(0.5, 0.5, 0.0, 1.0);
    glBegin(GL_QUADS);
      glNormal3f(-1.0, 0.0, 0.0);   //Facing -x (Left side)
      glVertex3f(-60.0, 0.0, -15.0);
      glVertex3f(-60.0, 0.0, 15.0);
      glVertex3f(-60.0, 120.0, 15.0);
      glVertex3f(-60.0, 120.0, -15.0);

      glNormal3f(1.0, 0.0, 0.0);   //Facing +x (Right side)
      glVertex3f(60.0, 0.0, 15.0);
      glVertex3f(60, 5.0, -15.0);
      glVertex3f(60.0, 120.0, -15.0);
      glVertex3f(60.0, 120.0, 15.0);
    glEnd();
}

//the tower between wall and wall
void tower(void)
{
    glTranslatef(-90,0,0);
    GLUquadric *q;
    q = gluNewQuadric();

    glPushMatrix();
    glRotatef(-90, 1, 0,0);
    glColor4f (0.6, 0.6, 0.6, 1.0);
    gluCylinder(q, 30, 30, 125, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

    glPushMatrix();
    glRotatef(-90, 1, 0,0);
    glColor4f (0.7, 0.7, 0.7, 1.0);
    glTranslatef(0,0,0);
    gluDisk(q, 0, 30, 100,  1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

    glPushMatrix();
    glRotatef(-90, 1, 0,0);
    glColor4f (0.7, 0.7, 0.7, 1.0);
    glTranslatef(0,0,120);
    gluDisk(q, 0, 30, 100,  1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

    glPushMatrix();
    glRotatef(-90, 1, 0,0);
    glColor4f (0.7, 0.7, 0.7, 1.0);
    glTranslatef(0,0,125);
    gluDisk(q, 20, 30, 100,  1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

    glPushMatrix();
    glRotatef(-90, 1, 0,0);
    glColor4f (0.7, 0.7, 0.7, 1.0);
    glTranslatef(0,0,120);
    gluCylinder(q, 20, 20, 5, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

//---------tower's edge

    glPushMatrix();
    glColor4f (1, 0., 0., 1.0);
    glTranslatef(0,125,25);
    glutSolidCube(10);
    glPopMatrix();

    glPushMatrix();
    glRotatef(45, 0,1,0);
    glColor4f (1, 0., 0., 1.0);
    glTranslatef(0,125,25);
    glutSolidCube(10);
    glPopMatrix();

    glPushMatrix();
    glRotatef(90, 0,1,0);
    glColor4f (1, 0., 0., 1.0);
    glTranslatef(0,125,25);
    glutSolidCube(10);
    glPopMatrix();

    glPushMatrix();
    glRotatef(135, 0,1,0);
    glColor4f (1, 0., 0., 1.0);
    glTranslatef(0,125,25);
    glutSolidCube(10);
    glPopMatrix();

    glPushMatrix();
    glRotatef(180, 0,1,0);
    glColor4f (1, 0., 0., 1.0);
    glTranslatef(0,125,25);
    glutSolidCube(10);
    glPopMatrix();

    glPushMatrix();
    glRotatef(-45, 0,1,0);
    glColor4f (1, 0., 0., 1.0);
    glTranslatef(0,125,25);
    glutSolidCube(10);
    glPopMatrix();

    glPushMatrix();
    glRotatef(-90, 0,1,0);
    glColor4f (1, 0., 0., 1.0);
    glTranslatef(0,125,25);
    glutSolidCube(10);
    glPopMatrix();

    glPushMatrix();
    glRotatef(-135, 0,1,0);
    glColor4f (1, 0., 0., 1.0);
    glTranslatef(0,125,25);
    glutSolidCube(10);
    glPopMatrix();

    glPushMatrix();
    glRotatef(-180, 0,1,0);
    glColor4f (1, 0., 0., 1.0);
    glTranslatef(0,125,25);
    glutSolidCube(10);
    glPopMatrix();
}

//one wall with one tower as a modular
void wallsModel(void)
{
    glTranslatef(120,0,0);
    walls();
    glDisable(GL_TEXTURE_2D);
    tower();
}

//creates the whole casle
void fortress(void)
{
   for(int i = 0; i >= -720; i-=180)
      {
       if(i == -360){
           glPushMatrix();
           glTranslated(-240,0,0);
           tower();
           glPushMatrix();
           glTranslatef(0,125,0);
           solar();
           glPopMatrix();
           glPopMatrix();
      }else{
        glPushMatrix();
        glTranslatef(i,0,0);
        wallsModel();
        glPushMatrix();
        glTranslatef(0,125,0);
        solar();
        glPopMatrix();
        glPopMatrix();
       }
    }

   for(int i = -210; i >= -930; i-=180)
      { glPushMatrix();
        glTranslatef(-690,0,i);
        glRotatef(-90, 0, 1, 0);
        wallsModel();
        glPushMatrix();
        glTranslatef(0,125,0);
        solar();
        glPopMatrix();
        glPopMatrix();
        }

   for(int i = 240; i >= -540; i-=180)
      { glPushMatrix();
        glTranslatef(i,0,-900);
        glRotatef(-180, 0, 1, 0);
        wallsModel();
        glPushMatrix();
        glTranslatef(0,125,0);
        solar();
        glPopMatrix();
        glPopMatrix();
        }


   for(int i = 30; i >= -750; i-=180)
      {
        glPushMatrix();
        glTranslatef(210,0,i);
        glRotatef(90,0, 1, 0);
        wallsModel();
        glPushMatrix();
        glTranslatef(0,125,0);
        solar();
        glPopMatrix();
        glPopMatrix();
   }
}

//---patrol robot consist of its body and antenna----------------------------------
//antenna
void antenna(void){
    glColor3f(1., 0, 0);           //antenna
    glPushMatrix();
    glTranslatef(0, 8, 0);
    glRotatef(45, 0, 0, 1);
    glRotatef(-90, 1, 0 ,0);
    gluCylinder(q, 0.1, 0.1, 1, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

    glColor3f(1., 0, 0);           //antenna
    glPushMatrix();
    glTranslatef(0, 8, 0);
    glRotatef(-45, 0, 0, 1);
    glRotatef(-90, 1, 0 ,0);
    gluCylinder(q, 0.1, 0.1, 1, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

    glColor3f(0, 0, 1);              //antenna top end ball
    glPushMatrix();
      glTranslatef(0, 8, 0);
      glRotatef(45, 0,0,1);
      glTranslatef(0,1.1,0);
      glutSolidSphere(0.2, 100, 100);
    glPopMatrix();

    glColor3f(0, 0, 1);              //antenna top end ball
    glPushMatrix();
      glTranslatef(0, 8, 0);
      glRotatef(-45, 0,0,1);
      glTranslatef(0,1.1,0);
      glutSolidSphere(0.2, 100, 100);
    glPopMatrix();
}

//compeleted robot modular that can swing arms
void robot(void){
      GLUquadric *q;
       q = gluNewQuadric();
      glColor3f(1., 0.78, 0.06);		//Head
      glPushMatrix();
        glTranslatef(0, 7, 0);
        glutSolidSphere(1, 100, 100);
      glPopMatrix();

      glColor3f(1., 0, 0);              //eye
      glPushMatrix();
        glTranslatef(0.25, 7.25, 0.9);
        glutSolidSphere(0.1, 100, 100);
      glPopMatrix();

      glColor3f(1., 0, 0);
      glPushMatrix();
        glTranslatef(-0.25, 7.25, 0.9);
        glutSolidSphere(0.1, 100, 100);
      glPopMatrix();

      glPushMatrix();                   //antenna
      glRotatef(antennaMove, 0,1,0);
      antenna();
      glPopMatrix();

      glColor3f(1., 0., 0.);			//Torso
      glPushMatrix();
        glTranslatef(0, 5.5, 0);
        glScalef(3, 3, 3);
        glutSolidCube(1);
      glPopMatrix();


      glColor3f(0., 0., 1.);			// leg
      glPushMatrix();
        glTranslatef(0, 3, 0);
        glScalef(1, 3, 1);
        glutSolidCube(1);
      glPopMatrix();

      glPushMatrix();                   //wheel
       glRotatef(90,0,1,0);
       glTranslatef(0,0.75,-0.75);
       glColor3f(1., 0., 0.);
       glPushMatrix();
          gluCylinder(q, 0.6, 0.6, 1.5, 100, 1);
          gluQuadricDrawStyle(q, GLU_FILL);
       glPopMatrix();

        glPushMatrix();
        glColor4f (0.7, 0.7, 0.7, 1.0);
        glTranslatef(0,0,0);
        gluDisk(q, 0, 0.6, 100,  1);
        gluQuadricDrawStyle(q, GLU_FILL);
        glPopMatrix();

         glPushMatrix();
         glColor4f (0.7, 0.7, 0.7, 1.0);
         glTranslatef(0,0,1.5);
         gluDisk(q, 0, 0.6, 100,  1);
         gluQuadricDrawStyle(q, GLU_FILL);
         glPopMatrix();
   glPopMatrix();

       glColor3f(0., 0., 1.);			// base
       glPushMatrix();
         glTranslatef(0, 1.5, 0);
         glScalef(2, 0.5, 2);
         glutSolidCube(1);
       glPopMatrix();

      glColor3f(0., 0., 1.);			//Right arm
      glPushMatrix();
      glTranslatef(-2, 6.5, 0);
      glRotatef(alpha, 1, 0, 0);
      glTranslatef(2, -6.5, 0);
        glTranslatef(-2, 5, 0);
        glScalef(1, 4, 1);
        glutSolidCube(1);
      glPopMatrix();

      glColor3f(0., 0., 1.);			//Left arm
      glPushMatrix();
      glTranslatef(-2, 6.5, 0);
      glRotatef(-alpha, 1, 0, 0);
      glTranslatef(2, -6.5, 0);
        glTranslatef(2, 5, 0);
        glScalef(1, 4, 1);
        glutSolidCube(1);
      glPopMatrix();
}

//---robot pushing cargo-----------------------------------------------------------
//robot's arm facing forward to push a cargo
void robotPushObj(void){
    GLUquadric *q;
       q = gluNewQuadric();
      glColor3f(1., 0.78, 0.06);		//Head
      glPushMatrix();
        glTranslatef(0, 7, 0);
        glutSolidSphere(1, 100, 100);
      glPopMatrix();

      glColor3f(1., 0, 0);              //eye
      glPushMatrix();
        glTranslatef(0.25, 7.25, 0.9);
        glutSolidSphere(0.1, 100, 100);
      glPopMatrix();

      glColor3f(1., 0, 0);
      glPushMatrix();
        glTranslatef(-0.25, 7.25, 0.9);
        glutSolidSphere(0.1, 100, 100);
      glPopMatrix();

      glPushMatrix();
      glRotatef(antennaMove, 0,1,0);
      antenna();
      glPopMatrix();

      glColor3f(1., 0., 0.);			//Torso
      glPushMatrix();
        glTranslatef(0, 5.5, 0);
        glScalef(3, 3, 3);
        glutSolidCube(1);
      glPopMatrix();


      glColor3f(0., 0., 1.);			// leg
      glPushMatrix();
        glTranslatef(0, 3, 0);
        glScalef(1, 3, 1);
        glutSolidCube(1);
      glPopMatrix();

      glPushMatrix();
       glRotatef(90,0,1,0);
       glTranslatef(0,0.75,-0.75);
       glColor3f(1., 0., 0.);			//wheel
       glPushMatrix();
          gluCylinder(q, 0.6, 0.6, 1.5, 100, 1);
          gluQuadricDrawStyle(q, GLU_FILL);
       glPopMatrix();

        glPushMatrix();
        glColor4f (0.7, 0.7, 0.7, 1.0);
        glTranslatef(0,0,0);
        gluDisk(q, 0, 0.6, 100,  1);
        gluQuadricDrawStyle(q, GLU_FILL);
        glPopMatrix();

         glPushMatrix();
         glColor4f (0.7, 0.7, 0.7, 1.0);
         glTranslatef(0,0,1.5);
         gluDisk(q, 0, 0.6, 100,  1);
         gluQuadricDrawStyle(q, GLU_FILL);
         glPopMatrix();
   glPopMatrix();

       glColor3f(0., 0., 1.);			// base
       glPushMatrix();
         glTranslatef(0, 1.5, 0);
         glScalef(2, 0.5, 2);
         glutSolidCube(1);
       glPopMatrix();

      glColor3f(0., 0., 1.);			//Right arm
      glPushMatrix();
      glTranslatef(-2, 6.5, 0);
      glRotatef(-90, 1, 0, 0);
      glTranslatef(2, -6.5, 0);
        glTranslatef(-2, 5, 0);
        glScalef(1, 4, 1);
        glutSolidCube(1);
      glPopMatrix();

      glColor3f(0., 0., 1.);			//Left arm
      glPushMatrix();
      glTranslatef(-2, 6.5, 0);
      glRotatef(-90, 1, 0, 0);
      glTranslatef(2, -6.5, 0);
        glTranslatef(2, 5, 0);
        glScalef(1, 4, 1);
        glutSolidCube(1);
      glPopMatrix();
}

//cargo that robot will push
void cago(void)
{   glColor3f(1.0, 0.75, 0.5);
    glPushMatrix();
    glTranslatef(cargox,50,cargoz);
    glutSolidCube(100);
    glPopMatrix();
}

//-------------------spaceship----------------------------------------------------
//body consist of two part
void spaceshipBody(void){
    glPushMatrix();                  //top half ufo
    glColor4f (0.8, 0.8, 0.8, 1.0);
    glTranslatef(0,50,0);
    glRotatef(-90, 1, 0, 0);
    glutSolidCone(100, 40, 100, 100);
    glPopMatrix();

    glPushMatrix();               //bottom part ufo
    glColor4f (0.6, 0.6, 0.6, 1.0);
    glTranslatef(0,50,0);
    glRotatef(90, 1, 0, 0);
    glutSolidCone(100, 40, 100, 100);
    glPopMatrix();
}

//a sphere dock on the top
void spaceshipDock(void){
    glPushMatrix();
        glTranslatef(0,60,0);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, txId[2]);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        gluSphere ( q, 40.0, 50, 50 );
    glPopMatrix();

        glDisable(GL_TEXTURE_2D);
        glColor4f (0., 0., 1, 1.0);
        glPushMatrix();
            glRotatef(5,0,0,1);
            glTranslatef(0,95,0);
            gluSphere ( q, 10.0, 50, 50 );
        glPopMatrix();
        glPushMatrix();
            glRotatef(-5,0,0,1);
            glTranslatef(0,95,0);
            gluSphere ( q, 10.0, 50, 50 );
        glPopMatrix();

        glPushMatrix();
            glRotatef(30,0,0,1);
            glTranslatef(0,105,0);
            gluSphere ( q, 10.0, 50, 50 );
        glPopMatrix();

        glPushMatrix();
            glRotatef(-30,0,0,1);
            glTranslatef(0,105,0);
            gluSphere ( q, 10.0, 50, 50 );
        glPopMatrix();
}

//spaceship's transport circle beneath the it, just like what it is in cartoon
void spaceshipTransportCircle(void){
    glPushMatrix();
    glRotatef(90, 1,0,0);
    glColor3f(0,0.5,0);
    gluCylinder(q, 60, 60, 5, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0,-10,0);
    glRotatef(90, 1,0,0);
    glColor3f(0,0.5,0);
    gluCylinder(q, 60, 60, 5, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();


    glPushMatrix();
    glTranslatef(0,-20,0);
    glRotatef(90, 1,0,0);
    glColor3f(0,0.5,0);
    gluCylinder(q, 60, 60, 5, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();
}

//completed spaceship modular
void spaceship(void){
    glPushMatrix();
    spaceshipBody();
    glPopMatrix();

    glPushMatrix();
    glRotatef(-spaceshipBodyspin, 0,1,0);
    spaceshipDock();
    glPopMatrix();
}

//---the atom model on the top of eternal tower-----------------------------------------
//the track of atom's electron
void halo(void){
    glPushMatrix();
    glColor4f (0.7, 0.7, 0.7, 1.0);
    glTranslatef(0,230,0);
    glRotatef(90, 0, 1,0);
    gluCylinder(q, 40, 40, 1, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

    glPushMatrix();
    glColor4f (0.7, 0.7, 0.7, 1.0);
    glTranslatef(0,230,0);
    glRotatef(90, 1, 0,0);
    gluCylinder(q, 40, 40, 1, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

    glPushMatrix();
    glColor4f (0.7, 0.7, 0.7, 1.0);
    glTranslatef(0,230,0);
    glRotatef(45,0,0,1);
    glRotatef(90, 0, 1,0);
    gluCylinder(q, 40, 40, 1, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();

    glPushMatrix();
    glColor4f (0.7, 0.7, 0.7, 1.0);
    glTranslatef(0,230,0);
    glRotatef(-45,0,0,1);
    glRotatef(90, 0, 1,0);
    gluCylinder(q, 40, 40, 1, 100, 1);
    gluQuadricDrawStyle(q, GLU_FILL);
    glPopMatrix();
}

//atomic nucleus
void mainball(void){
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, txId[4]);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glPushMatrix();
            glTranslatef(0,230,0);
            glRotatef(mainballangle, 0,1,0);
            glRotatef(-90, 1,0,0);

            gluSphere ( q, 15.0, 50, 50 );
        glPopMatrix();
        glDisable(GL_TEXTURE_2D);

}

//atom's electron
void subball1(void){
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, txId[5]);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glPushMatrix();
            glTranslatef(0,230,0);
            glRotatef(outerballangle, 1,1,0);
            glTranslatef(-28.28, 28.28, 0);
            glRotatef(mainballangle, 0,1,0);
            gluSphere ( q, 10.0, 50, 50 );
        glPopMatrix();
        glDisable(GL_TEXTURE_2D);
}

void subball2(void){
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, txId[6]);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glPushMatrix();
            glTranslatef(0,230,0);
            glRotatef(outerballangle, -1,1,0);
            glTranslatef(-28.28, -28.28, 0);
            glRotatef(mainballangle, 0,1,0);
            gluSphere ( q, 10.0, 50, 50 );
        glPopMatrix();
        glDisable(GL_TEXTURE_2D);

}

void subball3(void){
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, txId[7]);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glPushMatrix();
            glTranslatef(0,230,0);
            glRotatef(outerballangle, 1,0,0);
            glTranslatef(0, 0, 40);
            glRotatef(mainballangle, 0,1,0);
            gluSphere ( q, 10.0, 50, 50 );
        glPopMatrix();
        glDisable(GL_TEXTURE_2D);

}

void subball4(void){
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, txId[8]);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glPushMatrix();
            glTranslatef(0,230,0);
            glRotatef(outerballangle, 0,1,0);
            glTranslatef(40, 0, 0);
            glRotatef(mainballangle, 0,1,0);
            gluSphere ( q, 10.0, 50, 50 );
        glPopMatrix();
        glDisable(GL_TEXTURE_2D);

}

//completed eternal tower modular(tower is in extruede shape)
void eternalTower(void){
    const float CDR = 3.14159265/180.0;
    float vx[N] = {10,20,20,10,0,-10,-20,-20,-10,10};
    float vy[N] = {0};
    float vz[N] = {-20,-10,10,20,30,20,10,-10,-20,-20};
    float wx[N], wy[N], wz[N];
    float text[N] = {0,1,0,1,0,1,0,1,0,1};

    spanangle = -10*CDR;
    for(int a = 0; a <9; a++){//9 level
    for(int i =  0; i < N; i++){
    wx[i] = vx[i]*cos(spanangle) + vz[i]*sin(spanangle);
    wy[i] = vy[i] +20;
    wz[i] = -vx[i]*sin(spanangle) + vz[i]*cos(spanangle);
}
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,txId[3]);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
   glBegin(GL_QUAD_STRIP);
   for(int i = 0; i < N; i++){
      glTexCoord2f(text[i], 0);  glVertex3f(vx[i],vy[i],vz[i]);
      glTexCoord2f(text[i], 1);  glVertex3f(wx[i], wy[i], wz[i]);

   }
   glEnd();

    for(int i =  0; i < N; i++){
        vx[i]=wx[i];
        vy[i]=wy[i];
        vz[i]=wz[i];
    }

}
    mainball();
    subball1();
    subball2();
    subball3();
    subball4();
    halo();
}

//---initialise everything------------------------------------------------------------
void initialise(void) 
{
    loadGLTextures();  // <<<<<<<<< Uncomment this statement
    loadMeshFile("Cannon.off");				//Specify mesh file name here

    float grey[4] = {0.2, 0.2, 0.2, 1.0};
    float white[4]  = {1.0, 1.0, 1.0, 1.0};

    q = gluNewQuadric();

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
   glEnable(GL_LIGHT1);


    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
    glMaterialf(GL_FRONT, GL_SHININESS, 50);
//	Define light's ambient, diffuse, specular properties
    glLightfv(GL_LIGHT0, GL_AMBIENT, grey);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);


    glLightfv(GL_LIGHT1, GL_AMBIENT, grey);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT1, GL_SPECULAR, white);

    glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 40.0);
    glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 0.01);

	glEnable(GL_DEPTH_TEST);
//    glEnable(GL_TEXTURE_2D);
    gluQuadricTexture (q, GL_TRUE);
	glEnable(GL_NORMALIZE);
    gluQuadricDrawStyle(q, GLU_FILL);
	glClearColor (0.0, 0.0, 0.0, 0.0);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective(80.0, 1.0, 100.0, 5000.0);   //Perspective projection
//    gluPerspective(60., 1.0, 10.0, 1000.0);   //Perspective projection

}

//---display--------------------------------------------------------------------------
void display(void)
{
    float lgt_pos[] = {-240.0f, 300.0f, -540.0f, 1.0f};  //light0 position (directly above the origin)
    float lgt_pos1[] = {lightPos, 15.0f, 250.0f, 1.0f};
    float spotdir[] = {1,-1,0};


    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

    //2 camera modes with one extra mode after launching the spaceship
    switch (controlview) {
    case 1:gluLookAt (eye_x, 90, eye_z, look_x, 90, look_z, 0, 1, 0);  //camera moving
        break;
    case -1:
        gluLookAt(-240, cam_hgt, -540, look_x1, cam_hgt, look_z1, 0, 1, 0);  //Camera can only rotate in this mode
        break;
    case 0:
        gluLookAt(-240, leftoff+100, -540, -230, 0, -490, 0, 1, 0);  //looking at the castle beneath spaceship
        break;
    }

    glLightfv(GL_LIGHT0, GL_POSITION, lgt_pos);   //light position

    skybox();
    floor();
    glDisable(GL_TEXTURE_2D);


    glPushMatrix();
    glTranslatef(-240,leftoff,-540);
    glScalef(2,2,2);
    spaceship();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0,transCircleMove, 0);
    glTranslatef(-240,leftoff,-540);
    glScalef(circlesizex, circlesizey, circlesizez);
    glScalef(2,2,2);
    spaceshipTransportCircle();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0,transCircleMove, 0);
    glTranslatef(-240,-80+leftoff,-540);
    glScalef(circlesizex1, circlesizey1, circlesizez1);
    glScalef(2,2,2);
    spaceshipTransportCircle();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-150,0,100);
    glRotatef(-90, 0,1,0);
    CannonModel();
    glPopMatrix();
    fortress();

    glPushMatrix();
    glTranslatef(moverobot, 0,0);
    glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, spotdir);
    glLightfv(GL_LIGHT1, GL_POSITION, lgt_pos1);   //light position
    glTranslatef(0,0,250);
    glRotatef(faceDirection, 0,1,0);
    glScalef(10,10,10);
    robot();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(robx, roby, robz);
    glTranslatef(0,0,100);
    cago();
    glTranslatef(60,0,250);
    glRotatef(faceDirectionCR, 0,1,0);
    glScalef(10,10,10);
    robotPushObj();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0,0,-300);
    glScalef(1.25,1.25,1.25);
    eternalTower();
    glPopMatrix();

//     Draw the shadow
        glPushMatrix();
        glTranslatef(0,0.1,0);
        float shadowMat[16] = {
            300,0,0,0,
            240,0,540,-1,
            0,0,300,0,
            0,0,0,300
        };

        glDisable(GL_LIGHTING);
        glPushMatrix();
        glMultMatrixf(shadowMat);
        glColor4f(0.2,0.2,0.2,1.0);
              glTranslatef(-200, 100, -300);
              glRotatef(spinobjectshadow, 1, 0, 0);
              glRotatef(spinobjectshadow*2, 0, 1, 0);
              glColor3f(0.2, 0.2, 0.2);
              glutSolidCube(30);
         glPopMatrix();
         glPopMatrix();

     //  Draw Object
          glEnable(GL_LIGHTING);
         glColor3f(1, 0, 1);
         glPushMatrix();
           glTranslatef(-200, 100, -300);
           glRotatef(spinobjectshadow, 1, 0, 0);
           glRotatef(spinobjectshadow*2, 0, 1, 0);
           glColor3f(1, 0, 0);
           glutSolidCube(30);
         glPopMatrix();

    glPushMatrix();
    glTranslatef(0, 15, 0);
    glRotatef(-30, 0, 0, 1);
    glutSolidCube(10);
    glPopMatrix();
    glutSwapBuffers();
}

//---cameral control------------------------------------------------------------------
 void special(int key, int x, int y)
 {
     if(key == GLUT_KEY_HOME){
        if(launch){
            if(controlview == 0){
            controlview = 1;}
            else if (controlview == 1){
                controlview = 0;
            }
        } else{
            controlview = -controlview;
     }
     }

     switch (controlview) {
     case 1:
         if(key == GLUT_KEY_DOWN)
         {  //Move backward
             temp_eye_x -= 10*sin(angle);
             temp_eye_z += 10*cos(angle);
         }
         else if(key == GLUT_KEY_UP)
         { //Move forward
             temp_eye_x += 10*sin(angle);
             temp_eye_z -= 10*cos(angle);
         }

         if(temp_eye_x > 1350 || temp_eye_x < -1350){
             if(temp_eye_x > 1350) temp_eye_x =1350;
             if(temp_eye_x < -1350) temp_eye_x =-1350;
         }
         if(temp_eye_z > 1150 || temp_eye_z < -1550){
             if(temp_eye_z > 1150) temp_eye_z =1150;
             if(temp_eye_z < -1550) temp_eye_z =-1550;
         }

    if(temp_eye_x < 1350 && temp_eye_x > -1350 && temp_eye_z < 1150 && temp_eye_z > -1550){
         if(key == GLUT_KEY_DOWN)
         {  //Move backward
             eye_x -= 10*sin(angle);
             eye_z += 10*cos(angle);
         }
         else if(key == GLUT_KEY_UP)
         { //Move forward
             eye_x += 10*sin(angle);
             eye_z -= 10*cos(angle);
         }
    }
    if(key == GLUT_KEY_LEFT) angle -= 5 * M_PI/180;  //Change direction
    else if(key == GLUT_KEY_RIGHT) angle += 5 * M_PI/180;// M_PI/180 for switch 5 deg to radians so that each direction change leads 5 degree turning
    look_x = eye_x + 100*sin(angle);
    look_z = eye_z - 100*cos(angle);

         break;
     case -1:
         if(key == GLUT_KEY_LEFT) {
             angle1 -= 5 * M_PI/180;  //Change direction
             look_x1 = -240 + 100*sin(angle1);
             look_z1 = -540 - 100*cos(angle1);
         } else if(key == GLUT_KEY_RIGHT){

         angle1 += 5 * M_PI/180;// M_PI/180 for switch 5 deg to radians so that each direction change leads 5 degree turning
         look_x1 = -240 + 100*sin(angle1);
         look_z1 = -540 - 100*cos(angle1);
}
         break;
     }
	glutPostRedisplay();
}

//---fire the cannon or launch the spaceship------------------------------------------
 void keyboard(unsigned char key, int x, int y){
     switch(key){
     case 'c': fire = true; break;
     case 's':if(launch==false) controlview = 0;
                launch = true;

         break;
     }
     glutPostRedisplay();
 }

//---main-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE| GLUT_RGB | GLUT_DEPTH );
   glutInitWindowSize (700, 700);
   glutInitWindowPosition (50, 50);

   glutCreateWindow ("Assignment1");
   initialise();
   glutDisplayFunc(display);
   glutSpecialFunc(special);
   glutKeyboardFunc(keyboard);

   glutTimerFunc(40, myTimer, 0);

   glutMainLoop();
   return 0;
}
