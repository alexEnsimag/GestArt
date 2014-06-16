#include <stdio.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <string>
#include "ViewerTps.hpp"
#include "../../Controller/Kinect/Parser.hpp"
#include <unistd.h>
#include <time.h>
#include "../../Model/OpenGL/math_3d.h"


// A modifier !
Skeleton *squel2;

ViewerTps::ViewerTps(){
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB); 
	glutInitWindowSize(800, 800);
	glutInitWindowPosition(100, 100);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
	squel2 = new Skeleton();
}

static void sleep2(unsigned int ms){
	clock_t goal = ms + clock();
	while(goal>clock());
}

static void RenderSceneCB()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(0.4, 0.0, 0.6);
    squel2->draw();
    glFlush();
/*
	//Animation
	Vector3f p;
	p.x = 0.0;
	p.y = 0.0;
	p.z = 0.0;
	squel2->setMainD(p);
	squel2->setMainG(p);
	squel2->setSchoulderD(p);
	squel2->setSchoulderG(p);
	squel2->setElbowD(p);
	squel2->setElbowG(p);
	squel2->setChest(p);
	squel2->setBassinD(p);
	squel2->setBassinG(p);
	squel2->setGenouxD(p);
	squel2->setGenouxG(p);
	squel2->setPiedD(p);
	squel2->setPiedG(p);
	squel2->setHead(p);
*/
	glutPostRedisplay();
}

static void InitializeGlutCallbacks()
{
    glutDisplayFunc(RenderSceneCB);
}

void ViewerTps::launch(){
	glutCreateWindow("Tutorial 01");
	InitializeGlutCallbacks();
	glutMainLoop();
}


ViewerTps::~ViewerTps(){
	delete squel2;
}
