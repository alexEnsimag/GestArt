#include <stdio.h>
#include <string.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <math.h>
#include "ViewerJeux.hpp"

GLuint VBO2;
Scene *decorBehind;
Scene *decorBefore;

bool ViewerJeux::InitMesh(){
	light = new Light();
	scene = new Mesh();
	alambic = new Mesh();
	table = new Mesh();
	table->setColor(115/255,54/255,0);
	chaise1 = new Mesh();
	chaise2 = new Mesh();
	etageres = new Mesh();
	fiole1 = new Mesh();
	fiole1->setColor(1.0f,0.0f,0.0f);
	fiole2 = new Mesh();
	fiole2->setColor(0.0f,1.0f,0.0f);
	fiole3 = new Mesh();
	fiole3->setColor(1.0f,0.0f,0.0f);
	fiole4 = new Mesh();
	fiole4->setColor(1.0f,1.0f,0.0f);

	if(!scene->LoadMesh("Modele/scene.3ds"))
		return false;
	if(!alambic->LoadMesh("Modele/alambic.3ds"))
		return false;
	if(!table->LoadMesh("Modele/table.3ds"))
		return false;
	if(!chaise1->LoadMesh("Modele/chaise1.3ds"))
		return false;
	if(!chaise2->LoadMesh("Modele/chaise2.3ds"))
		return false;
	if(!etageres->LoadMesh("Modele/etageres.3ds"))
		return false;
	if(!fiole1->LoadMesh("Modele/fiole1.3ds"))
		return false;
	if(!fiole2->LoadMesh("Modele/fiole2.3ds"))
		return false;
	if(!fiole3->LoadMesh("Modele/fiole3.3ds"))
		return false;
	if(!fiole4->LoadMesh("Modele/fiole4.3ds"))
		return false;

	decorBehind = new Scene(scene);
	decorBehind->add(fiole4);
	decorBehind->add(fiole3);
	decorBehind->add(fiole2);
	decorBehind->add(fiole1);
	decorBefore = new Scene(chaise1);
	decorBefore->add(alambic);
	decorBefore->add(table);
	decorBefore->add(chaise2);

	return true;
}

ViewerJeux::ViewerJeux(int argc, char** argv){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB); 
	glutInitWindowSize(800, 800);
	glutInitWindowPosition(100, 100);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-25.0, 25.0, -25.0, 25.0, -25.0, 25.0);
	glEnable(GL_DEPTH_TEST);
}

static void RenderSceneCB()
{
	glClear(GL_COLOR_BUFFER_BIT);
	//Meshes
	decorBehind->draw();

	setLight();

	decorBefore->draw();
	//animation
	glFlush();
	glutPostRedisplay();
}


static void InitializeGlutCallbacks()
{
	glutDisplayFunc(RenderSceneCB);
}

void ViewerJeux::launch(){
	glutCreateWindow("Game Mode");
	// Must be done after glut is initialized!
	GLenum res = glewInit();
	if (res != GLEW_OK) {
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
	}
	if(! InitMesh()){
		fprintf(stderr, "Error: '%s'\n", "error while loading the mesh");
		return;
	}

	InitializeGlutCallbacks();

	glutMainLoop();
}

ViewerJeux::~ViewerJeux(){
	delete decorBefore;
	delete light;
	delete decorBehind;
	delete fiole4;
	delete fiole3;
	delete fiole2;
	delete fiole1;
	delete table;
	delete scene;
	delete chaise1;
	delete chaise2;
	delete alambic;
	delete etageres;
}
