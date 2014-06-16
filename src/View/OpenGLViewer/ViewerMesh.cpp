#include <stdio.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <math.h>
#include "ViewerMesh.hpp"
#include "../../Model/OpenGL/Mesh.hpp"


// A modifier !
Mesh *m_pMesh2;

ViewerMesh::ViewerMesh(int argc, char** argv){

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH); 
//	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glutInitWindowSize(800, 800);
	glutInitWindowPosition(100, 100);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glFrontFace(GL_CW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
}


static void RenderSceneCB()
{
	glClear(GL_COLOR_BUFFER_BIT);
	m_pMesh2->Render(1/2.5);
        glutSwapBuffers();

}

static void InitializeGlutCallbacks()
{
	glutDisplayFunc(RenderSceneCB);
}

static bool InitMesh(){

	m_pMesh2 = new Mesh();

	return m_pMesh2->LoadMesh("Modele/Scotty.3ds");

}

void ViewerMesh::launch(){
	glutCreateWindow("Game Mode");
	// Must be done after glut is initialized!
	GLenum res = glewInit();
	if (res != GLEW_OK) {
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
	}

	if(! InitMesh()){
		fprintf(stderr, "Error: '%s'\n", "error while loading the mesh");
	}

	InitializeGlutCallbacks();
	glutMainLoop();
}


ViewerMesh::~ViewerMesh(){
	delete m_pMesh2;
}
