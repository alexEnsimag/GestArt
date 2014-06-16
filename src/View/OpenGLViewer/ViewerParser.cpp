#include "ViewerParser.hpp"

// A modifier !
Skeleton *squel;
Parser *parser;

/*
* Charge les differentes mesh blender
*/
bool ViewerParser::InitMesh(){
	aBrasD = new Mesh();
	aBrasG = new Mesh();
	brasD = new Mesh();
	brasG = new Mesh();
	torse = new Mesh();
	tete = new Mesh();
	bassin = new Mesh();
	jambeD = new Mesh();
	jambeG = new Mesh();
	molletD = new Mesh();
	molletG = new Mesh();

	if(!aBrasD->LoadMesh("Modele/avant_brasD.3ds"))
		return false;
	if(!aBrasG->LoadMesh("Modele/avant_brasG.3ds"))
		return false;
	if(!brasD->LoadMesh("Modele/brasD.3ds"))
		return false;
	if(!brasG->LoadMesh("Modele/brasG.3ds"))
		return false;
	if(!jambeD->LoadMesh("Modele/jambe.3ds"))
		return false;
	if(!jambeG->LoadMesh("Modele/jambe.3ds"))
		return false;
	if(!molletG->LoadMesh("Modele/mollet.3ds"))
		return false;
	if(!molletD->LoadMesh("Modele/mollet.3ds"))
		return false;
	if(!tete->LoadMesh("Modele/tete.3ds"))
		return false;
	if(!torse->LoadMesh("Modele/torse.3ds"))
		return false;
	if(!bassin->LoadMesh("Modele/bassin.3ds"))
		return false;

	return true;
}

/*
* Constructeur : Cree la fenetre et un nouveau parser afin
* d'animer notre squelette
*/
ViewerParser::ViewerParser(){

	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB); 
	glutInitWindowSize(800, 800);
	glutInitWindowPosition(100, 100);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-25.0, 25.0, -25.0, 25.0, -25.0, 25.0);
	glEnable(GL_DEPTH_TEST);

	parser = new Parser();
}

static void sleep2(unsigned int ms){
	clock_t goal = ms + clock();
	while(goal>clock());
}

/*
* Initialise les couleurs et dessine le squelette. Cette fonction est 
* appele en continue
*/
static void RenderSceneCB()
{
	sleep2(100000);
	glClear(GL_COLOR_BUFFER_BIT);
	// lights
	glEnable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	GLfloat const white[] = {1.0f,1.0f,1.0f, 1.f};
	glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
	GLfloat position[] = {1.0f, 1.0f, 0.5f}; 
	glLightfv(GL_LIGHT1, GL_POSITION, position);
	//Meshes & skeleton
	squel->draw();
	//animation
	glFlush();
	parser->parse(squel);
	glutPostRedisplay();
}

static void InitializeGlutCallbacks()
{
	glutDisplayFunc(RenderSceneCB);
}

/*
* Affiche la fenetre et lance la boucle qui rend la scene. 
*/ 
void ViewerParser::launch(string fileName){
	glutCreateWindow("Visualisation de mouvement");
	// Must be done after glut is initialized!
	GLenum res = glewInit();
	if (res != GLEW_OK) {
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
		return;
	}
	/*
	if(! InitMesh()){
		fprintf(stderr, "Error: '%s'\n", "error while loading the mesh");
		return;
	}
	*/
	//squel = new Skeleton(aBrasD, aBrasG, brasD, brasG, torse, tete, bassin, jambeD, jambeG, molletD, molletG);
	squel = new Skeleton();

	parser->openFile(fileName);

	InitializeGlutCallbacks();
	glutMainLoop();
	parser->close();
}


ViewerParser::~ViewerParser(){
	delete aBrasD;
	delete aBrasG;
	delete brasD;
	delete brasG;
	delete torse;
	delete tete;
	delete bassin;
	delete jambeD;
	delete jambeG;
	delete molletD;
	delete molletG;
	delete squel;
}
