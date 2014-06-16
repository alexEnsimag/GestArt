#include "Light.hpp"

	static GLfloat const white[] = {1.0f,1.0f,1.0f, 1.f};
	static GLfloat const black[] = {0.0f,0.0f,0.0f, 1.f};
	static GLfloat const purple[] = {150/255,0.0f,150/255, 1.f};

Light::Light(){
	glEnable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

void setLight(){
	glLightfv(GL_LIGHT1, GL_AMBIENT, black);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
	glLightfv(GL_LIGHT1, GL_SPECULAR, white);
	GLfloat position[] = {0.9f, 0.9f, 0.9f}; 
	glLightfv(GL_LIGHT1, GL_POSITION, position);

	glLightfv(GL_LIGHT2, GL_AMBIENT, black);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, purple);
	glLightfv(GL_LIGHT2, GL_SPECULAR, purple);
	GLfloat position2[] = {0.0f, 0.9f, 0.0f}; 
	glLightfv(GL_LIGHT2, GL_POSITION, position2);
}
