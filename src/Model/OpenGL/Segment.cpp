#include "Segment.hpp"


Bone::Bone(Vector3f* a, Vector3f* b){
	p1 = a;
	p2 = b;
	mesh = NULL;
}

Bone::Bone(Vector3f* a, Vector3f* b, Mesh* m){
	p1 = a;
	p2 = b;
	mesh = m;
}

// Affichage de l'os avec la mesh
void Bone::displayMesh(){
	if (mesh == NULL)
		return;
	Vector3f center = getMiddle();
	Vector3f rot = getRotation();
	glPushMatrix();
	glTranslatef(center.x, center.y, center.z);
	glRotatef(rot.z,0, 0, 1);
	mesh->Render(1.0/2.5);
	glPopMatrix();
}

// Affichage de l'os
void Bone::display(){
	glBegin(GL_LINES);
	glVertex3f(p1->x,p1->y,p1->z);
	glVertex3f(p2->x,p2->y,p2->z);
	glEnd();
	displayMesh();
}

Vector3f Bone::getMiddle(){
	Vector3f result;
	result.x = (p1->x+p2->x)/2;
	result.y = (p1->y+p2->y)/2;
	result.z = (p1->z+p2->z)/2;
	return result;
}

double Bone::length(){
	double result = 0;
	result += (p2->x-p1->x)*(p2->x-p1->x);
	result += (p2->y-p1->y)*(p2->y-p1->y);
	result += (p2->z-p1->z)*(p2->z-p1->z);
}

Vector3f Bone::getVector(){
	Vector3f result;
	result.x = (p2->x-p1->x);
	result.y = (p2->y-p1->y);
	result.z = (p2->z-p1->z);
	return result;
}

Vector3f Bone::getRotation(){
	float x = getVector().x;
	float y = getVector().y;
	float z = getVector().z;
	Vector3f result;
	result.x = 0;
	result.y = 2*atan(z/(x+sqrt(x*x+z*z)))*180/M_PI;
	result.z = 2*atan(y/(x+sqrt(x*x+y*y)))*180/M_PI;
	return result;
}
