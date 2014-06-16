/*
* Definit ce qu'est un os. 
*/

#ifndef DEF_SEGMENT
#define	DEF_SEGMENT

#include <vector>
#include <stdio.h>
#include "math_3d.h"

#include "Mesh.hpp"

class Bone{
	public:

		Bone(Vector3f* a, Vector3f* b);
		Bone(Vector3f* a, Vector3f* b, Mesh *m);
		// deux points de l'os
		Vector3f *p1;
		Vector3f *p2;
		// calcule longueur de l'os
		double length();
		// affichage de l'os
		void display();
	private:
		Mesh *mesh;
		// affichage de l'os avec la mesh
		void displayMesh();
		// retourne milieu de l'os
		Vector3f getMiddle();	
		Vector3f getRotation();
		Vector3f getVector();
};


#endif

