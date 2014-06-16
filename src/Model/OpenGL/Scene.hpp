#ifndef DEF_SCENE
#define DEF_SCENE
#include <stdio.h>
#include <math.h>


#include "Segment.hpp"
#include <GL/freeglut.h>
using namespace std;
#include <vector>

class Scene {
	public:
		Scene(Mesh* init);
		void add(Mesh* m);
		void draw();
	private:
		vector<Mesh*> meshes;

};
#endif
