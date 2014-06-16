#ifndef DEF_VIEWERJEUX
#define DEF_VIEWERJEUX

#include "../../Model/OpenGL/Mesh.hpp"
#include "../../Model/OpenGL/Scene.hpp"
#include "../../Model/OpenGL/Light.hpp"



class ViewerJeux {
	public:
		ViewerJeux(int argc, char** argv);
		~ViewerJeux();
		void launch();
	private:
		bool InitMesh();
		Light *light;
		Mesh* scene;
		Mesh* alambic;
		Mesh* table;
		Mesh* chaise1;
		Mesh* chaise2;
		Mesh* etageres;
		Mesh* fiole1;
		Mesh* fiole2;
		Mesh* fiole3;
		Mesh* fiole4;

};

#endif
