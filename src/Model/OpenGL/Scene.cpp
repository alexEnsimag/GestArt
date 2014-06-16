#include "Scene.hpp"


Scene::Scene(Mesh* scene){
	meshes.push_back(scene);	
}
void Scene::add(Mesh* m){
	meshes.push_back(m);	
}

void Scene::draw(){
	glPushMatrix();
	glTranslatef(0,-0.9, -0.65);
	for(int i=0; i<meshes.size(); i++){
		meshes[i]->Render(1.0/5.0);
	}
	glPopMatrix();
}
