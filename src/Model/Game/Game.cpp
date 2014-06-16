
#include "Game.hpp"

using namespace std;

Game::Game(){
	updateScenar();
}

int Game::getNbScenar(){
	return scenar.size();
}

/*
*Retourne le scenario numero i
*/
Scenario *Game::getScenar(int i){
	if (i >= scenar.size()){
		cout << "Erreur, tentative d'acces a un scénario inexistant" << endl;
		return &(scenar[0]);
	}
	return &(scenar[i]);
}

// lance le scenario numero i
void Game::launch(int i){
	scenar[i].launch();
}

// maj des scnarios a partir du fichier
void Game::updateScenar(){
	scenar.clear();
	//Parcours du dossier Scenar/ et ajout au jeu
	DIR* rep = opendir("Scenario/");
	if(rep == NULL){
		cout << "ERREUR DOSSIER SCENARIO NOT FOUND" << endl;
		return;
	}
	struct dirent* fichier = NULL;
	string comp = "txt";
	string badswp = "~";
	string nameF;
	string dir = "Scenario/";
	size_t found;

	while ((fichier = readdir(rep)) != NULL){
		nameF = dir + fichier->d_name;
		found = nameF.find(badswp);
		if (found!=std::string::npos){
		}else{
			found = nameF.find(comp);
			if (found!=std::string::npos){
				Scenario *s = new Scenario();
				s->load(nameF);
				scenar.push_back(*s);
			}
		}
	}
	closedir(rep);
}

// supprime un scnenario
void Game::delScenar(int i){
	string fileName = "Scenario/" + scenar[i].getName() + ".txt";
	int res = remove(fileName.c_str());
	scenar.erase(scenar.begin()+i);
}
