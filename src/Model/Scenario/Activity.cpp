#include "Activity.hpp"
using namespace std;


/*
* Constructeur : met a jour les attributs d'une activite
*/
Activity::Activity(string n, string p, int nb){
	name = n;
	param = p;
        trialsNumber = nb;
        wellDone = false;

	chargerDuree();
}

string Activity::getName(){
	return name;
}

string Activity::getParam(){
	return param;
}

int Activity::getTrialsNumber(){
	return trialsNumber;
}

int Activity::getDuree(){
	return duree;
}


void Activity::decrementTrialsNumber(){
	trialsNumber--;
}

// transforme l'activite en string
string Activity::toString(){
	std::stringstream trialsNb;
	trialsNb << trialsNumber;
	string result = name + ", " + param + " x" + trialsNb.str();
	return result;
}

bool Activity::getWellDone(){
	return wellDone;
}

void Activity::setWellDone(bool b){
	wellDone = b;
	if(name == "ActiviteTestMouv" && b){
		displayMessage("Mouvement reconnu");		
	}	
}

void Activity::init(){
	wellDone = false;
}

void Activity::exec(){
	cout<< "Not yet implemented:" << name << ", " << param << endl;
}

void Activity::closeAct(){
}

/*
* Lance l'activite tant que clle-ci n'est pas reussi et que le 
* nombre d'essai n'a pas encore ete depasse
*/
void Activity::launch(){
	init();
	int count =0;
	while (count < trialsNumber && !wellDone){
		if(count>0){
			// si rate et nb d'essai pas depasse : lancement
			// de la video essaie encore
			Video::launchVideo("Video/EssaieEncore.avi", 5000);
		}
		count ++;
		// execute l'activite
		exec();
	}
	// fermeture de l'activite
	closeAct();
}

void Activity::displayMessage(string s){
	Window *win = new Window();
	win->displayMessage(s);

}


/*
* Charge la duree de la video de l'activite a partir d'un fichier
*/
void Activity::chargerDuree(){
	string nomFichier = "Scenario/Activity/" + getName() + ".txt";

	ifstream fichier(nomFichier.c_str(), ios::in);
	if(!fichier){
		cout << "Erreur à l'ouverture du Fichier" << endl;
		return;
	}
	string param2;
	int duree2;
		while(!fichier.eof()){
			fichier >> param2>> duree2;
			if(param2 == getParam()){
				duree=duree2;
			}
		}	
	fichier.close();

}

int Activity::getDuration() {
	return duration;
}
