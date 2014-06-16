
#include "Scenario.hpp"
using namespace std;

// Ajoute une activite a la liste d'activite du scenario
void Scenario::addActivity(Activity *a){
	activities.push_back(a);
}

/*
* Ecrit tous les parametres d'un scenario dans un fichier
*/
void Scenario::record(){
	string fileName = "Scenario/" + name + ".txt";
	ofstream file(fileName.c_str(), ios::out | ios::trunc);
	if(!file){
		cout << "Erreur a la création du Fichier" << endl;
		return;
	}
	file << name << endl;
	// parcourt de la liste des activites
	for(int i=0; i<activities.size(); i++){
		// ecriture de l'activite
		file << activities[i]->getName() << endl;
		// ecriture des parametres
		file << activities[i]->getParam() << endl;
		// ecriture du nombre d'essai
		file << activities[i]->getTrialsNumber() << endl;
	}
	file.close();
}

/*
* Lecture du fichier d'un scenario (fileName) et ajout des activites
* contenu dans ce fichier dans la liste des activites du scenario
*/
void Scenario::load(string fileName){
	//Ouverture du fichier
	ifstream file(fileName.c_str(), ios::in);
	if(!file){
		cout << "Erreur à l'ouverture du Fichier" << endl;
		return;
	}
	string mot;
	string param;
	file >> mot;
	setName(mot);
	int trialsNb;

	while(!file.eof()){
		file >> mot >> param >> trialsNb;
		// si activite Objet
		if (mot == "ActiviteObjet"){
			addActivity(new ObjectActivity(param, trialsNb));
		// si activite Forme
		}else if (mot == "ActiviteForme"){
			addActivity(new ShapeActivity(param, trialsNb));
		}else{
			cout << "Activitée non reconnue" << endl;
		}
	}
	//On enleve le doublon de fin
	activities.pop_back();
	file.close();
}

// modifie le nom du scenario
void Scenario::setName(string s){
	name = s;
}

// retourne le nom du scenario
string Scenario::getName(){
	return name;
}


	
/*
* Lancement du scenario : - video d'initialisation
			  - lancement of et processing
			  - lance chaque activite
			  - fermeture of et processing 
			   
*/
// true si of et processing sont lancé, false sinon
bool b = false;
void Scenario::launch(){
	// lancement video debut
	Video::launchVideo("Video/Intro.avi", 7000);
	// parcourt de la liste des activités
	for(int i=0; i<activities.size(); i++){
		activities[i]->setWellDone(false);
		string uri = "Video/" + activities[i]->getParam() + ".avi";
		Video::launchVideo(uri, activities[i]->getDuree());

		// si c'est une activité forme 
		if(!b && activities[i]->getName()=="ActiviteForme"){
			b = true;
			// lancement of et processing
			of = new Of();
			of->launchOfRecognize(true, false);
		}
		// si c'est une activite objet, et que b vaut true
		if(b && activities[i]->getName()=="ActiviteObjet"){
			// on ferme of et processing
			killOf();
			b = false;
		}	
		cout<<activities[i]->getName()<<", "<<activities[i]->getParam()<<endl;
		// lancement activite
		activities[i]->launch();
	}
	// si of et proc encore ouverts, on les ferme
	if(b){
		killOf();
	}
}

// retourne nombre d'activités
int Scenario::getNbActivities(){
	return activities.size();
}

// retourne l'activite numero i
Activity* Scenario::getActivity(int i){
	return(activities[i]);
}

// supprime l'activite numero i
void Scenario::removeActivity(int i){
	activities.erase(activities.begin()+i);
}

// ferme of et proc
void Scenario::killOf(){
	of->killOf();
}


