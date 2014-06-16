
#include "GesturesMap.hpp"

using namespace std;

// Map numero, nom de mouvement
std::map<int, std::string> gests;

// nom du fichier contenant les mouvments
const string GesturesMap::gestFileName = "gestFile.txt";

// Ajoute les gestes du fichier dans la map gests
void GesturesMap::addGestesFile(){
     //Ouverture du fichier
     ifstream fichier(GesturesMap::gestFileName.c_str(), ios::in);

     if(!fichier){
         cout << "Erreur à l'ouverture du Fichier des gestes" << endl;
     } else {
         string name;
         int number;
	// lecture du fichier
         while(!fichier.eof()){
             fichier >> number;
		fichier >> name;
		// insertion dans la map
		gests.insert ( std::pair<int,string>(number,name) );
	}
     }
}


/*
* Ajoute un geste nomme name et son numero number correspondant au fichier et a la map
*/
void GesturesMap::addGest(string name, int number){
    // ajout a la map
    gests.insert ( std::pair<int,string>(number,name) );
    // ecriture dans le fichier
    const char * space = " ";
    const char * endLine = "\n";
    ofstream gestFile(GesturesMap::gestFileName.c_str(), ios_base::app);
    gestFile << number;
    gestFile << space;
    gestFile << name.c_str();
    gestFile << endLine;
    gestFile.close();
}

// Ajoute le nom d'un nouveau geste à la map
void GesturesMap::recordGeste(string name){
	addGest(name, gests.size() + 1);
}

// retourne le nom du geste de numero key
string GesturesMap::getGestByKey(int key) { 
	return gests[key]; 
}
	
// retourne le numero du geste de nom name
int GesturesMap::getGestByName(string name) { 
	std::map<int, std::string>::const_iterator it;
	int key = -1;

	for (it = gests.begin(); it != gests.end(); ++it) {
		if (it->second == name) {
			key = it->first;
			break;
		}
	}
	return key;
}

