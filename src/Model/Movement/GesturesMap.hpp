/*
* Classe static : Permet la gestion des mouvements et de leur numero
* correspondant. 
* Openframework enregistre les mouvements avec des numero (classLabel). * Il est plus simple pour l'utilisateur de nomme des mouvements par des
* noms. Un fichier contient donc la correspondance nom de mouvment
* et numero. Cux ci sont enregistre dans une map et on peut ainsi
* facilement recuperer le nom ou le numero d'un geste.
*/

#ifndef GESTURESMAP_H
#define GESTURESMAP_H	
#include <iostream>
#include <fstream>
#include <string>
#include <map>

class GesturesMap{
	public:
		// ajoute un geste et son numero au fichier
		static void addGest(std::string name, int number);
		// 
		static void recordGeste(std::string name);
		// retourne le nom du mouvement a partir d'un numero
		static std::string getGestByKey(int key);	
		// retourne le numero du mouvement a partir de son nom
		static int getGestByName(std::string name);
		// Ajoute les gestes contenu dans le fichier dans la 
		// map
		static void addGestesFile();

		// nom du fichier contenant la correspondance
		// nomd'un mouvement numero
		static const std::string gestFileName;
};
#endif
