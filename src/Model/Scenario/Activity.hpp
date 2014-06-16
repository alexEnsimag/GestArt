/*
* Classe activite, ele contient toutes les fonctions communes
* a toutes les activites
* Une activite a un nom, un nom de parametre et un nombre d'essai
*/

#ifndef DEF_Activity
#define DEF_Activity

#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <fstream>
//#include <sstream>
#include <string>
#include <vector>

#include "../../View/Interface/Window.hpp"
#include "../../Controller/Video/Video.hpp"

#include "../../Model/Movement/GesturesMap.hpp"

class Activity {
	public:
		Activity(std::string n, std::string p,int nb);
		virtual ~Activity(){}
		// retourne le nom de l'activite
		std::string getName();
		// retourne le nom du param
		std::string getParam();
		// transforme en string l'activite
		std::string toString();
		// retourne nombre d'essai d'une activite
		int getTrialsNumber();
		
		int getDuree();
		// decremente le nombre d'essai d'une activite
		void decrementTrialsNumber();
		// retourne le booleen wellDone
		bool getWellDone();
		// modifie le booleen wellDone
		void setWellDone(bool b);
		// initialise l'activite : doit etre redefini dans chaque activite
		virtual void init();
		// lance l'activite
		void launch();
		// execute l'activite : doit atre redefini dans chaque activite
		virtual void exec();
		// ferme l'activite : doit etre redefini dans chaque activite
		virtual void closeAct();
		// affiche le message donne en parametre dans une nvlle fenetre
		void displayMessage(std::string s);
		int getDuration();
	protected:
		// booleen disant si l'activite est reussi ou non
		bool wellDone;
		// duree de l'Activite avant qu'elle soit considérée comme ratée, durée en ms
		int duration; 
	private:
		// nb d'essai
		int trialsNumber;
		// nom de l'activite
		std::string name;
		// nom du parametre
		std::string param;
		// duree de la video
		int duree;
		void chargerDuree();
		
};
#endif
