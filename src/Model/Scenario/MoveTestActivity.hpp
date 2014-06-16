/*
* Classe servant pour la reconnaissance d'un mouvement prealablement
* enregistre par l'administrateur. 
*/

#ifndef DEF_MOVETESTACTIVITY
#define DEF_MOVETESTACTIVITY

#include "Activity.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "../../View/Interface/Dialogue.hpp"
#include "../../Controller/Kinect/Osc.hpp"
#include "../../Controller/Kinect/Of.hpp"


class MoveTestActivity : Activity{
	public: 
		MoveTestActivity() : Activity("ActiviteTestMouv", "", 0){};
		// test si le mouvement gestureName est reconnu
		void tryGesture(std::string gestureName);
		// methodes non utiles ici
		virtual void init(){};
		virtual void exec(){};
		virtual void closeAct(){};
		void launch() { cout << "Activite Test, ne peut etre lancee !" << endl; };
	private:
};

#endif
