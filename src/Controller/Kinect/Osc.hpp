/*
* Classe permettant de lancer le programme Osc servant à recuperer les
* donnees de processing lors de la reconnaissance de mouvements
*/

#ifndef DEF_OSC
#define DEF_OSC

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "../../Model/Scenario/Activity.hpp"
#include "../../Model/Scenario/MoveTestActivity.hpp"
#include "../../Model/Scenario/ReceiveClassLabel.hpp"




class Osc{
	public:
		// lance le programme
		void launchOsc(int classLabel, Activity *act);
	private:
};


#endif



