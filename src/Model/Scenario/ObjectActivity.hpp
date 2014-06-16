/*
* Classe representant une activite objet : activite qui consiste
* a reconnaitre le QrCode d'un objet presente a la kinect. 
*/

#ifndef DEF_OBJECTACTIVITY
#define DEF_OBJECTACTIVITY

#include "Activity.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>


class ObjectActivity : public Activity {
	public: 
		ObjectActivity(std::string p, int nb) : Activity("ActiviteObjet",p, nb){}
		// initialisation de l'activite objet
		virtual void init();
		// execution de l'activité
		virtual void exec();
		// fermeture de l'activite
		virtual void closeAct();

	private:
};

#endif
