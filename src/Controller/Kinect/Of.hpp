/*
* Classe permettant de lancer open framework pour la reconnaissance ou 
* l'enregistrement de gestes.
*/

#ifndef DEF_OF
#define DEF_OF

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include "Processing.hpp"


#include <string>


#define OF_PATH_RECOGNIZE "lib/OpenFrameworks/of_v0.8.1_linux64_release/apps/myApps/recognizeGesture/bin/"
#define OF_PATH_RECOGNIZE_HIDE "lib/OpenFrameworks/of_v0.8.1_linux64_release/apps/myApps/recognizeGestureHide/bin/"
#define OF_PATH_REGISTER "lib/OpenFrameworks/of_v0.8.1_linux64_release/apps/myApps/registerGesture/bin/"


class Of{
	public:
		// lance open framework et processing pour la reconnaissance
		void launchOfRecognize(bool showSkel, bool showOF);
		// lance open framework et processing pour l'enregistrement
		void launchOfRegister();
		// ferme openframework et processing
		void killOf();
	private:
		// pid de processing
		pid_t pidProc;
		// pid d'open framework
		pid_t pidOf;
};


#endif



