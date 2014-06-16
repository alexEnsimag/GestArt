/*
* Definit un jeu. Un jeu contuen une liste de scneario
*/

#ifndef DEF_GAME
#define DEF_GAME

#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>
#include "../../Model/Scenario/Scenario.hpp"

#ifndef WIN32
	#include <sys/types.h>
#endif

class Game{
	public:
		Game();
		// retourne le nombre de scenario
		int getNbScenar();
		// retourne le scenario numero i
		Scenario* getScenar(int i);
		// lance le scenario
		void launch(int i);
		// maj du scenario
		void updateScenar();
		// supprime le scenario numero i
		void delScenar(int i);

	private:	
		// liste de scenario
		std::vector<Scenario> scenar;
};

#endif
