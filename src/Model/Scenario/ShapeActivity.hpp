/*
* Classe representant une activite formes. Une activite formes consiste
* a recfonnaitre un mouvement fait devant la kinect.
* Les differents parametres possibles de sont les différents noms de geste de la base.
*/


#ifndef DEF_SHAPEACTIVITY
#define DEF_SHAPEACTIVITY

#include "Activity.hpp"
#include <string>
#include "../../Controller/Kinect/Processing.hpp"
#include <gtkmm/messagedialog.h>

class ShapeActivity : public Activity {
		public: 
			ShapeActivity(string p, int nbE) : Activity("ActiviteForme", p, nbE){};
			// initialisation de l'activite
			virtual void init();
			// execution de l'activite
			virtual void exec();
			// fermeture de l'activite
			virtual void closeAct();
			void update(string classLabel);

		private:
};

#endif
