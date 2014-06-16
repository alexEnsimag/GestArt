#include "ShapeActivity.hpp"
#include "../../Controller/Kinect/Osc.hpp"
#include "InfoActivity.hpp"


using namespace std;

/*
* Initialisation de l'activite. Lancement des videos
*/
void ShapeActivity::init(){
	//wellDone = false;
	duration = DUR_RECO_GESTES;
	//string uri = "Video/" + getParam() + ".avi";
	//Video::launchVideo(uri, getDuree());
	
	Video::launchVideo("Video/RefaireGeste.avi", 8000);
}

/*
* Execution de l'activite. Lancement de Osc pour reconnaitre le 
* mouvement voulu
*/
void ShapeActivity::exec(){
	Osc *osc = new Osc();
	// recuperation du numero correspondanrt au gest
	int numLabel = GesturesMap::getGestByName(getParam());
	osc->launchOsc(numLabel, this);
}

/*
* Fermeture de l'activite avec lancement des videos selon la
* reussite ou l'echec de l'activite
*/
void ShapeActivity::closeAct(){
	if(getWellDone() == true){
		Video::launchVideo("Video/Bravo.avi", 3000);
	}else{
		Video::launchVideo("Video/Presque.avi", 5000);
	}
}

