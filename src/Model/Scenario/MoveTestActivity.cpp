
#include "MoveTestActivity.hpp"
#include "InfoActivity.hpp"

using namespace std;


/*
* Lance les applications necessaires afin de savoir si le mouvement
* appele textField est reconnu ou non
*/
void MoveTestActivity::tryGesture(string textField){
	duration = DUR_TEST_MOV;
	wellDone = false;
	int key = GesturesMap::getGestByName(textField);
	if(key == -1) {
		cout << "Geste inexistant" << endl;
		return;		
	} else {

		cout << "ClassLabel du geste : " << key << endl;		
		Of *of = new Of();
		of->launchOfRecognize(false, true); 


		Osc *ocs = new Osc();
		ocs->launchOsc(key, this); 
	}

}


