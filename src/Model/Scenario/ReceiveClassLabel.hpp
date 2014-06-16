#ifndef DEF_RECEIVECLASSLABEL
#define DEF_RECEIVECLASSLABEL

#include <iostream>
#include <cstring>


#include "../../osc/OscReceivedElements.h"
#include "../../osc/OscPacketListener.h"
#include "../../osc/UdpSocket.h"

#include "ShapeActivity.hpp"
#include "../../View/Interface/PageAdmin.hpp"


/*
Classe issue d'un example d'oscpack

Récupère les messages envoyés par OpenFrameworks,
messages correspondant aux gestes reconnus


L'écoute des messages est stoppée dès que le geste attendu
est reçu
*/

#define PORT 12346

class ExamplePacketListener : public osc::OscPacketListener {
public:
	void setS(UdpListeningReceiveSocket *s) {
		time = 0;
		_s = s;
	}

	void setActivite(Activity *a) {
		_activity = a;
		duration = a->getDuration();
	}
	
	void setClassLabel(int num){
		_classLabel = num;
	}

protected:
	UdpListeningReceiveSocket * _s;
	Activity * _activity; // activité attendant une reconnaissance de geste
	int _classLabel; // numero du geste attendu
	int time; // temps écoulé depuis le début de l'essai
	int duration; // durée de l'essai avant qu'il soit considéré comme échoué
   
    // Traitement des messages
    virtual void ProcessMessage( const osc::ReceivedMessage& m, 
				const IpEndpointName& remoteEndpoint )
    {
        (void) remoteEndpoint; // suppress unused parameter warning

        try{
            // osc::OsckPacketListener handles the bundle traversal.
			osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
			// Test de reconnaissance du geste attendu
			// Récupération du paramètre du message
			std::string classLabel = (arg++)->AsString();
			// Conversion du _classLabel en string
			cout << "Geste detecte : " << classLabel << endl;
			char * s;
			sprintf(s, "%d",_classLabel);
			if(strcmp(classLabel.c_str(), s) == 0) {
				std::cout << "ClassLabel : " << classLabel << "\n";
				_activity->setWellDone(true);
				sleep(2); // attente pour laisser le temps au joueur de finir le geste
				_s->Break(); // arrêt de l'écoute
			// Test si le temps imparti est terminé
			}else if(duration != 0) {
				time++;
				if(time == duration){
				    		time = 0;
						_s->Break();
				}

			}
		

        }catch( osc::Exception& e ){
            // any parsing errors such as unexpected argument types, or 
            // missing arguments get thrown as exceptions.
            //std::cout << m.AddressPattern() << ": " << e.what() << "\n";
        }
    }
};


#endif
