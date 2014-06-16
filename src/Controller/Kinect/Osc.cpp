
#include "Osc.hpp"


/*
* Lance le programme Osc qui permet de savoir quand of reconnait 
* le mouvement de classe label et de mettre a jour le boolean wellDone
* de l'activite correspondante.
*/
void Osc::launchOsc(int classLabel, Activity *act){
	// creation d'un listener
	ExamplePacketListener listener;
	// maj classLabel
	listener.setClassLabel(classLabel);
	UdpListeningReceiveSocket s(
			IpEndpointName( IpEndpointName::ANY_ADDRESS, PORT ),
			&listener );    
	listener.setS(&s);
	// maj activite correspondante
	listener.setActivite(act);
	s.Run();
}
