#include "Video.hpp"

using namespace std;

// fonction utilitaire, permet d'attendre ms millisecondes
static void sleep2(unsigned int ms){
	clock_t goal = ms + clock();
	while(goal>clock());
}


/*
* Lance une video appele videoName de duree durationMs.
*/
void Video::launchVideo(string videoName, int durationMs){
	libvlc_instance_t * inst;
	libvlc_media_player_t *mp;
	libvlc_media_t *m;

	// Charge le VLC engine
	inst = libvlc_new (0, NULL);

	// Cree un nouveau item
	try{
		m = libvlc_media_new_path (inst, videoName.c_str());
	}catch (int e){
		cout << "Fichier non trouvé:" << videoName << endl;
		return;
	}

	// Cree l'environnement
	mp = libvlc_media_player_new_from_media (m);

	// Pas besoin de gardier le media pour l'instant
	libvlc_media_release (m);

	libvlc_set_fullscreen(mp, 1); 		

	// joue le media_player 

	libvlc_media_player_play (mp);
	// laisse jouer un peu
	sleep2(durationMs*1000); 

	// Arret de la video 
	libvlc_media_player_stop (mp);

	// Libere le media_player 
	libvlc_media_player_release (mp);

	libvlc_release (inst);

}





