
/*
* Classe permettant de lancer des videos.
*/

#ifndef DEF_VIDEO
#define DEF_VIDEO

#include <string>
#include <vlc/vlc.h>
#include <unistd.h>
#include <time.h>
#include <iostream>

class Video{

	public : 
		// lance la video videoName de duree durationMs
		static void launchVideo(std::string videoName, int durationMs);

};

#endif
