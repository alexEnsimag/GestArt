/*
* Classe permettant l'affichage de la age d'accueil du jeu
*
*/


#ifndef DEF_MENU
#define DEF_MENU  

#include <gtkmm/window.h>  
#include <gtkmm/button.h>
#include <gtkmm/checkbutton.h>
#include <gtkmm/stock.h>
#include <gtkmm/image.h>
#include <gtkmm/box.h>
#include <gtkmm/main.h>
#include <gtkmm/filechooserdialog.h>
#include <gtkmm/filefilter.h>
#include <string>
#include <gtkmm/entry.h>
#include <gtkmm/messagedialog.h>
#include <gtkmm/label.h>

#include "Dialogue.hpp"
#include "../OpenGLViewer/ViewerJeux.hpp"
#include "../OpenGLViewer/ViewerTps.hpp"
#include "../../Model/Game/Game.hpp"

#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
#include "../../Controller/Kinect/Of.hpp"

class Menu;
#include "InterfaceG.hpp"

class Menu : public Gtk::Window {
	public : 
		Menu(int argc, char** argv, InterfaceG* const it, Game *g);
		~Menu();
	private :
		Gtk::HBox *boxH;
		Gtk::HBox *boxScenar;
		Gtk::VBox *boxVG;
		Gtk::VBox *boxVD;
		Gtk::Button *jouer;
		Gtk::Button *login;
		Gtk::Button *quitter;

		// nom du scenario
		Gtk::Label *scenarLabel;
		// passer a scenario suivant
		Gtk::Button *scenarG;
		// passer a scenario prec
		Gtk::Button *scenarD;
		int selectedScenar;

		Gtk::Image *imgMenu;
		Gtk::Image *imgSponsor;
		// pour la vue opengl
		//ViewerJeux *viewerJeux;
		//ViewerTps *viewerTps;
		InterfaceG *it;
		Game *game;
	
		void launch();
		void identification();
		void nextScen();
		void prevScen();
};
#endif
