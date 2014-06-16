/*
* Classe permettant l'affichage de l'interface de modification
* des scénarios. C'est une interace fille de l'interface administrateur.
*
*/

#ifndef DEF_AFFICHE_SCENAR
#define DEF_AFFICHE_SCENAR

#include <gtkmm/window.h>  
#include <gtkmm/button.h>
#include <gtkmm/checkbutton.h>
#include <gtkmm/stock.h>
#include <gtkmm/image.h>
#include <gtkmm/box.h>
#include <gtkmm/label.h>

#include <unistd.h>
#include <stdlib.h>


class DisplayScenar;
#include "InterfaceG.hpp"


class DisplayScenar : public Gtk::Window {
	public : 
		DisplayScenar(InterfaceG* const it, Game *g);
		~DisplayScenar();
	private :
		Gtk::HBox *boxH;
		Gtk::VBox *boxVG;
		Gtk::VBox *boxVM;
		Gtk::VBox *boxVD;
		Gtk::VBox *boxVED;
		Gtk::Button *retour;
		Gtk::Button *newScenar;

		Game *game;
		InterfaceG *it;
		// retour vers l'interface pere	
		void retAdmin();
		// modification d'un scenario
		void modifScenar(int i);
		// ajout d'un scenario
		void addScenar();
		// suppression d'un scenario
		void delScenar(int i);
};
#endif
