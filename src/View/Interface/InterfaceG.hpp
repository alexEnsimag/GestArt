/*
* Classe principale, qui permet de créer l'interface de jeu.
*
*/

#ifndef DEF_INTERFACE
#define DEF_INTERFACE

#include <gtkmm/main.h>
#include <GL/glew.h>

class InterfaceG;

#include "Menu.hpp"
#include "PageAdmin.hpp"
#include "ModifScenar.hpp"
#include "DisplayScenar.hpp"

class InterfaceG {
	public:
		InterfaceG(int argc, char** argv);
		void run();
		~InterfaceG();	
		void pageAdmin();
		void retMenuFromAdmin();
		void pageDisplayScenar();
		void retAdminFromScenar();
		void pageModifScenar();
		void pageModifScenar(Scenario *s);
		void retFromModifScenar();
		void maj();
	private:
		Gtk::Main main;
		// Interface d'entrée de jeu
		Menu *menu;
		// Interface administrateur
		PageAdmin *admin;
		// Interface de gestion des scenarios par l'admin
		DisplayScenar *displayScenar;
		// Interface de modifcation des scenarios
		ModifScenar *modifScenar;
		// Interface de jeu
		Game *game;
};
#endif
