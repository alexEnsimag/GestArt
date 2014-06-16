/*
* Classe permettant l'affichage de la fenêtre d'ajout d'un nouveau
* scénario. Utilisation de la même fenetre pour la modification d'un
* d'un scenario existant
*/

#ifndef DEF_MODIF_SCENAR
#define DEF_MODIF_SCENAR

#include <gtkmm/window.h>  
#include <gtkmm/button.h>
#include <gtkmm/checkbutton.h>
#include <gtkmm/stock.h>
#include <gtkmm/image.h>
#include <gtkmm/box.h>
#include <gtkmm/label.h>
#include <gtkmm/entry.h>
#include <gtkmm/comboboxtext.h>

#include <iostream>
#include <unistd.h>
#include <stdlib.h>


class ModifScenar;
#include "InterfaceG.hpp"

class ModifScenar : public Gtk::Window {
	public : 
		ModifScenar(InterfaceG* const it, Scenario *s);
		~ModifScenar();

	private :
		Gtk::HBox *boxH;
		Gtk::VBox *boxVG;
		Gtk::VBox *boxVM;
		Gtk::VBox *boxVD;
		Gtk::Button *annuler;
		Gtk::Button *valider;
		Gtk::Button *validerActivite;
		Gtk::Entry *nomScenar;
		
		Gtk::Label *newActivite;
		// nb d'essai de l'activite
		Gtk::Entry *trialsNumber;
		// liste des activites existantes
		Gtk::ComboBoxText *listeActivite;
		// liste des parametres de l'activite choisit
		Gtk::ComboBoxText *listeParam;

		InterfaceG *it;
		Scenario *scenar;
	
		void updateParam();
		void retDisplayScenar();
		void validation();
		void delActivity(int i);
		void addActivity();
		void loadActivities();
		void reloadPage();
};
#endif
