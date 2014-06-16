#include "DisplayScenar.hpp"

/*
* Constructeur: Cree la fenetre de gestion des scenarios
*
*/
DisplayScenar::DisplayScenar(InterfaceG* const itG, Game *g){

		set_title("Gest-Art Application : Scénarios");
		set_icon_from_file("Images/icon.png");
		set_border_width(20);

		//Redimensionnement
		resize(500,500);
		//Positionnement
	 	set_position(Gtk::WIN_POS_CENTER);
		//Link vers le parent
		it = itG;
		game = g;

		//Création des widget
		boxH = new Gtk::HBox(false, 10);

		boxVG = new Gtk::VBox(false ,10);
		boxVM = new Gtk::VBox(false ,25);
		boxVD = new Gtk::VBox(false ,10);
		boxVED = new Gtk::VBox(false ,10);

		// bouton retour vers page admin
		retour = new Gtk::Button("Retour");
		retour->signal_clicked().connect(sigc::mem_fun(*this, &DisplayScenar::retAdmin));
		// bonton d'ajout d'un nouveau scenario
		newScenar = new Gtk::Button("New Scénario");
		newScenar->signal_clicked().connect(sigc::mem_fun(*this, &DisplayScenar::addScenar));
		

		//Ajout des Widgets
		boxH->pack_start(*boxVG,Gtk::PACK_SHRINK);
		boxH->pack_start(*boxVM);
		boxH->pack_start(*boxVD,Gtk::PACK_SHRINK);
		boxH->pack_start(*boxVED,Gtk::PACK_SHRINK);

		boxVG->pack_start(*retour,Gtk::PACK_SHRINK);
		boxVG->pack_start(*newScenar,Gtk::PACK_SHRINK);

		// Parcourt des scenarios existants
		for (int i=0; i<game->getNbScenar(); i++){
			Gtk::Label* lab = new Gtk::Label(game->getScenar(i)->getName());

			// bouton pour modifier chaque scenar existant
			Gtk::Button* mod = new Gtk::Button("modifier");
			mod->signal_clicked().connect(sigc::bind<int>(sigc::mem_fun(*this, &DisplayScenar::modifScenar), i));
			// bouton pour la suppression
			Gtk::Button* del = new Gtk::Button("-");
			del->signal_clicked().connect(sigc::bind<int>(sigc::mem_fun(*this, &DisplayScenar::delScenar), i));
			
			boxVM->pack_start(*lab,Gtk::PACK_SHRINK);
			boxVD->pack_start(*mod,Gtk::PACK_SHRINK);
			boxVED->pack_start(*del,Gtk::PACK_SHRINK);
		}

		boxH->show();
		add(*boxH);
}

DisplayScenar::~DisplayScenar(){
	delete boxVD;
	delete boxVM;
	delete boxVG;
	delete boxH;
	delete retour;
	delete newScenar;
}

// Retour a la page admin
void DisplayScenar::retAdmin(){
	it->retAdminFromScenar();
}

// Redirection vers la page d'ajout d'un scenario
void DisplayScenar::addScenar(){
	it->pageModifScenar();
}

// Redirection vers la page de modification d'un scenario
void DisplayScenar::modifScenar(int i){
	it->pageModifScenar(game->getScenar(i));
}

// Supprime un scenario de la liste des scenarios du jeu
// Maj et retour vers la page de gestion des scenarios
void DisplayScenar::delScenar(int i){
	game->delScenar(i);
	it->pageDisplayScenar();
}
