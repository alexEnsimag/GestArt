#include "Menu.hpp"


/*
* Constructeur: Cree la fenetre d'accueil du game
*/
Menu::Menu(int argc, char** argv, InterfaceG* const itG, Game* g){

		set_title("Gest-Art Application");
		set_icon_from_file("Images/icon.png");
		set_border_width(20);
		selectedScenar = 0;

		//Redimensionnement
		resize(700,700);
		//Positionnement
	 	set_position(Gtk::WIN_POS_CENTER);
		
		//Link avec l'interface mère
		it = itG;

		//Ajout du lien avec le game:
		game = g;
		
		//Creation de la vue openGL
		//viewerJeux = new ViewerJeux(argc, argv);
		//viewerTps = new ViewerTps();

		//Création des widget
		boxH = new Gtk::HBox(false, 10);
		boxScenar = new Gtk::HBox (false,10);

		boxVG = new Gtk::VBox(false ,10);
		boxVD = new Gtk::VBox(false ,10);

		// Affichage des scenarios pouvant etre joues
		scenarLabel = new Gtk::Label(game->getScenar(0)->getName());
		scenarG = new Gtk::Button ("<");
		scenarG->signal_clicked().connect(sigc::mem_fun(*this, &Menu::prevScen));
		scenarD = new Gtk::Button (">");
		scenarD->signal_clicked().connect(sigc::mem_fun(*this, &Menu::nextScen));

		
		// bouton pour jouer le scenario
jouer = new Gtk::Button("JOUER");
		jouer->signal_clicked().connect(sigc::mem_fun(*this, &Menu::launch));
		// s'identifier cad aller vers pageAdmin
		login = new Gtk::Button("S'identifier");
		login->signal_clicked().connect(sigc::mem_fun(*this, &Menu::identification));
		
		quitter = new Gtk::Button(Gtk::Stock::QUIT);
		quitter->signal_clicked().connect(sigc::ptr_fun(&Gtk::Main::quit));
		imgMenu = new Gtk::Image("Images/menu.png");
		imgSponsor = new Gtk::Image("Images/sponsor.png");

		//Ajout des Widgets
		boxH->pack_start(*boxVG,Gtk::PACK_SHRINK);
		boxH->pack_start(*boxVD);

		boxVG->pack_start(*login,Gtk::PACK_SHRINK);
		boxVG->pack_start(*quitter,Gtk::PACK_SHRINK);
		//boxVG->pack_start(*imgSponsor);

		boxVD->pack_start(*imgMenu);
		boxVD->pack_start(*boxScenar,Gtk::PACK_SHRINK);
		boxVD->pack_start(*jouer);

		boxScenar->pack_start(*scenarG,Gtk::PACK_SHRINK);
		boxScenar->pack_start(*scenarLabel);
		boxScenar->pack_start(*scenarD,Gtk::PACK_SHRINK);
		boxH->show();
		add(*boxH);
}

Menu::~Menu(){
	delete jouer;
	delete scenarG;
	delete scenarD;
	delete scenarLabel;
	delete login;
	delete quitter;
	delete imgMenu;
	delete imgSponsor;
	delete boxVD;
	delete boxVG;
	delete boxScenar;
	delete boxH;
}

// lancement du game avec le scenario choisit 
void Menu::launch(){
	game->launch(selectedScenar);
}

/*
* Affichage d'une fenetre de dialogue pour rentrer le 
* mdp admin
*/
void Menu::identification(){
	string texteF;
	Dialogue diag("Acces interface Administrateur", this, "Veuillez entrer le code admministrateur");
	diag.setText("admin");
	diag.textZone.set_visibility(false);
	int response = diag.run();
	if(response == Gtk::RESPONSE_OK) { 
        	texteF = diag.getText();
	if(texteF == "admin"){
		diag.hide();
		it->pageAdmin();
	}else{
		Gtk::MessageDialog diagE(*this, "Erreur de code", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_CLOSE);
		int response = diagE.run();
		identification();
	}
    	}
}

// Affichage du scenario suivant
void Menu::nextScen(){
	selectedScenar +=1;
	selectedScenar %= game->getNbScenar(); 
	scenarLabel->set_text(game->getScenar(selectedScenar)->getName());
}

/*
* Affichage du scenario precedent
*/
void Menu::prevScen(){
	selectedScenar += game->getNbScenar() - 1 ;
	selectedScenar %= game->getNbScenar(); 
	scenarLabel->set_text(game->getScenar(selectedScenar)->getName());
}

