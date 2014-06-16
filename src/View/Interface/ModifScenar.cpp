#include "ModifScenar.hpp"


/*
* Constructeur: Cree la fenetre de modifications/ajout de scenarios
* Creer un nouveau scenario consiste a choisir un nom puis
* a ajouter des activites a ce scenario.
* Modifier un scenario consiste juste a ajouter des activites a celui-ci
*/
ModifScenar::ModifScenar(InterfaceG* const itG, Scenario *s){

	set_title("Gest-Art Application : Modification de scénario");
	set_icon_from_file("Images/icon.png");
	set_border_width(20);

	//Redimensionnement
	resize(500,500);
	//Positionnement
	set_position(Gtk::WIN_POS_CENTER);
	//Link vers le parent
	it = itG;
	if (s == NULL){
		scenar = new Scenario();
		scenar->setName("New_Scenario");
	}else{
		scenar = s;
	}
	//Création des widget
	boxH = new Gtk::HBox(false, 10);

	boxVG = new Gtk::VBox(false ,10);
	boxVM = new Gtk::VBox(false , 25);
	boxVD = new Gtk::VBox(false ,10);

	annuler = new Gtk::Button("Retour");
	annuler->signal_clicked().connect(sigc::mem_fun(*this, &ModifScenar::retDisplayScenar));

	valider = new Gtk::Button("Valider");
	valider->signal_clicked().connect(sigc::mem_fun(*this, &ModifScenar::validation));

	// choix d'une activite deja existante 
	newActivite = new Gtk::Label("Add an Activity:");

	// affichage des activites existantes dans une combo box
	listeActivite = new Gtk::ComboBoxText;
	loadActivities();
	listeActivite->signal_changed().connect(sigc::mem_fun(*this, &ModifScenar::updateParam));


	// ajout de l'activite
	validerActivite = new Gtk::Button("Ajouter l'activité");
	validerActivite->signal_clicked().connect(sigc::mem_fun(*this, &ModifScenar::addActivity));

	listeParam = new Gtk::ComboBoxText;

	// choix du nombre d'essai pour l'activite choisit
	trialsNumber = new Gtk::Entry();
	trialsNumber->set_text("nb essais");

	nomScenar = new Gtk::Entry();
	nomScenar->set_text(scenar->getName());

	//Ajout des Widgets
	boxH->pack_start(*boxVG,Gtk::PACK_SHRINK);
	boxH->pack_start(*boxVM);
	boxH->pack_start(*boxVD,Gtk::PACK_SHRINK);

	boxVG->pack_start(*nomScenar,Gtk::PACK_SHRINK);
	boxVG->pack_start(*newActivite,Gtk::PACK_SHRINK);
	boxVG->pack_start(*listeActivite,Gtk::PACK_SHRINK);
	boxVG->pack_start(*listeParam,Gtk::PACK_SHRINK);
	boxVG->pack_start(*trialsNumber,Gtk::PACK_SHRINK);
	boxVG->pack_start(*validerActivite);

	boxVG->pack_start(*annuler,Gtk::PACK_SHRINK);
	boxVG->pack_start(*valider,Gtk::PACK_SHRINK);

	for (int i=0; i<scenar->getNbActivities(); i++){
		Gtk::Label* lab = new Gtk::Label(scenar->getActivity(i)->toString());

		Gtk::Button* mod = new Gtk::Button("-");
		mod->signal_clicked().connect(sigc::bind<int>(sigc::mem_fun(*this, &ModifScenar::delActivity), i));

		boxVM->pack_start(*lab,Gtk::PACK_SHRINK);
		boxVD->pack_start(*mod,Gtk::PACK_SHRINK);
	}
	boxH->show();
	add(*boxH);
}

ModifScenar::~ModifScenar(){
	delete boxVD;
	delete boxVM;
	delete boxVG;
	delete boxH;
	delete annuler;
	delete valider;
	delete validerActivite;
	delete nomScenar;
	delete trialsNumber;
	delete listeActivite;
	delete listeParam;
}

// Retour aa la page de gestion des scenarios
void ModifScenar::retDisplayScenar(){
	it->retFromModifScenar();
}

// validation des modif
void ModifScenar::validation(){
	scenar->setName(nomScenar->get_text());
	scenar->record();
	it->maj();
	it->retFromModifScenar();
}

// suppression d'une activite a un scenario
void ModifScenar::delActivity(int i){
	scenar->removeActivity(i);
	reloadPage();
}

/*
*  Ajout de l'activite au scenario choisit
*/
void ModifScenar::addActivity(){
	string s = listeActivite->get_active_text();

	if(s == "ActiviteForme"){
		ShapeActivity *a = new ShapeActivity(listeParam->get_active_text(), atoi(trialsNumber->get_text().c_str()));
		scenar->addActivity(a);
	}else if(s== "ActiviteObjet"){
		ObjectActivity *a = new ObjectActivity(listeParam->get_active_text(), atoi(trialsNumber->get_text().c_str()));
		scenar->addActivity(a);
	}else{
		cout << "Ajout d'une activité inconnue" << endl;
	}
	scenar->setName(nomScenar->get_text());
	scenar->record();
	reloadPage();
}



void ModifScenar::reloadPage(){
	delete boxVD;
	delete boxVM;
	delete boxVG;
	delete boxH;
	delete annuler;
	delete valider;
	delete validerActivite;
	delete nomScenar;
	delete trialsNumber;
	delete listeActivite;
	delete listeParam;
	it->pageModifScenar(scenar);
}

/*
* Ajoute à la liste d'activite (combo box) les activites existantes
*/
void ModifScenar::loadActivities(){
	DIR* rep = opendir("Scenario/Activity/");
	if(rep == NULL){
		cout << "ERREUR DOSSIER SCENARIO NOT FOUND" << endl;
		return;
	}
	struct dirent* file = NULL;
	string comp = ".txt";
	string nomF;
	size_t found;
	string nomActivite;
	while ((file = readdir(rep)) != NULL){
		nomF = file->d_name;
		found = nomF.find(comp);
		if (found!=std::string::npos){
			nomActivite = nomF.substr(0,found);
			listeActivite->append(nomActivite);
		}
	}
	closedir(rep);
}

/**
* Ajoute à la liste de parametre (combo box) les parametres correspondant
* a l'activite choisie 
*/
void ModifScenar::updateParam(){
	string fileName = "Scenario/Activity/" + listeActivite->get_active_text() + ".txt";

	ifstream file(fileName.c_str(), ios::in);
	if(!file){
		cout << "Erreur à l'ouverture du Fichier" << endl;
		return;
	}
	string param;
	int duree;
	while(!file.eof()){
		file >> param >> duree;
		listeParam->append(param);
	}
	file.close();
}
