/*
* Classe principale permettant de lancer l'application
*/

#include "../../View/Interface/InterfaceG.hpp"
int main(int argc, char** argv)
{
	//Création de l'interface Graphique
	InterfaceG *it = new InterfaceG(argc, argv);
	//Boucle principale
	it->run();
	//Destruction de l'interface
	delete it;
	return 0;
}
