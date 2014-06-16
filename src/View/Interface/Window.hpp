/*
* Classe permettant l'affichage d'un message dans une fenetre 
*/

#ifndef DEF_WINDOW
#define DEF_WINDOW

#include <string>
#include <gtkmm.h>

class Window  {
    public :
	Gtk::Window win;
	void displayMessage(std::string s);
	
    private :
};
#endif
