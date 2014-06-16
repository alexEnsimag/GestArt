
#include "Window.hpp"

using namespace std;

/*
* Affichage du message donné en paramètre dans une nouvelle
* fenetre
*/
void Window::displayMessage(string s){
	win.set_border_width(50);
	Gtk::Label lab(s);
	win.add(lab);
	//fen.resize(100, 50);
	win.show_all();
	Gtk::Main::run(win);
}
