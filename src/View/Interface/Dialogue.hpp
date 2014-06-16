/*
* Classe permettant l'affichage d'une fenetre
* avec une zone de texte à remplir
* Elle hérite de la classe existante Gtk::Dialog pour l'affichage
* de pop-up
*/


#ifndef DEF_DIALOGUE
#define DEF_DIALOGUE
#include <string>
#include <gtkmm/box.h>
#include <gtkmm/button.h>
#include <gtkmm/dialog.h>
#include <gtkmm/entry.h>
#include <gtkmm/label.h>
#include <gtkmm/stock.h>
#include <gtkmm/window.h>

class Dialogue : public Gtk::Dialog {
    public :
        Dialogue(std::string title, Gtk::Window* parent, std::string message);
	// recupere le texte du dialogue
        std::string getText();
	// modifie le texte du dialogue
        void setText(std::string text);
        Gtk::Entry textZone;
        
    private :
        Gtk::Box* boxV;
        Gtk::Label label;
};
#endif
