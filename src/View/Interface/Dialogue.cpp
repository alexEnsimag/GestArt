
#include "Dialogue.hpp"

/*
* Creation de la boite de dialogue avec une zone texte et quelques
* boutons
*/
Dialogue::Dialogue(std::string title, Gtk::Window* parent, std::string message) : Gtk::Dialog(title, *parent), boxV(get_vbox()), label(message) { //Récupération de la boîte verticale avec get_vbox().
    boxV->pack_start(label);
    boxV->pack_start(textZone);
    //Ajout de boutons à la boîte de dialogue.
    add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
    add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);
    show_all();
}

/*
* Permet de recuperer le texte de la boite de dialogue
*/
std::string Dialogue::getText() {
    return textZone.get_text();
}

/*
* Permet de modifier le texte de la boite de dialogue
*/
void Dialogue::setText(std::string text) {
    textZone.set_text(text);
}
