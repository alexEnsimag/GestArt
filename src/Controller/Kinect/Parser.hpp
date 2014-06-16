/*
* Cette classe permet l'animation du squelette opengl en parsant le
* de mouvement chosit 
*/

#ifndef DEF_PARSER
#define DEF_PARSER

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>


#include <string>
#include "../../Model/OpenGL/Skeleton.hpp"

using namespace std;

// membres du squelette et correspondance des mots du fichier
#define HEAD "/head_pos_body"
#define MAING "/lefthand_pos_body"
#define MAIND "/righthand_pos_body"
#define SHOULDERL "/leftshoulder_pos_body"
#define SHOULDERR "/rightshoulder_pos_body"
#define ELBOWL "/leftelbow_pos_body"
#define ELBOWR "/rightelbow_pos_body"
#define TORSO "/torso_pos_body"
#define HIPR "/righthip_pos_body"
#define HIPL "/lefthip_pos_body"
#define KNEER "/rightknee_pos_body"
#define KNEEL "/leftknee_pos_body"
#define FOOTR "/rightfoot_pos_body"
#define FOOTL "/leftfoot_pos_body"

#define NBPOINTS 12

class Parser{
	public:
		// ouvre le fichier a parser
		void openFile(string fileName);
		// parse le fichier
		void parse(Skeleton *skeleton);
		// ferme le fichier a parser
		void close();
	private:	
		// fichier
		ifstream *file;
};


#endif



