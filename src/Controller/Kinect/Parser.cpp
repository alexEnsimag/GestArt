#include "Parser.hpp"



/*
* Ouvre le fichier a parser
*/
void Parser::openFile(string fileName) {
	file = new ifstream(fileName.c_str(), ios::in);
}

/*
* Parse le fichier et met a jour les positions des membres du squelette
*/
void Parser::parse(Skeleton *skeleton){

	string mot1;
	float pos1, pos2, pos3;

	// parse le fichier
	for(int i=0; i<NBPOINTS; i++){
		*file >> mot1 >> pos1 >> pos2 >> pos3;
		Vector3f p;
		// maj des coordonnes
		p.x = pos1/1000;
		p.y = pos2/1000;
		p.z = (pos3-3000)/1500;
		if(! mot1.compare(MAING)){
			skeleton->setHandD(p);
		}else if(! mot1.compare(MAIND)){
			skeleton->setHandG(p);
		}else if(! mot1.compare(SHOULDERL)){
			skeleton->setShoulderD(p);
		}else if(! mot1.compare(SHOULDERR)){
			skeleton->setShoulderG(p);
		}else if(! mot1.compare(ELBOWL)){
			skeleton->setElbowD(p);
		}else if(! mot1.compare(ELBOWR)){
			skeleton->setElbowG(p);
		}else if(! mot1.compare(TORSO)){
			skeleton->setChest(p);
		}else if(! mot1.compare(HIPL)){
			skeleton->setPelvisD(p);
		}else if(! mot1.compare(HIPR)){
			skeleton->setPelvisG(p);
		}else if(! mot1.compare(KNEEL)){
			skeleton->setKneexD(p);
		}else if(! mot1.compare(KNEER)){
			skeleton->setKneexG(p);
		}else if(! mot1.compare(FOOTL)){
			skeleton->setFootD(p);
		}else if(! mot1.compare(FOOTR)){
			skeleton->setFootG(p);
		}else if(! mot1.compare(HEAD)){
			skeleton->setHead(p);
		}else{
		} 
	}
}

/*
* Ferme le fichier a parser
*/
void Parser::close(){
	file->close();
}


