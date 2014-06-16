#ifndef ADMIN_H
#define ADMIN_H	
#include <string>
using namespace std;
#include <map>
//#include <pair>
class Admin{
	public:
		Admin(); 
		void addGest(string name, int number);
		void enregistrementGeste(string name);
		string getGestByKey(int key);	
		int getGestByName(string name);

	private:	
		string identifiant;
		string motDePasse;
		string gestFileName;
		std::map<int, string> gests;
};
#endif
