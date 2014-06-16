#!/bin/sh
#Script de compilation et de lancement du projet

help(){
  echo    "Utilisation: \n"
  echo    "./run.sh [-h] [-c] [-r] [-e]"
  echo    "l’ordre des options n'a pas d'importance -r et -e sont incompatibles"
  echo    "h: help"
  echo    "c: Compilation uniquement"
  echo    "e: Execution uniquement"
  echo    "r: Création ou Recréation des dossier necessaires à la compilation\n"
  exit
}

while getopts :hcer opt
do
	case $opt in
	h) #option "h" help
	opt_H="true" ;;
	c) #option "c" Compilation
	opt_C="true" ;;
	e) #option "e" Execution
	opt_E="true" ;;
	r) #option "r" reset
	opt_R="true" ;;
	*)
	echo "Mauvais arguments";
	help;
	exit 1;;
	esac
done
test "$opt_H" && help
if test "$opt_R" = "true"
then
	rm -r build/
	mkdir build
	cd build
	cmake ..
	cd ..
fi

if test "$opt_E" != "true"
then
	cd build
	make
	cd ..
fi

if test "$opt_C" != "true"
then
	./build/Gest-Art_App
fi
