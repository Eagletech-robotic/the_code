
// fonction de génération des sorties.
//  le but est de pouvoir tester les autres fonctions indépendement de la cible
//  et de générer les sorties de façon fixe temporrellement à chaque cycle

#include "output.h"
#include "motor.h"

void outputInit(output_t * output){
	output->ratio2 = 0;
	output->ratio15 = 0;
}

void outputSet(output_t *output) {
	motorSet(output->ratio2, output->ratio15);
}

