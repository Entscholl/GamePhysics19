#include "Spring.h"
#include "util/vectorbase.h"
using namespace GamePhysics;


Spring::Spring(int mass1, int mass2, float initLength, float stiff)
{
	masspoint1 = mass1;
	masspoint2 = mass2;
	initialLength = initLength;
	stiffness = stiff;
}


Spring::~Spring()
{
}

