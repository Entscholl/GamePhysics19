#pragma once
#include "util/vectorbase.h"
using namespace GamePhysics;

class Spring
{
public:
	Spring(int mass1, int mass2, float initLength, float stiff);
	~Spring();
	int masspoint1;
	int masspoint2;
	float stiffness;
	float initialLength;
	float currentLength;
	
};

