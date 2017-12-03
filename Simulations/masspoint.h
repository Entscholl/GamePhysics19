#pragma once
#include "util/vectorbase.h"
using namespace GamePhysics;

class masspoint
{
public:
	masspoint(Vec3 pos, Vec3 vel, bool fixed, float m);
	~masspoint();
	Vec3 position;
	Vec3 velocity;
	Vec3 vel_init;//initial velocity for leapfrog 
	Vec3 force;
	float damping;
	float mass;
	bool isFixed;
};

