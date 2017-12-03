#include "masspoint.h"
using namespace GamePhysics;


masspoint::masspoint(Vec3 pos, Vec3 vel, bool fixed, float m)
{
	position = pos;
	velocity = vel;
	isFixed = fixed;
	mass = m;
	vel_init = vel;
}



masspoint::~masspoint()
{
}
