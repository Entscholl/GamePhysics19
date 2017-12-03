#include "RigidBodySystem.h"


RigidBodySystem::RigidBodySystem()
{
	m_Bodies.reserve(5);
}


RigidBodySystem::~RigidBodySystem()
{
}

void RigidBodySystem::SceneSetup(int flag)
{
	switch (flag) {
	case 0:
	{
		addRigidBody(Vec3(0,0,0),Vec3(1,0.6,0.5),2);
		setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)*0.25f));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		simulateTimestep(2, { 0,0,0 });
		Vec3 vel = getLinearVelocityOfRigidBody(0), ang = getAngularVelocityOfRigidBody(0);
		Vec3 point = {0.3,0.5,0.25};
		point = vel + cross(ang,point );
		printf("linear: (%f,%f,%f), angular: (%f,%f,%f), world_space_vel: (%f,%f,%f)\n",vel.x,vel.y,vel.z,ang.x,ang.y,ang.z,point.x,point.y,point.z);
	}
	break;
	default:
		break;
	}
}

void RigidBodySystem::simulateTimestep(float timeStep, Vec3 externalForce)
{
	for (auto value : m_Bodies) {
		value.pos += (timeStep*value.lin);
		value.lin += timeStep*(externalForce / value.mass);
		value.or += timeStep / 2 * Quat(0, value.ang.x, value.ang.y, value.ang.z) * value.or;
		value.L += timeStep*value.torque;
		Mat4d rotation = value.or.getRotMat(), transpose = rotation;
		transpose.transpose();
		value.Iinv = rotation* value.IbodyInv * transpose;
		value.ang = value.Iinv*value.L;
	}
}

int RigidBodySystem::getNumberOfRigidBodies()
{
	return m_Bodies.size();
}

Vec3 RigidBodySystem::getPositionOfRigidBody(int i)
{
	return m_Bodies[i].pos;
}

Vec3 RigidBodySystem::getLinearVelocityOfRigidBody(int i)
{
	return m_Bodies[i].lin;
}

Vec3 RigidBodySystem::getAngularVelocityOfRigidBody(int i)
{
	return m_Bodies[i].ang;
}

void RigidBodySystem::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_Bodies[i].force += force;
	m_Bodies[i].torque += cross(loc, force);
}

void RigidBodySystem::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody body = {};
	body.pos = position;
	body.mass = mass;
	body.lin = { 0,0,0 };
	body. or = { 0,0,0,0 };
	body.L = { 0,0,0 };
	body.IbodyInv.initId();
	body.IbodyInv.value[0][0] = 1.0 / 12.0*mass*(size.z*size.z + size.y*size.y);
	body.IbodyInv.value[1][1] = 1.0 / 12.0*mass*(size.x*size.x + size.y*size.y);
	body.IbodyInv.value[2][2] = 1.0 / 12.0*mass*(size.x*size.x + size.z*size.z);
	body.IbodyInv = body.IbodyInv.inverse();
	m_Bodies.push_back(RigidBody(body));
}

void RigidBodySystem::setOrientationOf(int i, Quat orientation)
{
	m_Bodies[i]. or = orientation;
}

void RigidBodySystem::setVelocityOf(int i, Vec3 velocity)
{
	m_Bodies[i].lin = velocity;
}
