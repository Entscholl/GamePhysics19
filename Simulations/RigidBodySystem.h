#pragma once
#include "util\quaternion.h"
#include <vector>
using namespace GamePhysics;
class RigidBodySystem
{
public:
	struct RigidBody{
		Vec3 pos;
		Quat or ;
		Vec3 lin;
		Vec3 ang;
		int mass;
		Vec3 L;
		Vec3 torque;
		Mat4d IbodyInv;
		Mat4d Iinv;
		Vec3 force;
	};
	RigidBodySystem();
	~RigidBodySystem();
	void SceneSetup(int flag);
	void simulateTimestep(float timeStep, Vec3 externalForce);
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
private:
	std::vector<RigidBody> m_Bodies;
};