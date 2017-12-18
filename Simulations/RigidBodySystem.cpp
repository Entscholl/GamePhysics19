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
		deleteAll();
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0.0f, 0.5f, 1.0f), (float)(M_PI)*0.25f));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		simulateTimestep(2);
		Vec3 vel = getLinearVelocityOfRigidBody(0), ang = getAngularVelocityOfRigidBody(0);
		Vec3 point = { 0.3,0.5,0.25 };
		point -= getPositionOfRigidBody(0);
		point = vel + cross(ang, point);
		printf("linear: (%f,%f,%f), angular: (%f,%f,%f), world_space_vel: (%f,%f,%f)\n", vel.x, vel.y, vel.z, ang.x, ang.y, ang.z, point.x, point.y, point.z);
		deleteAll();
	}
	break;
	case 1:
	{
		deleteAll();
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)*0.25f));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 0, 0));
		simulateTimestep(2);
	}
	break;
	case 2:
	{
		deleteAll();
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0.0f, 0.5f, 1.0f), (float)(M_PI)*0.3f));
		applyForceOnBody(0, Vec3(0, 0, 0), Vec3(1, 0, 0));
		addRigidBody(Vec3(3, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)*0.25f));
		simulateTimestep(2);
	}
	break;
	case 3:
	{
		deleteAll();
		addRigidBody(Vec3(-2, -2, 0), Vec3(1, 1, 1), 2);
		setOrientationOf(0, Quat(Vec3(0.0f, 0.5f, 1.0f), (float)(M_PI)*0.25f));
		applyForceOnBody(0, Vec3(-2, -2, 0), Vec3(1, 1, 0));

		addRigidBody(Vec3(2, -2, 0), Vec3(1, 1, 1), 2);
		setOrientationOf(1, Quat(Vec3(0.0f, 0.5f, 1.0f), (float)(M_PI)*0.75f));
		applyForceOnBody(1, Vec3(-2, -2, 0), Vec3(-1, 1, 0));

		addRigidBody(Vec3(-2, 2, 0), Vec3(1, 1, 1), 2);
		setOrientationOf(2, Quat(Vec3(0.0f, 0.5f, 1.0f), (float)(M_PI)*0.75f));
		applyForceOnBody(2, Vec3(-2, -2, 0), Vec3(1, -1, 0));

		addRigidBody(Vec3(2, 2, 0), Vec3(1, 1, 1), 2);
		setOrientationOf(3, Quat(Vec3(0.0f, 0.5f, 1.0f), (float)(M_PI)*0.25f));
		applyForceOnBody(3, Vec3(-2, -2, 0), Vec3(-1, -1, 0));

		simulateTimestep(2);
	}
	break;
	default:
		break;
	}
}

void RigidBodySystem::deleteAll()
{
	m_Bodies.clear();
}
void RigidBodySystem::resolveCollision() {
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		//PositionMatrix
		GamePhysics::Mat4 pos;
		pos.initTranslation(getPositionOfRigidBody(i).x, getPositionOfRigidBody(i).y, getPositionOfRigidBody(i).z);
		//Scalematrix
		GamePhysics::Mat4 scale;
		scale.initScaling(getSizeOfRigidBody(i).x, getSizeOfRigidBody(i).y, getSizeOfRigidBody(i).z);
		//Rotationmatrix
		GamePhysics::Mat4 rot2,rot1 = getOrientation(i).getRotMat();
		Mat4 trans1 = rot1;
		trans1.transpose();
		GamePhysics::Mat4 endOne = scale * rot1 * pos;
		for (int j = i+1; j < getNumberOfRigidBodies(); j++) {
			//PositionMatrix
			pos.initTranslation(getPositionOfRigidBody(j).x, getPositionOfRigidBody(j).y, getPositionOfRigidBody(j).z);
			//Scalematrix
			scale.initScaling(getSizeOfRigidBody(j).x, getSizeOfRigidBody(j).y, getSizeOfRigidBody(j).z);
			//Rotationmatrix
			rot2 = getOrientation(j).getRotMat();
			Mat4 trans2 = rot2;
			trans2.transpose();
			GamePhysics::Mat4 endTwo = scale * rot2 * pos;
			CollisionInfo info = checkCollisionSAT(endOne, endTwo);
			if (info.isValid) {
				Mat4 TensorInverse1 = rot1 * m_Bodies[i].Iinv* trans1;
				Mat4 TensorInverse2 = rot2 * m_Bodies[j].Iinv* trans2;
				Vec3 ang1 = TensorInverse1.transformVector(m_Bodies[i].L);
				Vec3 ang2 = TensorInverse2.transformVector(m_Bodies[j].L);
				Vec3 vel1 = m_Bodies[i].lin + cross(ang1, info.collisionPointWorld-m_Bodies[i].pos);
				Vec3 vel2 = m_Bodies[j].lin + cross(ang2, info.collisionPointWorld - m_Bodies[j].pos);
				Vec3 vrel =  vel1 - vel2;
				Vec3 normal = info.normalWorld;
				float vrelnormal = dot(vrel, normal);
				if (vrelnormal <= 0.0f) {
					Real c = 0.0;
					Real J = -(1.0 + c)*vrelnormal / (1.0 / m_Bodies[i].mass + 1.0 / m_Bodies[j].mass + dot(
						cross(TensorInverse1.transformVector(cross(info.collisionPointWorld - m_Bodies[i].pos, normal)), info.collisionPointWorld - m_Bodies[i].pos) +
						cross(TensorInverse2.transformVector(cross(info.collisionPointWorld - m_Bodies[j].pos, normal)), info.collisionPointWorld - m_Bodies[j].pos),
						normal));
					m_Bodies[i].lin += J * normal * (1.0 / m_Bodies[i].mass);
					m_Bodies[j].lin -= J * normal * (1.0 / m_Bodies[j].mass);

					m_Bodies[i].L += cross(info.collisionPointWorld - m_Bodies[i].pos, J*normal);
					m_Bodies[j].L -= cross(info.collisionPointWorld - m_Bodies[j].pos, J*normal);
				}
			}
		}
	}
}
void RigidBodySystem::simulateTimestep(float timeStep)
{
	resolveCollision();
	for (int i = 0; i < m_Bodies.size(); i++) {
		m_Bodies[i].pos += (timeStep*m_Bodies[i].lin);
		m_Bodies[i].lin += timeStep*(m_Bodies[i].force / m_Bodies[i].mass);
		m_Bodies[i].force = { 0,0,0 };
		Mat4d rotation = m_Bodies[i]. or .getRotMat(), transpose = rotation;
		transpose.transpose();
		m_Bodies[i].Iinv = rotation* m_Bodies[i].IbodyInv * transpose;

		m_Bodies[i].ang = m_Bodies[i].Iinv.transformVector(m_Bodies[i].L);
		
		m_Bodies[i].L += timeStep*m_Bodies[i].torque;
		m_Bodies[i].torque = { 0,0,0 };
		m_Bodies[i]. or += timeStep / 2.0 * Quat(m_Bodies[i].ang.x, m_Bodies[i].ang.y, m_Bodies[i].ang.z, 0) * m_Bodies[i]. or ;
		m_Bodies[i]. or = m_Bodies[i]. or .unit();
	}
	
}

int RigidBodySystem::getNumberOfRigidBodies()
{
	return m_Bodies.size();
}

Vec3 RigidBodySystem::getPositionOfRigidBody(int i)
{
	if (m_Bodies.size() > i) {
		return m_Bodies[i].pos;
	}
	else
		return Vec3(0, 0, 0);
}

Vec3 RigidBodySystem::getLinearVelocityOfRigidBody(int i)
{
	if (m_Bodies.size() > i) {
		return m_Bodies[i].lin;
	}
	else
		return Vec3(0, 0, 0);
}

Vec3 RigidBodySystem::getAngularVelocityOfRigidBody(int i)
{
	if (m_Bodies.size() > i) {
		return m_Bodies[i].ang;
	}
	else
		return Vec3(0, 0, 0);
}

void RigidBodySystem::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	if (m_Bodies.size() > i) {
		m_Bodies[i].force += force;
		m_Bodies[i].torque += cross(loc, force);
	}
}

void RigidBodySystem::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody body = {};
	body.pos = position;
	body.mass = mass;
	body.lin = { 0,0,0 };
	body. or = { 0,0,0,0 };
	body.L = { 0,0,0 };
	body.size = size;
	body.IbodyInv.initId();
	body.IbodyInv.value[0][0] = 12.0/(mass*(size.z*size.z + size.y*size.y));
	body.IbodyInv.value[1][1] = 12.0/(mass*(size.x*size.x + size.y*size.y));
	body.IbodyInv.value[2][2] = 12.0/(mass*(size.x*size.x + size.z*size.z));
	body.IbodyInv.value[3][3] = 1.0;
	body.IbodyInv = body.IbodyInv.inverse();
	m_Bodies.push_back(RigidBody(body));
}

void RigidBodySystem::setOrientationOf(int i, Quat orientation)
{
	if (m_Bodies.size() > i) 
		m_Bodies[i]. or = orientation;
}

void RigidBodySystem::setVelocityOf(int i, Vec3 velocity)
{
	if (m_Bodies.size() > i) 
		m_Bodies[i].lin = velocity;
}
Vec3 RigidBodySystem::getSizeOfRigidBody(int i)
{
	if (m_Bodies.size() > i)
		return m_Bodies[i].size;
	else
		return Vec3(0, 0, 0);
}

Quat RigidBodySystem::getOrientation(int i)
{
	if (m_Bodies.size() > i)
		return m_Bodies[i]. or ;
	else
		return Quat(0, 0, 0, 0);
}
void RigidBodySystem::applyExternalForces(Vec3 externalForce) {
	for (int i = 0; i < m_Bodies.size(); i++) {
		applyForceOnBody(i, Vec3( 0,0,0 ) - m_Bodies[i].pos, externalForce);
	}
}