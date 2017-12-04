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
		simulateTimestep(2);
		Vec3 vel = getLinearVelocityOfRigidBody(0), ang = getAngularVelocityOfRigidBody(0);
		Vec3 point = {0.3,0.5,0.25};
		point -= getPositionOfRigidBody(0);
		point = vel + cross(ang,point );
		printf("linear: (%f,%f,%f), angular: (%f,%f,%f), world_space_vel: (%f,%f,%f)\n",vel.x,vel.y,vel.z,ang.x,ang.y,ang.z,point.x,point.y,point.z);
		
		addRigidBody(Vec3(1, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)*0.25f));
	}
	break;
	case 1:
	{
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)*0.25f));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 0, 0));
		simulateTimestep(2);
		Vec3 vel = getLinearVelocityOfRigidBody(0), ang = getAngularVelocityOfRigidBody(0);
		Vec3 point = { 0.3,0.5,0.25 };
		point -= getPositionOfRigidBody(0);
		point = vel + cross(ang, point);
		printf("linear: (%f,%f,%f), angular: (%f,%f,%f), world_space_vel: (%f,%f,%f)\n", vel.x, vel.y, vel.z, ang.x, ang.y, ang.z, point.x, point.y, point.z);

		addRigidBody(Vec3(1, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)*0.25f));
		//applyForceOnBody(1, Vec3(1,0,0), Vec3(-1, 0, 0)); 
	}
	break;
	default:
		break;
	}
}

void RigidBodySystem::simulateTimestep(float timeStep)
{
	for (int i = 0; i < m_Bodies.size(); i++) {
		m_Bodies[i].pos += (timeStep*m_Bodies[i].lin);
		m_Bodies[i].lin += timeStep*(m_Bodies[i].force / m_Bodies[i].mass);
		m_Bodies[i].force = { 0,0,0 };
		m_Bodies[i].or += timeStep / 2.0 * Quat(0, m_Bodies[i].ang.x, m_Bodies[i].ang.y, m_Bodies[i].ang.z) * m_Bodies[i].or;
		m_Bodies[i].or = m_Bodies[i].or.unit();
		m_Bodies[i].L += timeStep*m_Bodies[i].torque;
		m_Bodies[i].torque = { 0,0,0 };
		Mat4d rotation = m_Bodies[i].or.getRotMat(), transpose = rotation;
		transpose.transpose();
		m_Bodies[i].Iinv = rotation* m_Bodies[i].IbodyInv * transpose ;
		
		m_Bodies[i].ang = m_Bodies[i].Iinv.transformVector(m_Bodies[i].L);
	}
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		//PositionMatrix
		GamePhysics::Mat4 pos;
		pos.initTranslation(getPositionOfRigidBody(i).x, getPositionOfRigidBody(i).y, getPositionOfRigidBody(i).z);
		//Scalematrix
		GamePhysics::Mat4 scale;
		scale.initScaling(getSizeOfRigidBody(i).x, getSizeOfRigidBody(i).y, getSizeOfRigidBody(i).z);
		//Rotationmatrix
		GamePhysics::Mat4 rot = getOrientation(i).getRotMat();
		GamePhysics::Mat4 endOne = scale * rot * pos;
		for (int j = i+1; j < getNumberOfRigidBodies(); j++) {
			//PositionMatrix
			pos.initTranslation(getPositionOfRigidBody(j).x, getPositionOfRigidBody(j).y, getPositionOfRigidBody(j).z);
			//Scalematrix
			scale.initScaling(getSizeOfRigidBody(j).x, getSizeOfRigidBody(j).y, getSizeOfRigidBody(j).z);
			//Rotationmatrix
			rot = getOrientation(j).getRotMat();
			GamePhysics::Mat4 endTwo = scale * rot * pos;
			CollisionInfo info = checkCollisionSAT(endOne, endTwo);
			if (info.isValid) {
				Real c = 1.0;
				Vec3 vrel = m_Bodies[i].lin-m_Bodies[j].lin;
				Vec3 normal = -info.normalWorld;
				Real J = -(1.0 + c)*dot(vrel, normal)/( 1.0/m_Bodies[i].mass + 1.0 / m_Bodies[j].mass + dot(
					cross(m_Bodies[i].Iinv.transformVector(cross(m_Bodies[i].pos, normal)), m_Bodies[i].pos) +
					cross(m_Bodies[j].Iinv.transformVector(cross(m_Bodies[j].pos, normal)), m_Bodies[j].pos),
					normal));
				m_Bodies[i].lin += J * normal * (1.0/ m_Bodies[i].mass);
				m_Bodies[j].lin -= J * normal * (1.0 / m_Bodies[j].mass);
				
				m_Bodies[i].L += cross(m_Bodies[i].pos,J*normal);
				m_Bodies[j].L -= cross(m_Bodies[j].pos, J*normal);
			}
		}
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
	body.size = size;
	body.IbodyInv.initId();
	body.IbodyInv.value[0][0] = 1.0 / 12.0*mass*(size.z*size.z + size.y*size.y);
	body.IbodyInv.value[1][1] = 1.0 / 12.0*mass*(size.x*size.x + size.y*size.y);
	body.IbodyInv.value[2][2] = 1.0 / 12.0*mass*(size.x*size.x + size.z*size.z);
	body.IbodyInv.value[3][3] = 1.0;
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
Vec3 RigidBodySystem::getSizeOfRigidBody(int i)
{
	return m_Bodies[i].size;
}

Quat RigidBodySystem::getOrientation(int i)
{
	return m_Bodies[i]. or ;
}