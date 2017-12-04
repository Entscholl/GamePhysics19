#include "RigidBodySystemSimulator.h"
RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_pRigidBodySystem = new RigidBodySystem();
}

RigidBodySystemSimulator::~RigidBodySystemSimulator()
{
	if (m_pRigidBodySystem) {
		delete m_pRigidBodySystem;
		m_pRigidBodySystem = 0;
	}
}

const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "BasicTest,Setup1,Setup2,Setup3";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		break;
	case 1:
	{
	}
	break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::reset()
{
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4*Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
	for (int i = 0; i < m_pRigidBodySystem->getNumberOfRigidBodies(); i++) {
		//PositionMatrix
		GamePhysics::Mat4* pos = new Mat4();
		pos->initTranslation(m_pRigidBodySystem->getPositionOfRigidBody(i).x, m_pRigidBodySystem->getPositionOfRigidBody(i).y, m_pRigidBodySystem->getPositionOfRigidBody(i).z);
		//Scalematrix
		GamePhysics::Mat4* scale = new Mat4();
		scale->initScaling(m_pRigidBodySystem->getSizeOfRigidBody(i).x, m_pRigidBodySystem->getSizeOfRigidBody(i).y, m_pRigidBodySystem->getSizeOfRigidBody(i).z);
		//Rotationmatrix
		GamePhysics::Mat4 rot = m_pRigidBodySystem->getOrientation(i).getRotMat();
		DUC->drawRigidBody(*scale * rot * *pos);
		delete pos;
		delete scale;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_pRigidBodySystem->SceneSetup(testCase);
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Vec3 pullforce(0, 0, 0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 forceView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 forceWorld = worldViewInv.transformVectorNormal(forceView);
		float forceScale = 0.03f;
		pullforce = pullforce + (forceWorld * forceScale);
	}
	//pullforce -=  pullforce * 5.0f * timeElapsed;
	// Gravity
	Vec3 gravity = Vec3(0, -9.81f, 0);
	m_externalForce = pullforce;
	int closest = 0;
	float length = FLT_MAX;
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		if (sqrt(getPositionOfRigidBody(i).squaredDistanceTo(Vec3(m_trackmouse.x, m_trackmouse.y, 0))) < length) {
			closest = i;
			length = sqrt(getPositionOfRigidBody(i).squaredDistanceTo(Vec3(m_trackmouse.x, m_trackmouse.y, 0)));
		}
	}
	m_pRigidBodySystem->applyForceOnBody(closest, Vec3(m_trackmouse.x, m_trackmouse.y, getPositionOfRigidBody(closest).z), m_externalForce);
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	externalForcesCalculations(timeStep);
	m_pRigidBodySystem->simulateTimestep(timeStep);
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_pRigidBodySystem->getNumberOfRigidBodies();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_pRigidBodySystem->getPositionOfRigidBody(i);
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->getLinearVelocityOfRigidBody(i);
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->getAngularVelocityOfRigidBody(i);
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_pRigidBodySystem->applyForceOnBody(i, loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_pRigidBodySystem->addRigidBody(position, size, mass);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_pRigidBodySystem->setOrientationOf(i, orientation);
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_pRigidBodySystem->setVelocityOf(i, velocity);
}
