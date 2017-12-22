#include "SphereSystemSimulator.h"

std::function<float(float)> SphereSystemSimulator::m_Kernels[5] = {
	[](float x) {return 1.0f; },              // Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
};

// SphereSystemSimulator member functions

SphereSystemSimulator::SphereSystemSimulator()
{
	m_fDamping = 0.70f;
	m_fForceScaling = 10.0f;
	m_fMass = 0.10f;
	m_fRadius = 0.05f;
	m_iNumSpheres = 100;
	m_fGravity = 9.81;
	m_iKernel = 1;
	m_externalForce = Vec3(0, 0, 0);
	m_pSphereSystem = new SphereSystem();
	m_pSphereSystemGrid = new SphereSystem();
	m_pSphereSystemGrid->setStructure(1);
}
SphereSystemSimulator::~SphereSystemSimulator() {
	if (m_pSphereSystem) {
		delete m_pSphereSystem;
		m_pSphereSystem = 0;
	}
	if (m_pSphereSystemGrid) {
		delete m_pSphereSystemGrid;
		m_pSphereSystemGrid = 0;
	}
}

const char * SphereSystemSimulator::getTestCasesStr()
{
	return "Test1, Test2, Test3";
}

void SphereSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "step=0.001  min=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.001  min=0");
		TwAddVarRW(DUC->g_pTweakBar, "# of Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Kernel", TW_TYPE_INT32, &m_iKernel, "min=0 max=4");
		TwAddVarRW(DUC->g_pTweakBar, "ForceScaling", TW_TYPE_FLOAT, &m_fForceScaling, "step=0.001  min=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Radius", TW_TYPE_FLOAT, &m_fRadius, "step=0.001  min=0");
		break;
	case 1:
	{
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "step=0.001  min=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.001  min=0");
		TwAddVarRW(DUC->g_pTweakBar, "# of Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Kernel", TW_TYPE_INT32, &m_iKernel, "min=0 max=4");
		TwAddVarRW(DUC->g_pTweakBar, "ForceScaling", TW_TYPE_FLOAT, &m_fForceScaling, "step=0.001  min=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Radius", TW_TYPE_FLOAT, &m_fRadius, "step=0.001  min=0");

	}
	break;
	case 2:
	{
	}
	break;
	default:
		break;
	}
}

void SphereSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	if (m_iTestCase == 0 || m_iTestCase == 2) {
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.83, 0.36, 0.36));
		const Vec3 rad(m_fRadius, m_fRadius, m_fRadius);
		auto& spheres = m_pSphereSystem->getSpheres();
		for (size_t i = 0; i < spheres.size(); i++) {
			DUC->drawSphere(spheres[i].pos, rad);
		}
	}
	if (m_iTestCase == 1 || m_iTestCase == 2) {
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.33, 0.76, 0.36));
		const Vec3 rad(m_fRadius, m_fRadius, m_fRadius);
		auto& spheres = m_pSphereSystemGrid->getSpheres();
		for (size_t i = 0; i < spheres.size(); i++) {
			DUC->drawSphere(spheres[i].pos, rad);
		}
	}
}

void SphereSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	if (m_iTestCase == 0 || m_iTestCase == 2) {
		m_pSphereSystem->setDamping(m_fDamping);
		m_pSphereSystem->setMass(m_fMass);
		m_pSphereSystem->setGravity(m_externalForce);
		m_pSphereSystem->setForceScaling(m_fForceScaling);

		m_pSphereSystem->setRadius(m_fRadius);
		m_pSphereSystem->setSpheres(m_iNumSpheres);
		m_pSphereSystem->sceneSetup(m_iTestCase);
	}
	if (m_iTestCase == 1 || m_iTestCase == 2) {
		m_pSphereSystemGrid->setDamping(m_fDamping);
		m_pSphereSystemGrid->setMass(m_fMass);
		m_pSphereSystemGrid->setGravity(m_externalForce);
		m_pSphereSystemGrid->setForceScaling(m_fForceScaling);

		m_pSphereSystemGrid->setRadius(m_fRadius);
		m_pSphereSystemGrid->setSpheres(m_iNumSpheres);
		m_pSphereSystemGrid->sceneSetup(m_iTestCase);
	}
}

void SphereSystemSimulator::externalForcesCalculations(float timeElapsed)
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
	m_externalForce = gravity + pullforce;
}

void SphereSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0:
	{
		m_pSphereSystem->setDamping(m_fDamping);
		m_pSphereSystem->setMass(m_fMass);
		m_pSphereSystem->setGravity(m_externalForce);
		m_pSphereSystem->setForceScaling(m_fForceScaling);
		m_pSphereSystem->setRadius(m_fRadius);
		m_pSphereSystem->setSpheres(m_iNumSpheres);

		m_pSphereSystem->simulateTimestep(timeStep, m_Kernels[m_iKernel]);
		m_pSphereSystem->boundingBoxCheck(timeStep);
	} break;
	case 1:
	{
		m_pSphereSystemGrid->setDamping(m_fDamping);
		m_pSphereSystemGrid->setMass(m_fMass);
		m_pSphereSystemGrid->setGravity(m_externalForce);
		m_pSphereSystemGrid->setForceScaling(m_fForceScaling);
		m_pSphereSystemGrid->setRadius(m_fRadius);
		m_pSphereSystemGrid->setSpheres(m_iNumSpheres);

		m_pSphereSystemGrid->simulateTimestep(timeStep, m_Kernels[m_iKernel]);
		m_pSphereSystemGrid->boundingBoxCheck(timeStep);
	} break;
	case 2: {
		m_pSphereSystem->setGravity(m_externalForce);
		m_pSphereSystem->simulateTimestep(timeStep, m_Kernels[m_iKernel]);
		m_pSphereSystem->boundingBoxCheck(timeStep);

		m_pSphereSystemGrid->setGravity(m_externalForce);
		m_pSphereSystemGrid->simulateTimestep(timeStep, m_Kernels[m_iKernel]);
		m_pSphereSystemGrid->boundingBoxCheck(timeStep);
	} break;
	}

}

void SphereSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void SphereSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
