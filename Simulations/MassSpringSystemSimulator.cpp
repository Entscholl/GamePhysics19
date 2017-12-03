#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_fMass      = 0.01f;
	m_fStiffness = 25.0f;
	m_fDamping   = 0.01f;
	m_iIntegrator = 1;
	m_iCube = 1;
	m_iTestCase = 1;
	m_externalForce = Vec3(0,0,0);
	m_pMassSpringSystem = new MassSpringSystem();
}

const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "BasicTest,Setup1,Setup2";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC	 =	DUC;
	switch (m_iTestCase)
	{
	case 0:
		break;
	case 1:
		{
			TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,LeapFrog,Midpoint");
			TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		}
		break;
	case 2:
		{
			TwAddVarRW(DUC->g_pTweakBar, "SpringCube", TW_TYPE_INT32, &m_iCube, "min = 1");
			TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,LeapFrog,Midpoint");
			TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
			TwAddVarRW(DUC->g_pTweakBar, "Mass",        TW_TYPE_FLOAT, &m_fMass,       "step=0.001  min=0.001");
			TwAddVarRW(DUC->g_pTweakBar, "Stiffness",   TW_TYPE_FLOAT, &m_fStiffness,  "step=0.001  min=0.001");
			TwAddVarRW(DUC->g_pTweakBar, "Damping",     TW_TYPE_FLOAT, &m_fDamping,    "step=0.001  min=0");

		}
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::reset(){
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_pMassSpringSystem->SetCube(m_iCube);
	m_pMassSpringSystem->SceneSetup(m_iTestCase);
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed){
	Vec3 pullforce(0,0,0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 forceView    = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 forceWorld   =	worldViewInv.transformVectorNormal(forceView);
		float forceScale = 0.03f;
		pullforce =  pullforce + (forceWorld * forceScale);
	}
	//pullforce -=  pullforce * 5.0f * timeElapsed;
	// Gravity
	Vec3 gravity = Vec3(0, -9.81f, 0);
	m_externalForce = gravity + pullforce;
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 1:
		{
			switch (m_iIntegrator)
			{
				case 0: m_pMassSpringSystem->AdvanceEuler(timeStep); break;
				case 1: m_pMassSpringSystem->AdvanceLeapFrog(timeStep); break;
				case 2: m_pMassSpringSystem->AdvanceMidPoint(timeStep); break;
			}
		}
		break;
	case 2:
		{
			
			m_pMassSpringSystem->SetGravity(m_externalForce);
			m_pMassSpringSystem->SetMass(m_fMass);
			m_pMassSpringSystem->SetStiffness(m_fStiffness);
			m_pMassSpringSystem->SetDamping(m_fDamping);
	
			switch (m_iIntegrator)
			{
				case 0: m_pMassSpringSystem->AdvanceEuler(timeStep); m_pMassSpringSystem->BoundingBoxCheck(); break;
				case 1: m_pMassSpringSystem->AdvanceLeapFrog(timeStep); m_pMassSpringSystem->BoundingBoxCheck(); break;
				case 2: m_pMassSpringSystem->AdvanceMidPoint(timeStep); m_pMassSpringSystem->BoundingBoxCheck(); break;
			}
		}
		break;
	default:
		break;
	}
}
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*Vec3(0.83,0.36,0.36));
	auto& points = m_pMassSpringSystem->GetPoints();
	float pointSize = 0.01f;

	for (size_t i=0; i<points.size(); i++)
	{
		DUC->drawSphere(points[i].pos, Vec3(pointSize,pointSize,pointSize));
	}

	DUC->beginLine();
	auto& springs = m_pMassSpringSystem->GetSprings();
	for(size_t i=0; i<springs.size(); i++)
	{
		Vec3 color = Vec3(0,0.4,0);
		DUC->drawLine(points[springs[i].point1].pos,color,points[springs[i].point2].pos,color);	
	}
	DUC->endLine();

}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
	m_pMassSpringSystem->SetStiffness(m_fStiffness);
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
	m_pMassSpringSystem->SetMass(m_fMass);
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
	m_pMassSpringSystem->SetDamping(m_fDamping);
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	int index = m_pMassSpringSystem->AddPoint(position,isFixed);
	m_pMassSpringSystem->SetPointVelocity(index, velocity);
	return index;	
}

void MassSpringSystemSimulator::addSpring(int index1, int index2, float initialLength)
{
	 m_pMassSpringSystem->AddSpring(index1,index2,initialLength);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_pMassSpringSystem->GetPoints().size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_pMassSpringSystem->GetSprings().size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_pMassSpringSystem->GetPoints()[index].pos;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_pMassSpringSystem->GetPoints()[index].vel;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
	m_pMassSpringSystem->SetGravity(m_externalForce.toDirectXVector());
}