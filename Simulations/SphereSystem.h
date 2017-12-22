#pragma once
#include "util/vectorbase.h"
#include <vector>
#include <functional>
#include <list>
#include "util/vector4d.h"


struct Sphere {
	int id;
	GamePhysics::Vec3 pos;
	GamePhysics::Vec3 vel;
};
class SphereSystem
{
public:
	SphereSystem();
	~SphereSystem();
	const std::vector<Sphere>& getSpheres();
	void setRadius(float);
	void setMass(float);
	void setStiffness(float);
	void setDamping(float);
	void setGravity(GamePhysics::Vec3);
	void setForceScaling(float);
	std::vector<GamePhysics::Vec3> computeForces(std::function<float(float)>);
	void updatePositions(float time);
	void updateVelocities(float time, const std::vector<GamePhysics::Vec3>& forces);
	void boundingBoxCheck(float time);
	void setSpheres(int spheres);
	void sceneSetup(int testCase);
	void setStructure(int);
	void simulateTimestep(float, std::function<float(float)>);
	void resolveCollision(float time);
private:
	GamePhysics::Vec3 clamp(int);
	//void collide(Sphere &, Sphere&, GamePhysics::Vec3*, GamePhysics::Vec3*, std::function<float(float)>);

private:
	std::vector<Sphere> m_Spheres;
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	float m_fRadius = 0.049999f;
	float m_fForceScaling;
	int m_iStructure = 0;
	GamePhysics::Vec3 m_fGravity;
	bool m_bLeapFirst;
	int m_iSpheres;
	GamePhysics::Vec3 m_GridDimensions;
	std::vector< std::vector< std::vector< std::vector<Sphere*> > > > m_Grid;
	int m_iGridSize;
	std::vector<GamePhysics::nVec4i> m_Entries;
};

