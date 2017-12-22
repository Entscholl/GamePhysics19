#include "SphereSystem.h"
using namespace GamePhysics;


SphereSystem::SphereSystem()
{
	m_GridDimensions = { 1.0f,1.0f,1.0f };
	m_fRadius = 0.05f;
	int x = m_GridDimensions.x / (2.0*m_fRadius);
	int y = m_GridDimensions.y / (2.0 * m_fRadius);
	int z = m_GridDimensions.z / (2.0 * m_fRadius);
	m_iGridSize = x*y * z * 10;
	m_Grid.clear();
	for (int i = 0; i < x; i++) {
		std::vector< std::vector< std::vector<Sphere*> > > p;
		for (int j = 0; j < y; j++) {
			std::vector< std::vector<Sphere*> > g;
			for (int k = 0; k < z; k++) {
				std::vector<Sphere*> h;

				g.push_back(h);
			}
			p.push_back(g);
		}
		m_Grid.push_back(p);
	}
}


SphereSystem::~SphereSystem()
{
}

const std::vector<Sphere> & SphereSystem::getSpheres()
{
	return m_Spheres;
}

void SphereSystem::setRadius(float radius)
{
	if (m_fRadius != radius) {
		m_fRadius = radius;
		int x = m_GridDimensions.x / (2.0f*m_fRadius);
		int y = m_GridDimensions.y / (2.0f * m_fRadius);
		int z = m_GridDimensions.z / (2.0f * m_fRadius);
		m_iGridSize = x*y * z * 10;
		m_Grid.clear();
		for (int i = 0; i < x; i++) {
			std::vector< std::vector< std::vector<Sphere*> > > p(y);
			for (int j = 0; j < y; j++) {
				std::vector< std::vector<Sphere*> > g(z);
				for (int k = 0; k < z; k++) {
					std::vector<Sphere*> h(10);

					g.push_back(h);
				}
				p.push_back(g);
			}
			m_Grid.push_back(p);
		}
	}
}

void SphereSystem::setMass(float mass)
{
	m_fMass = mass;
}

void SphereSystem::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void SphereSystem::setDamping(float damping)
{
	m_fDamping = damping;
}

void SphereSystem::setGravity(Vec3 gravity)
{
	m_fGravity = gravity;
}

void SphereSystem::setForceScaling(float forceScaling)
{
	m_fForceScaling = forceScaling;
}
inline Vec3 projectUonV(const Vec3& u, const Vec3& v) {
	return v*(dot(u, v)*dot(v, v));
}

std::vector<Vec3> SphereSystem::computeForces(std::function<float(float)> kernel)
{
	std::vector<Vec3> forces(m_Spheres.size(), m_fGravity * m_fMass);
	switch (m_iStructure) {
	case 0: {
		float doubleR = m_fRadius * 2;
		for (size_t i = 0; i < m_Spheres.size(); i++)
		{
			Vec3 delta = { 0,0,0 };
			Vec3 posA = m_Spheres[i].pos;
			float distance = 0.0;
			for (size_t j = 0; j < m_Spheres.size(); j++) {
				Vec3 posB = m_Spheres[j].pos;
				delta = posB - posA;
				distance = normalize(delta);
				if (distance < doubleR) {
					Vec3 f = -delta* (m_fForceScaling*kernel(distance / doubleR));
					forces[i] += f;
					forces[j] -= f;
				}
			}
		}
	}break;
	case 1: {
		nVec4i entry;
		float xDim, yDim, zDim;
		xDim = m_GridDimensions.x / 2.0;
		yDim = m_GridDimensions.y / 2.0;
		zDim = m_GridDimensions.z / 2.0;
		float xDim2 = xDim / m_fRadius;
		float yDim2 = yDim / m_fRadius;
		float zDim2 = zDim / m_fRadius;
		int num;
		for (size_t i = 0; i < m_Spheres.size(); i++)
		{
			entry.x = (m_Spheres[i].pos.x + xDim)*xDim2;
			entry.y = (m_Spheres[i].pos.y + yDim)*yDim2;
			entry.z = (m_Spheres[i].pos.z + zDim)*zDim2;
			entry.t = m_Grid[entry.x][entry.y][entry.z].size();
			m_Grid[entry.x][entry.y][entry.z].push_back(&m_Spheres[i]);
			m_Entries.push_back(entry);
		}
		float doubleR = m_fRadius * 2;
		for (int i = 0; i < m_Entries.size(); i++) {
			Vec3 delta = { 0,0,0 };
			entry = m_Entries[i];
			Vec3 posA = m_Grid[entry.x][entry.y][entry.z][entry.t]->pos;
			float distance = 0.0;
			for (int z = (entry.z - 1) < 0 ? 0 : entry.z - 1; z <= (entry.z + 1 > m_Grid.size() ? m_Grid.size() : entry.z + 1); z++) {
				for (int y = (entry.y - 1) < 0 ? 0 : entry.y - 1; y <= (entry.y + 1 > m_Grid[z].size() ? m_Grid[z].size() : entry.y + 1); y++) {
					for (int x = (entry.x - 1) < 0 ? 0 : entry.x - 1; x <= (entry.x + 1 > m_Grid[z][y].size() ? m_Grid[z][y].size() : entry.x + 1); x++) {
						for (int t = 0; t< m_Grid[x][y][z].size(); t++) {
							if (z == entry.z && y == entry.y && x == entry.x && t == entry.t) break;
							Vec3 posB = m_Grid[x][y][z][t]->pos;
							delta = posB - posA;
							distance = normalize(delta);
							if (distance < doubleR) {
								Vec3 f = -delta* (m_fForceScaling*kernel(distance / doubleR));
								forces[m_Grid[entry.x][entry.y][entry.z][entry.t]->id] += f;
								forces[m_Grid[x][y][z][t]->id] -= f;
							}
						}
					}
				}
			}
		}
		for (int i = 0; i < m_Grid.size(); i++) {
			for (int j = 0; j < m_Grid[0].size(); j++) {
				for (int k = 0; k < m_Grid[0][0].size(); k++) {
					m_Grid[i][j][k].clear();
				}
			}
		}
		m_Entries.clear();
	} break;
	}
	// Damping forces
	for (size_t i = 0; i<m_Spheres.size(); i++)
	{
		Vec3 vel = m_Spheres[i].vel;

		forces[i] += vel * -m_fDamping;
	}
	return forces;
}

void SphereSystem::updatePositions(float time)
{
	for (size_t i = 0; i<m_Spheres.size(); i++)
	{
		Vec3 pos = m_Spheres[i].pos;
		Vec3 vel = m_Spheres[i].vel;
		pos += vel * time;
		m_Spheres[i].pos = pos;
	}
}

void SphereSystem::updateVelocities(float time, const std::vector<GamePhysics::Vec3>& forces)
{
	for (size_t i = 0; i<m_Spheres.size(); i++)
	{
		Vec3 vel = m_Spheres[i].vel;
		float m = m_fMass;

		vel += forces[i] * (time / m);

		m_Spheres[i].vel = vel;
	}
}

void SphereSystem::boundingBoxCheck(float times)
{
	for (size_t i = 0; i < m_Spheres.size(); i++)
	{
		Vec3 pos = m_Spheres[i].pos;
		Vec3 vel = m_Spheres[i].vel;

		for (int f = 0; f < 6; f++)
		{
			float sign = (!(f & 0x1)) ? -1.0f : 1.0f;
			if (sign * (pos.value[f >> 1] - sign * m_fRadius) < -0.5f)
			{
				pos.value[f >> 1] = sign * (-0.5f + m_fRadius);
				vel.value[f >> 1] = 0;
			}
		}
		m_Spheres[i].pos = pos;
		m_Spheres[i].vel = vel;
	}
}

void SphereSystem::setSpheres(int spheres)
{
	m_iSpheres = spheres;
	while (m_Spheres.size() > m_iSpheres) {
		m_Spheres.pop_back();
	}
	while (m_Spheres.size() < m_iSpheres) {
		Sphere s = { m_Spheres.size(), clamp(m_Spheres.size()),{ 0,0,0 } };
		m_Spheres.push_back(s);
	}
}

void SphereSystem::sceneSetup(int testCase)
{
	m_bLeapFirst = true;
	m_Spheres.clear();
	switch (testCase) {
	case 0: {
		for (int i = 0; i < m_iSpheres; i++) {
			Sphere s = { i,  clamp(i),{ 0,0,0 } };
			m_Spheres.push_back(s);
		}
	}
			break;
	case 1: {
		for (int i = 0; i < m_iSpheres; i++) {
			Sphere s = { i,{ 0,0,0 },{ 0,0,0 } };
			m_Spheres.push_back(s);
		}
	}
			break;
	case 2: {
		for (int i = 0; i < m_iSpheres; i++) {
			Sphere s = { i,{ 0,0,0 },{ 0,0,0 } };
			m_Spheres.push_back(s);
		}
	}
			break;
	}
}

void SphereSystem::setStructure(int structure)
{
	m_iStructure = structure;
}

void SphereSystem::simulateTimestep(float time, std::function<float(float)> kernel)
{
	std::vector<Vec3> forces = computeForces(kernel);
	if (m_bLeapFirst)
		updateVelocities(time*0.5, forces);
	else
		updateVelocities(time, forces);
	m_bLeapFirst = false;

	updatePositions(time);
}

void SphereSystem::resolveCollision(float time)
{
}

GamePhysics::Vec3 SphereSystem::clamp(int i)
{
	int denominator = (0.5 - 0.001) / (m_fRadius);
	if (denominator == 0) denominator = 1;
	Vec3 pos = { 0,0,0 };
	pos.x = -0.5 + m_fRadius + 0.001 + ((i % denominator) * 2 * m_fRadius);
	pos.z = -0.5 + m_fRadius + 0.001 + (((i % (denominator*denominator)) / (1.0* denominator)) * 2 * m_fRadius);
	pos.y = 0.5 - m_fRadius - 0.001 - ((i / (denominator*denominator)) * 2 * m_fRadius);
	return pos;
}
/*
pos.x = -0.5 + m_fRadius + 0.001 + ((m_Spheres.size() % 9) * 2 * m_fRadius);
pos.z = -0.5 + m_fRadius + 0.001 + (((m_Spheres.size() % 81) / 9.0) * 2 * m_fRadius);
pos.y = 0.5 - m_fRadius - 0.001 - ((m_Spheres.size() / 81.0) * 2 * m_fRadius);
*/