#ifndef MASSSPRINGSYSTEM_H
#define MASSSPRINGSYSTEM_H

#include <vector>
#include "util\vectorbase.h"
#include <iostream>
using namespace GamePhysics;
using std::cout;

class MassSpringSystem
{
public:
	struct Point
	{
		Vec3 pos;
		Vec3 vel;
		bool fixed;
	};	

	struct Spring
	{
		int point1;
		int point2;
		float initialLength;
	};	

	MassSpringSystem() : m_cube(0), m_leapfrogFirst(false){}

	int AddPoint(const Vec3& pos, bool fixed)
	{
		Point p;
		p.pos = pos;
		p.vel = Vec3(0,0,0);
		p.fixed = fixed;
		m_points.push_back(p);

		return (int)m_points.size()-1;
	}

	void AddSpring(int p1, int p2, float initialLength)
	{
		Spring s;
		s.point1 = p1;
		s.point2 = p2;
		s.initialLength = initialLength;
		m_springs.push_back(s);
	}

	void AddSpring(int p1, int p2)
	{
		Vec3 d = m_points[p1].pos - m_points[p2].pos;
		AddSpring(p1, p2, norm(d));
	}

	void SetMass     (float mass     ) {m_mass      = mass     ;}
	void SetStiffness(float stiffness) {m_stiffness = stiffness;}
	void SetDamping  (float damping  ) {m_damping   = damping  ;}
	void SetCube	 (int   cube	 ) {m_cube = cube;			}

	void SetGravity  (const Vec3& gravity) {m_gravity = gravity;}

	const std::vector<Point>&  GetPoints()  {return m_points; }
	const std::vector<Spring>& GetSprings() {return m_springs;}

	void AdvanceEuler(float dt);
	void AdvanceLeapFrog(float dt);
	void AdvanceMidPoint(float dt);

	void BoundingBoxCheck(float times = 1.0f);

	void SetPointVelocity(int p, const Vec3& vel){
		m_points[p].vel = vel;
	}

	void SetPointPosition(int p, const Vec3& pos){
		m_points[p].pos = pos;
	}

	void PrintPoint(size_t p){
		cout << "point " << p << " vel (" << m_points[p].vel.x << ", " << m_points[p].vel.y << ", " << m_points[p].vel.z 
			<< "), pos (" << m_points[p].pos.x << ", " << m_points[p].pos.y << ", " << m_points[p].pos.z << ") \n";
	}

	void SceneSetup(int sceneflag);

private:
	std::vector<Vec3> ComputeForces();
	void UpdatePositions(float dt);
	void UpdateVelocities(float dt, const std::vector<Vec3>& forces);

	std::vector<Point>  m_points;
	std::vector<Spring> m_springs;

	float m_mass;
	float m_stiffness;
	float m_damping;

	int  m_cube;

	Vec3 m_gravity;

	bool m_leapfrogFirst;
};
#endif