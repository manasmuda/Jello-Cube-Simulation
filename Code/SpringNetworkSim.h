#ifndef __SPRING_NETWORK_SIM
#define __SPRING_NETWORK_SIM

#include <vector>
#include <unordered_map>
#include <string>
#include "jello.h"
#include <iostream>
#include <algorithm>

#include <chrono>

using namespace std;
using namespace std::chrono;

struct world;

struct SpringNetworkSim
{

private:
	struct CollissionData {
		point normal;
		double length;

		CollissionData() : length(0), normal() {}

		bool isValid() { return length > 0; }
	};

public:
	SpringNetworkSim(world* world);

	/*void addObject(SpringNetworkObject* obj);
	SpringNetworkObject* getObject(string id);
	bool removeObject(string id);
	bool removeObject(SpringNetworkObject* obj);
	bool hasObject(string id);*/

	void updateTime();
	void updateSim();
	Hit getRayCastPoint(point rayOrigin, point rayDir);

	private:
	void computeAcceleration(struct world::object* obj, struct point***& a);
	point getForceOnPoint(struct world::object* obj, int i, int j, int k);

	point getForceFieldAtPoint(point position);

	CollissionData getCollissionDataWithBoundingBox(struct world::object* obj, point p);
	CollissionData getCollissionDataWithObject(struct world::object* obj, struct world::object* collisionObj, point p);

	// perform one step of Euler and Runge-Kutta-4th-order integrators
	// updates the jello structure accordingly
	void Euler(struct world::object* obj);
	void RK4(struct world::object* obj);


	public:
	steady_clock::time_point lastTime;
	double deltaTime;

	world* curWorld;
	world orgWorld;


	/*private:
	unordered_map<string, SpringNetworkObject*> objMap;*/


};

#endif