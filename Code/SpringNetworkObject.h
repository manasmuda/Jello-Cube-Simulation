#ifndef __SPRING_NETWORK_OBJECT
#define __SPRING_NETWORK_OBJECT

#include <iostream>
#include <string>
#include "StringUtils.h"

using namespace std;

struct world;

struct SpringNetworkObject
{

	const int ID_LENGTH = 10;

public:

	SpringNetworkObject() : SpringNetworkObject(nullptr) { }

	SpringNetworkObject(world* world) {
		this->id = generateRandomString(ID_LENGTH);
		this->curWorld = world;
	}

	string id;
	world* curWorld;

};

#endif
