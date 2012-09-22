#ifndef ClearPhysicsSDK_H__
#define ClearPhysicsSDK_H__

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>

#ifdef CLEARPHYSICSSDK_EXPORTS
	#define CLEAR_PHYSICS_API __declspec(dllexport)
#else
	#define CLEAR_PHYSICS_API __declspec(dllimport)
#endif

#pragma warning (disable: 4005)
#pragma warning (disable: 4275)

// STL
#include <string>
#include <list>
#include <queue>
#include <vector>
#include <algorithm>
#include <map>
#include <set>
#include <iterator>
#include <fstream>
#include <iostream>

using namespace std;


//#define OLD_BULLET_VERSION
#ifdef OLD_BULLET_VERSION

#pragma comment(lib, "libbulletcollision_d.lib")
#pragma comment(lib, "libbulletdynamics_d.lib")
#pragma comment(lib, "libbulletmath_d.lib")

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <Bullet-C-Api.h>


#else // OLD_BULLET_VERSION

	#pragma comment(lib, "BulletCollision_Debug.lib")
	#pragma comment(lib, "BulletDynamics_Debug.lib")
	#pragma comment(lib, "LinearMath_Debug.lib")

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <Bullet-C-Api.h>

#endif




#include <cgl.h>
#pragma comment(lib, "ClearGraphicsLibrary.lib")

#include <BlocoCore.h>
#pragma comment(lib, "BlocoCore.lib")

#include "PhyEvents.h"

#include "DebugDrawer.h"

#include "Constants.h"
#include "MathConversion.h"
#include "NullPhysics.h"
#include "BulletPhysics.h"
#include "PhysicsEventListener.h"

#endif

