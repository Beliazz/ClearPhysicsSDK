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
#include <assert.h>

using namespace std;

#pragma warning( disable : 4251 )

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

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <Bullet-C-Api.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>


#endif

#include <BlocoCore\include\Vec.h>
#include <3dParty\FastDeligate\include\FastDelegate.h>

#include "IPhysic.h"

#include "Constants.h"
#include "MathConversion.h"
#include "NullPhysics.h"

#include "DynamicMotionState.h"
#include "KinematicMotionState.h"
#include "BulletPhysics.h"

#endif

