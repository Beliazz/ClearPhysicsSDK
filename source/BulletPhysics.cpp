#include "ClearPhysicsSDK.h"

struct MaterialData
{
	float m_restitution;
	float m_friction;
};

MaterialData g_PhysicsMaterials[] = 
{
	// restitution       friction
	{     0.05f,           0.9f     },			// playdough
	{     0.25f,           0.5f     },			// a 'normal' material
	{     0.95f,           0.5f     },          // a 'bouncy' material
	{     0.25f,           0.0f     },          // a 'slippery' material
};

cpl::BulletPhysics::BulletPhysics()
{
	// auto_ptr<> will automatically initialize themselves to NULL
	m_bDebugDrawWorld = false;
}

cpl::BulletPhysics::~BulletPhysics()
{
	// delete any physics objects which are still in the world


	// iterate backwards because removing the last object doesn't affect the
	//  other objects stored in a vector-type array
	for ( int ii=m_dynamicsWorld->getNumCollisionObjects()-1; ii>=0; --ii )
	{
		btCollisionObject * const obj = m_dynamicsWorld->getCollisionObjectArray()[ii];

		RemoveCollisionObject( obj );
	}

	// destroy all the BulletActor objects
	for (std::map<ActorId,map<ComponentId,btRigidBody*>>::iterator it = m_actorMap.begin(); it != m_actorMap.end(); ++it)
	{
		for (std::map<ComponentId,btRigidBody*>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2)
		{
			delete it2->second;

		}
	}
	m_actorMap.clear();
}

bool cpl::BulletPhysics::VInitialize()
{
	// VInitialize creates the components that Bullet uses

	// this controls how Bullet does internal memory management during the collision pass
	m_collisionConfiguration.reset( new btDefaultCollisionConfiguration() );

	// this manages how Bullet detects precise collisions between pairs of objects
	m_dispatcher.reset( new btCollisionDispatcher( m_collisionConfiguration.get() ) );

	// Bullet uses this to quickly (imprecisely) detect collisions between objects.
	//   Once a possible collision passes the broad phase, it will be passed to the
	//   slower but more precise narrow-phase collision detection (btCollisionDispatcher).
	m_broadphase.reset( new btDbvtBroadphase() );

	// Manages constraints which apply forces to the physics simulation.  Used
	//  for e.g. springs, motors.  We don't use any constraints right now.
	m_solver.reset( new btSequentialImpulseConstraintSolver );

	// This is the main Bullet interface point.  Pass in all these components to customize its behavior.
	m_dynamicsWorld.reset( new btDiscreteDynamicsWorld( m_dispatcher.get(), 
		m_broadphase.get(), 
		m_solver.get(), 
		m_collisionConfiguration.get() ) );

	// and set the internal tick callback to our own method "BulletInternalTickCallback"
	m_dynamicsWorld->setInternalTickCallback( BulletInternalTickCallback );
	m_dynamicsWorld->setWorldUserInfo( this );
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher.get());


	m_bRun = true;

	m_fStepSimulationTime = 0.0f;
	m_fSyncVisualSceneTime = 0.0f;
	m_fUpdateKinematicControllerTime = 0.0f;
	m_fDebugDrawWorldTime = 0.0f;

	return true;
}

void cpl::BulletPhysics::VOnUpdate( float const deltaSeconds )
{
	m_dynamicsWorld->stepSimulation(1.f/60.f);

	if (m_bDebugDrawWorld)
	{
		m_dynamicsWorld->debugDrawWorld();
	}

	//m_dynamicsWorld->stepSimulation(deltaSeconds,10);
}

void cpl::BulletPhysics::AddShape( ActorId actorID, ComponentId componentID, Mat initialTransform, btCollisionShape * const shape, ePhysicBodyType type, btScalar const mass, enum PhysicsMaterial const mat )
{
	btRigidBody * body = NULL;

	if (type == ePhysicBodyType_Dynamic )
	{
		btVector3 localInertia( 0.f, 0.f, 0.f );
		shape->calculateLocalInertia( mass, localInertia );

		btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, new CDynamicMotionState( actorID, componentID, initialTransform, fastdelegate::MakeDelegate(this, &cpl::BulletPhysics::OnMoved) ), shape, localInertia );

		// set up the materal properties
		rbInfo.m_restitution	 = g_PhysicsMaterials[mat].m_restitution;
		rbInfo.m_friction		 = g_PhysicsMaterials[mat].m_friction;

		body = new btRigidBody(rbInfo);

		m_dynamicsWorld->addRigidBody( body );
	}
	else if ( mass == 0.0f || type == ePhysicBodyType_Kinematic )
	{
		btVector3 localInertia( 0.f, 0.f, 0.f );
		btRigidBody::btRigidBodyConstructionInfo rbInfo( mass,new CDynamicMotionState( actorID, componentID, initialTransform, fastdelegate::MakeDelegate(this, &cpl::BulletPhysics::OnMoved)), shape,localInertia);

		// set up the materal properties
		rbInfo.m_restitution	 = g_PhysicsMaterials[mat].m_restitution;
		rbInfo.m_friction		 = g_PhysicsMaterials[mat].m_friction;

		body = new btRigidBody(rbInfo);

  		body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);

		m_dynamicsWorld->addRigidBody( body );
	}

	pair<ActorId, ComponentId> idPair = pair<ActorId, ComponentId>(actorID,componentID);
	m_actorMap[actorID][componentID]  = body;
	m_rigidBodyToActorId[body]		  = idPair;
}

void cpl::BulletPhysics::AddShape( ActorId actorID, ComponentId componentID, Mat initialTransform, btCollisionShape * shape, ePhysicBodyType type, short group, short mask, btScalar mass, enum PhysicsMaterial mat )
{
	btRigidBody * body = NULL;

	if (type == ePhysicBodyType_Dynamic )
	{
		btVector3 localInertia( 0.f, 0.f, 0.f );
		shape->calculateLocalInertia( mass, localInertia );

		btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, new CDynamicMotionState( actorID, componentID, initialTransform, fastdelegate::MakeDelegate(this, &cpl::BulletPhysics::OnMoved)), shape, localInertia );

		// set up the materal properties
		rbInfo.m_restitution	 = g_PhysicsMaterials[mat].m_restitution;
		rbInfo.m_friction		 = g_PhysicsMaterials[mat].m_friction;

		body = new btRigidBody(rbInfo);

		m_dynamicsWorld->addRigidBody( body, group, mask );
	}
	else if ( mass == 0.0f || type == ePhysicBodyType_Kinematic )
	{
		btVector3 localInertia( 0.f, 0.f, 0.f );
		btRigidBody::btRigidBodyConstructionInfo rbInfo( mass,new CDynamicMotionState( actorID, componentID, initialTransform, fastdelegate::MakeDelegate(this, &cpl::BulletPhysics::OnMoved)), shape,localInertia);

		// set up the materal properties
		rbInfo.m_restitution	 = g_PhysicsMaterials[mat].m_restitution;
		rbInfo.m_friction		 = g_PhysicsMaterials[mat].m_friction;

		body = new btRigidBody(rbInfo);

		body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
		//   		body->setActivationState(DISABLE_DEACTIVATION);

		m_dynamicsWorld->addRigidBody( body, group, mask);
	}

	pair<ActorId, ComponentId> idPair = pair<ActorId, ComponentId>(actorID,componentID);
	m_actorMap[actorID][componentID]  = body;
	m_rigidBodyToActorId[body]		  = idPair;
}

void cpl::BulletPhysics::VAddPlane( Vec planeNormal, float planeConstante,
							   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat )
{
	btCollisionShape* planeShape = new btStaticPlaneShape(Vec_to_btVector3(planeNormal),planeConstante);

	AddShape( actorID, componentID, initialTransform, planeShape, type, 0.0f, mat );
}

void cpl::BulletPhysics::VAddPlane( Vec planeNormal, float planeConstante,
							   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat )
{
	btCollisionShape* planeShape = new btStaticPlaneShape(Vec_to_btVector3(planeNormal),planeConstante);

	AddShape( actorID, componentID, initialTransform, planeShape, type, group, mask, 0.0f, mat );
}

void cpl::BulletPhysics::VAddSphere( float const radius,
							    ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat )
{
	// create the collision body, which specifies the shape of the object
	btSphereShape * const collisionShape = new btSphereShape( radius );

	// calculate absolute mass from specificGravity
	float const volume = (4.f / 3.f) * XM_PI * radius * radius * radius;
	btScalar const mass = volume * specificGravity;

	AddShape( actorID, componentID, initialTransform, collisionShape, type, mass, mat );
}

void cpl::BulletPhysics::VAddSphere( float radius,
							    ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat )
{
	// create the collision body, which specifies the shape of the object
	btSphereShape * const collisionShape = new btSphereShape( radius );

	// calculate absolute mass from specificGravity
	float const volume = (4.f / 3.f) * XM_PI * radius * radius * radius;
	btScalar const mass = volume * specificGravity;

	AddShape( actorID, componentID, initialTransform, collisionShape,type, group, mask, mass, mat );
}

void cpl::BulletPhysics::VAddBox( Vec dimensions,
							 ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat)
{
	// 	// create the collision body, which specifies the shape of the object
	btBoxShape * const collisionShape = new btBoxShape( Vec_to_btVector3( dimensions ) );

	// calculate absolute mass from specificGravity
	float const volume = dimensions.GetX() * dimensions.GetY() * dimensions.GetZ();
	btScalar const mass = volume * specificGravity;

	AddShape( actorID, componentID, initialTransform, collisionShape, type, mass, mat );
}

void cpl::BulletPhysics::VAddBox( Vec dimensions,
							 ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat )
{
	// 	// create the collision body, which specifies the shape of the object
	btBoxShape * const collisionShape = new btBoxShape( Vec_to_btVector3( dimensions ) );

	// calculate absolute mass from specificGravity
	float const volume = dimensions.GetX() * dimensions.GetY() * dimensions.GetZ();
	btScalar const mass = volume * specificGravity;

	AddShape( actorID, componentID, initialTransform, collisionShape, type, group, mask, mass, mat );
}

void cpl::BulletPhysics::VAddCapsule( float radius, float height,
								 ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat )
{
	// 	// create the collision body, which specifies the shape of the object
	btCapsuleShape * const capsule = new btCapsuleShape( radius, height );

	// calculate absolute mass from specificGravity
	float const volume = radius*radius*XM_PIDIV4 * height + radius*radius*radius*XM_PIDIV4*3 ;
	btScalar const mass = volume * specificGravity;

	AddShape( actorID, componentID, initialTransform, capsule, type, mass, mat );
}

void cpl::BulletPhysics::VAddCapsule( float radius, float height,
								 ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat )
{
	// 	// create the collision body, which specifies the shape of the object
	btCapsuleShape * const capsule = new btCapsuleShape( radius, height );

	// calculate absolute mass from specificGravity
	float const volume = radius*radius*XM_PIDIV4 * height + radius*radius*radius*XM_PIDIV4*3 ;
	btScalar const mass = volume * specificGravity;

	AddShape( actorID, componentID, initialTransform, capsule, type, group, mask, mass, mat );
}

void cpl::BulletPhysics::VAddMesh( float* pVertices, unsigned int numVertices, DWORD* pIndices, unsigned int numIndices,
							  ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat )
{
	btTriangleIndexVertexArray* pMesh = new btTriangleIndexVertexArray( numIndices/3, (int*)pIndices, sizeof(DWORD)*3, numVertices, (btScalar*)pVertices, sizeof(float)*3 );

	btGImpactMeshShape * pShape = new btGImpactMeshShape(pMesh);
	pShape->updateBound();


	// approximate absolute mass using bounding box
	btVector3 aabbMin(0,0,0), aabbMax(0,0,0);
	pShape->getAabb( btTransform::getIdentity(), aabbMin, aabbMax );

	btVector3 const aabbExtents = aabbMax - aabbMin;

	float const volume = aabbExtents.x() * aabbExtents.y() * aabbExtents.z();
	btScalar const mass = volume * specificGravity;

	AddShape( actorID, componentID, initialTransform, pShape, type, mass, mat );
}

void cpl::BulletPhysics::VAddMesh( float* pVertices, unsigned int numVertices, DWORD* pIndices, unsigned int numIndices,
							  ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat )
{
	btTriangleIndexVertexArray* pMesh = new btTriangleIndexVertexArray( numIndices/3, (int*)pIndices, sizeof(DWORD)*3, numVertices, (btScalar*)pVertices, sizeof(float)*3 );

	btGImpactMeshShape * pShape = new btGImpactMeshShape(pMesh);
	pShape->updateBound();

	// approximate absolute mass using bounding box
	btVector3 aabbMin(0,0,0), aabbMax(0,0,0);
	pShape->getAabb( btTransform::getIdentity(), aabbMin, aabbMax );

	btVector3 const aabbExtents = aabbMax - aabbMin;

	float const volume = aabbExtents.x() * aabbExtents.y() * aabbExtents.z();
	btScalar const mass = volume * specificGravity;

	AddShape( actorID, componentID, initialTransform, pShape, type, group, mask, mass, mat );
}

void cpl::BulletPhysics::VRemoveActor(ActorId id)
{
	printf("BulletPhysics::VRemoveActor(), Not implemented \n");

// 	if ( btRigidBody * const body = FindActorBody( id ) )
// 	{
// 		// destroy the body and all its components
// 		RemoveCollisionObject( body );
// 
// 		// clear the relevant elements from the lookup maps
// 		ActorIDToBulletActorMap::iterator it = m_actorBodies.find(id);
// 		if (it != m_actorBodies.end())
// 		{
// 			BulletActor* pDead = it->second;
// 			delete pDead;
// 			m_actorBodies.erase(it);
// 		}
// 		m_rigidBodyToActorId.erase( body );
// 	}
}

void cpl::BulletPhysics::VRemoveComponent( ActorId id, ComponentId compId )
{
	printf("BulletPhysics::VRemoveActor(), Not implemented \n");

	// 	if ( btRigidBody * const body = FindActorBody( id ) )
	// 	{
	// 		// destroy the body and all its components
	// 		RemoveCollisionObject( body );
	// 
	// 		// clear the relevant elements from the lookup maps
	// 		ActorIDToBulletActorMap::iterator it = m_actorBodies.find(id);
	// 		if (it != m_actorBodies.end())
	// 		{
	// 			BulletActor* pDead = it->second;
	// 			delete pDead;
	// 			m_actorBodies.erase(it);
	// 		}
	// 		m_rigidBodyToActorId.erase( body );
	// 	}
}

void cpl::BulletPhysics::VCreateTrigger(Vec pos, const float dim, int triggerID)
{
	printf("BulletPhysics::VCreateTrigger(), Not implemented \n");

// 	// create the collision body, which specifies the shape of the object
// 	btBoxShape * const boxShape = new btBoxShape( Vec_to_btVector3( Vec(dim,dim,dim) ) );
// 
// 	// triggers are immoveable.  0 mass signals this to Bullet.
// 	btScalar const mass = 0;
// 
// 	// set the initial position of the body from the actor
// 	Mat triggerTrans = MatIdentity();
// 	MatSetPosition(triggerTrans, pos);
// 	ActorMotionState * const myMotionState = DEBUG_CLIENTBLOCK ActorMotionState( triggerTrans );
// 
// 	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, boxShape, btVector3(0,0,0) );
// 	btRigidBody * const body = new btRigidBody(rbInfo);
// 
// 	m_dynamicsWorld->addRigidBody( body );
// 
// 	// a trigger is just a box that doesn't collide with anything.  That's what "CF_NO_CONTACT_RESPONSE" indicates.
// 	body->setCollisionFlags( body->getCollisionFlags() | btRigidBody::CF_NO_CONTACT_RESPONSE );
// 	body->setUserPointer( DEBUG_CLIENTBLOCK int(triggerID) );
}

void cpl::BulletPhysics::VApplyForce( Vec dir, float newtons, ActorId aid, ComponentId compId )
{
	if ( btRigidBody * const body = FindComponentBody( aid, compId ) )
	{
		btVector3 const force( dir.GetX() * newtons,
			dir.GetY() * newtons,
			dir.GetZ() * newtons );

		body->applyCentralImpulse( force );
	}
}

void cpl::BulletPhysics::VApplyTorque( Vec dir, float newtons, ActorId aid, ComponentId compId )
{
	if ( btRigidBody * const body = FindComponentBody( aid, compId ) )
	{
		btVector3 const torque( dir.GetX() * newtons,
			dir.GetY() * newtons,
			dir.GetZ() * newtons );

		body->applyTorqueImpulse( torque );
	}
}

void cpl::BulletPhysics::VSetVelocity( ActorId actorId, ComponentId compId, Vec vel )
{
	if ( btRigidBody * const body = FindComponentBody(actorId,compId) )
	{
		btVector3 btVel = Vec_to_btVector3(vel);
		body->setLinearVelocity(btVel);
	}
}

void cpl::BulletPhysics::VSetAngularVelocity( ActorId actorId, ComponentId compId, Vec vel )
{
	if ( btRigidBody * const body = FindComponentBody(actorId,compId) )
	{
		btVector3 btVel = Vec_to_btVector3(vel);
		body->setAngularVelocity(btVel);
	}
}

bool cpl::BulletPhysics::VKinematicMove( Mat mat, ActorId aid, ComponentId compId )
{
	if ( btRigidBody * const body = FindComponentBody( aid, compId ) )
	{
		if(body->isStaticObject())
		{
			body->setWorldTransform(Mat_to_btTransform(mat));
		}
		else
		{
			body->setWorldTransform(Mat_to_btTransform(mat));
		}

		return true;
	}

	return false;
}

void cpl::BulletPhysics::VTranslate( ActorId actorId, ComponentId compId, Vec vec )
{
	if ( btRigidBody * const body = FindComponentBody(actorId,compId) )
	{
		btVector3 btVel = Vec_to_btVector3(vec);
		body->translate(btVel);
	}
}

void cpl::BulletPhysics::VRotate( ActorId actorId, ComponentId compId, Vec vec )
{
	printf("BulletPhysics::VRotate(), Not implemented \n");
}

void cpl::BulletPhysics::VStopActor(ActorId actorId)
{
	printf("BulletPhysics::VStopActor(), Not implemented \n");

// 	BulletActor* pBulletActor = FindBulletActor(actorId,compId);
// 	assert(pBulletActor);
// 	// [rez] None of these actually do what I want....
// 	//pBulletActor->m_pRigidBody->clearForces();
// 	//pBulletActor->m_pRigidBody->setFriction(FLT_MAX);  // this doesn't stop the actor.....
// 	pBulletActor->m_pRigidBody->setLinearVelocity(btVector3(0,0,0));
}

void cpl::BulletPhysics::VStopComponent( ActorId actorId,ComponentId compId )
{
	printf("BulletPhysics::VStopComponent(), Not implemented \n");
}

Vec cpl::BulletPhysics::VGetVelocity( ActorId actorId, ComponentId compId )
{
	if ( btRigidBody * const body = FindComponentBody(actorId,compId) )
	{
		btVector3 btVel = body->getLinearVelocity();
		return btVector3_to_Vec(btVel);
	}

	return Vec(1.1f,2.2f,3.3f,4.4f);
}

Vec cpl::BulletPhysics::VGetAngularVelocity( ActorId actorId, ComponentId compId )
{
	if ( btRigidBody * const body = FindComponentBody(actorId,compId) )
	{
		btVector3 btVel = body->getAngularVelocity();
		return btVector3_to_Vec(btVel);
	}

	return Vec(1.1f,2.2f,3.3f,4.4f);
}

Vec cpl::BulletPhysics::VRayCast( Vec start, Vec end, bool& hit )
{
	vector<Vec> hitPoints;

	btVector3 btStart = Vec_to_btVector3(start);
	btVector3 btEnd   = Vec_to_btVector3(end);

	btCollisionWorld::ClosestRayResultCallback RayCallback(btStart, btEnd);

	// Perform raycast
	m_dynamicsWorld->rayTest(btStart, btEnd,RayCallback);

	if(RayCallback.hasHit()) 
	{
		Vec hitPos    = btVector3_to_Vec(RayCallback.m_hitPointWorld);
		Vec hitNormal = btVector3_to_Vec(RayCallback.m_hitNormalWorld);

		hit = true;
		return hitPos;
	}

	hit = false;
	return Vec();
}

void cpl::BulletPhysics::TogglePause()
{
	if (m_bRun)
	{
		m_bRun = false;
		printf("Physic: Pause\n");
	}
	else
	{
		m_bRun = true;
		printf("Physic: Run\n");
	}
}

void cpl::BulletPhysics::DebugDrawWorld( bool value )
{
	m_bDebugDrawWorld = value;
}

void cpl::BulletPhysics::VSetDebugMode( int mode )
{

}

void cpl::BulletPhysics::SetDebugDrawer( btIDebugDraw* pDebugDraw )
{
	m_dynamicsWorld->setDebugDrawer(pDebugDraw);
}


void cpl::BulletPhysics::OnMoved( ActorId actorID, ComponentId componentID, Mat translation, Mat rotation)
{
	for(std::map<string, OnMovedDeligate>::iterator it = m_onMovedSubscriberMap.begin(); it != m_onMovedSubscriberMap.end(); it++)
	{
		it->second( actorID, componentID, translation, rotation );
	}
}

void cpl::BulletPhysics::OnPhysTriggerEnter( ActorId actorID_A, ComponentId componentID_A, ActorId actorID_B, ComponentId componentID_B )
{
	for(std::map<string, OnPhysTriggerEnterDeligate>::iterator it = m_onPhysTriggerEnterSubscriberMap.begin(); it != m_onPhysTriggerEnterSubscriberMap.end(); it++)
	{
		it->second( actorID_A, componentID_A, actorID_B, componentID_B );
	}
}

void cpl::BulletPhysics::OnCollision( ActorId actorID_A, ComponentId componentID_A, ActorId actorID_B, ComponentId componentID_B, vector<ContactInfo> contactInfo)
{
	for(std::map<string, OnCollisionDeligate>::iterator it = m_onCollisionSubscriberMap.begin(); it != m_onCollisionSubscriberMap.end(); it++)
	{
		it->second( actorID_A, componentID_A, actorID_B, componentID_B, contactInfo );
	}
}

void cpl::BulletPhysics::SubscribeOnMoved(string name, OnMovedDeligate onMovedDeligate)
{
	m_onMovedSubscriberMap[name] = onMovedDeligate;
}

void cpl::BulletPhysics::UnSubscribeOnMoved(string name)
{
	std::map<string, OnMovedDeligate>::iterator it = m_onMovedSubscriberMap.find(name);
	if(it == m_onMovedSubscriberMap.end())
		return;

	m_onMovedSubscriberMap.erase( it );
}

void cpl::BulletPhysics::SubscribeOnTriggerEnter(string name, OnPhysTriggerEnterDeligate onPhysTriggerEnterDeligate)
{
	m_onPhysTriggerEnterSubscriberMap[name] = onPhysTriggerEnterDeligate;
}

void cpl::BulletPhysics::UnSubscribeOnTriggerEnter(string name)
{
	std::map<string, OnPhysTriggerEnterDeligate>::iterator it = m_onPhysTriggerEnterSubscriberMap.find(name);
	if(it == m_onPhysTriggerEnterSubscriberMap.end())
		return;

	m_onPhysTriggerEnterSubscriberMap.erase( it );
}

void cpl::BulletPhysics::SubscribeOnCollision(string name, OnCollisionDeligate onCollisionDeligate)
{
	m_onCollisionSubscriberMap[name] = onCollisionDeligate;
}

void cpl::BulletPhysics::UnSubscribeOnCollision(string name)
{
	std::map<string, OnCollisionDeligate>::iterator it = m_onCollisionSubscriberMap.find(name);
	if(it == m_onCollisionSubscriberMap.end())
		return;

	m_onCollisionSubscriberMap.erase( it );
}


void cpl::BulletPhysics::BulletInternalTickCallback( btDynamicsWorld * const world, btScalar const timeStep )
{
	assert( world );

	assert( world->getWorldUserInfo() );
	BulletPhysics * const bulletPhysics = static_cast<BulletPhysics*>( world->getWorldUserInfo() );

	CollisionPairs currentTickCollisionPairs;

	// look at all existing contacts
	btDispatcher * const dispatcher = world->getDispatcher();
	for ( int manifoldIdx=0; manifoldIdx<dispatcher->getNumManifolds(); ++manifoldIdx )
	{
		// get the "manifold", which is the set of data corresponding to a contact point
		//   between two physics objects
		btPersistentManifold const * const manifold = dispatcher->getManifoldByIndexInternal( manifoldIdx );
		assert( manifold );

		// get the two bodies used in the manifold.  Bullet stores them as void*, so we must cast
		//  them back to btRigidBody*s.  Manipulating void* pointers is usually a bad
		//  idea, but we have to work with the environment that we're given.  We know this
		//  is safe because we only ever add btRigidBodys to the simulation
		btRigidBody const * const body0 = static_cast<btRigidBody const *>(manifold->getBody0());
		btRigidBody const * const body1 = static_cast<btRigidBody const *>(manifold->getBody1());

		// always create the pair in a predictable order
		bool const swapped = body0 > body1;

		btRigidBody const * const sortedBodyA = swapped ? body1 : body0;
		btRigidBody const * const sortedBodyB = swapped ? body0 : body1;

		CollisionPair const thisPair = std::make_pair( sortedBodyA, sortedBodyB );
		currentTickCollisionPairs.insert( thisPair );


		//
		// Debug Contact Info
		//
		/*
		ContactInfoList contactInfoList;

		int numContacts = manifold->getNumContacts();
		for (int j = 0; j < numContacts; j++)
		{
			btManifoldPoint pt = manifold->getContactPoint(j);
			if (pt.getDistance() < 0.0f)
			{			
				bulletPhysics->m_debugDrawer->drawLine(  pt.getPositionWorldOnB(), pt.getPositionWorldOnB() + pt.m_normalWorldOnB * btVector3(50.0f,50.0f,50.0f), btVector3(1.0f,0.0f,0.0f) );		
			}
		}
		*/


		if ( bulletPhysics->m_previousTickCollisionPairs.find( thisPair ) == bulletPhysics->m_previousTickCollisionPairs.end() )
		{
			// this is a new contact, which wasn't in our list before.  send an event to the game.
			bulletPhysics->SendCollisionPairAddEvent( manifold, body0, body1 );
		}
	}

	CollisionPairs removedCollisionPairs;

	// use the STL set difference function to find collision pairs that existed during the previous tick but not any more
	std::set_difference( bulletPhysics->m_previousTickCollisionPairs.begin(), bulletPhysics->m_previousTickCollisionPairs.end(),
		currentTickCollisionPairs.begin(), currentTickCollisionPairs.end(),
		std::inserter( removedCollisionPairs, removedCollisionPairs.begin() ) );

	for ( CollisionPairs::const_iterator it = removedCollisionPairs.begin(), 
		end = removedCollisionPairs.end(); it != end; ++it )
	{
		btRigidBody const * const body0 = it->first;
		btRigidBody const * const body1 = it->second;

		bulletPhysics->SendCollisionPairRemoveEvent( body0, body1 );
	}

	// the current tick becomes the previous tick.  this is the way of all things.
	bulletPhysics->m_previousTickCollisionPairs = currentTickCollisionPairs;

// 	// handle actors that want to turn manually
// 	for ( ActorIDToBulletActorMap::const_iterator it = bulletPhysics->m_actorBodies.begin();
// 		it != bulletPhysics->m_actorBodies.end();
// 		++it )
// 	{
// 		if ( it->second->m_desiredDeltaYAngleTime > 0 )
// 		{
// 			float const amountOfTimeToConsume = std::min( timeStep, it->second->m_desiredDeltaYAngleTime );
// 			float const deltaAngle = (amountOfTimeToConsume / it->second->m_desiredDeltaYAngleTime) * it->second->m_desiredDeltaYAngle;
// 
// 			// create a transform to represent the additional turning this frame
// 			btTransform angleTransform;
// 			angleTransform.setIdentity();
// 			angleTransform.getBasis().setEulerYPR( 0, deltaAngle, 0 ); // rotation about body Y-axis
// 
// 			btRigidBody * const body = it->second->m_pRigidBody;
// 
// 			// concatenate the transform onto the body's transform
// 			body->setCenterOfMassTransform( body->getCenterOfMassTransform() * angleTransform );
// 
// 			// tick down the parameters
// 			it->second->m_desiredDeltaYAngle -= deltaAngle;
// 			it->second->m_desiredDeltaYAngleTime -= amountOfTimeToConsume;
// 		}
// 	}
}

void cpl::BulletPhysics::SendCollisionPairAddEvent( btPersistentManifold const * manifold, btRigidBody const * const body0, btRigidBody const * const body1 )
{
	if ( body0->getUserPointer() || body1->getUserPointer() )
	{
		// only triggers have non-NULL userPointers

		// figure out which actor is the trigger
		btRigidBody const * triggerBody, * otherBody;

		if ( body0->getUserPointer() )
		{
			triggerBody = body0;
			otherBody = body1;
		}
		else
		{
			otherBody = body0;
			triggerBody = body1;
		}

		// send the trigger event.
		//int const triggerId = *static_cast<int*>(triggerBody->getUserPointer());
		//safeQueEvent( IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_PhysTrigger_Enter( triggerId, FindActorID( otherBody ) )));
	}
	else
	{
		ActorId id0 = FindActorID( body0 );
		ActorId id1 = FindActorID( body1 );

		if ( id0 == 0 || id1 == 0 )
		{
			// something is colliding with a non-actor.  we currently don't send events for that
			return;
		}

		ContactInfoList contactInfoList;

		int numContacts = manifold->getNumContacts();
		for (int j = 0; j < numContacts; j++)
		{
			btManifoldPoint pt = manifold->getContactPoint(j);
			if (pt.getDistance() < 0.0f)
			{
				ContactInfo contactiInfo;

				contactiInfo.ptA	   = btVector3_to_Vec( pt.getPositionWorldOnA() );
				contactiInfo.ptB	   = btVector3_to_Vec( pt.getPositionWorldOnB() );
				contactiInfo.normalOnB = btVector3_to_Vec( pt.m_normalWorldOnB );

				contactInfoList.push_back(contactiInfo);
			}
		}


		// send the event for the game
		//safeQueEvent(IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_PhysCollision(*id0, *id1, contactInfoList)));
	}
}

void cpl::BulletPhysics::SendCollisionPairRemoveEvent( btRigidBody const * const body0, btRigidBody const * const body1 )
{
	if ( body0->getUserPointer() || body1->getUserPointer() )
	{
		// figure out which actor is the trigger
		btRigidBody const * triggerBody, * otherBody;

		if ( body0->getUserPointer() )
		{
			triggerBody = body0;
			otherBody = body1;
		}
		else
		{
			otherBody = body0;
			triggerBody = body1;
		}

		// send the trigger event.
		//int const triggerId = *static_cast<int*>(triggerBody->getUserPointer());
		//safeQueEvent(IEventDataPtr(new EvtData_PhysTrigger_Leave( triggerId, FindActorID( otherBody ))));
	}
	else
	{
		ActorId id0 = FindActorID( body0 );
		ActorId id1 = FindActorID( body1 );

		if ( id0 == 0 || id1 == 0 )
		{
			// collision is ending between some object(s) that don't have actors.  we don't send events for that.
			return;
		}

		//safeQueEvent(IEventDataPtr(new EvtData_PhysSeparation( *id0, *id1 )));
	}
}

void cpl::BulletPhysics::RemoveCollisionObject( btCollisionObject * const removeMe )
{
	// first remove the object from the physics sim
	m_dynamicsWorld->removeCollisionObject( removeMe );

	// then remove the pointer from the ongoing contacts list
	for ( CollisionPairs::iterator it = m_previousTickCollisionPairs.begin();
		it != m_previousTickCollisionPairs.end(); )
	{
		CollisionPairs::iterator next = it;
		++next;

		if ( it->first == removeMe || it->second == removeMe )
		{
			SendCollisionPairRemoveEvent( it->first, it->second );
			m_previousTickCollisionPairs.erase( it );
		}

		it = next;
	}

	// if the object is a RigidBody (all of ours are RigidBodies, but it's good to be safe)
	if ( btRigidBody * const body = btRigidBody::upcast(removeMe) )
	{
		// delete the components of the object
		delete body->getMotionState();
		delete body->getCollisionShape();
		delete body->getUserPointer();
		delete body->getUserPointer();

		for ( int ii=body->getNumConstraintRefs()-1; ii >= 0; --ii )
		{
			btTypedConstraint * const constraint = body->getConstraintRef( ii );
			m_dynamicsWorld->removeConstraint( constraint );
			delete constraint;
		}
	}

	delete removeMe;
}

btRigidBody* cpl::BulletPhysics::FindComponentBody( ActorId id, ComponentId compId ) const
{
	std::map<ActorId,map<ComponentId,btRigidBody*>>::const_iterator itActorMap = m_actorMap.find(id);
	if(itActorMap == m_actorMap.end())
		return NULL;

	std::map<ComponentId,btRigidBody*>::const_iterator itComponentMap = itActorMap->second.find(id);
	if(itComponentMap == itActorMap->second.end())
		return NULL;

	return itComponentMap->second;
}

cpl::ActorId cpl::BulletPhysics::FindActorID( btRigidBody const * const body ) const
{
	RigidBodyIDPairMap::const_iterator found = m_rigidBodyToActorId.find( body );
	if ( found != m_rigidBodyToActorId.end() )
		return found->second.first;

	return 0;
}


cpl::IPhysic CLEAR_PHYSICS_API * cpl::CreateGamePhysics()
{
	std::auto_ptr<cpl::IPhysic> gamePhysics;
	gamePhysics.reset( new BulletPhysics );

	if (gamePhysics.get() && !gamePhysics->VInitialize())
	{
		// physics failed to initialize.  delete it.
		gamePhysics.reset();
	}

	return gamePhysics.release();
}

cpl::IPhysic CLEAR_PHYSICS_API * cpl::CreateNullPhysics()
{
	std::auto_ptr<cpl::IPhysic> gamePhysics;
	gamePhysics.reset( new NullPhysics );
	if (gamePhysics.get() && !gamePhysics->VInitialize())
	{
		// physics failed to initialize.  delete it.
		gamePhysics.reset();
	}

	return gamePhysics.release();
}
