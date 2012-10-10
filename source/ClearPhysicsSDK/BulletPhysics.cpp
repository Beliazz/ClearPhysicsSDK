#include "ClearPhysicsSDK.h"
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

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







CLEAR_PHYSICS_API ActorMotionState::ActorMotionState( Mat startingTransform )
	: m_worldToPositionTransform( startingTransform )
{ }

void CLEAR_PHYSICS_API ActorMotionState::getWorldTransform( btTransform & worldTrans ) const
{
	worldTrans = Mat_to_btTransform( m_worldToPositionTransform );
}

void CLEAR_PHYSICS_API ActorMotionState::setWorldTransform( const btTransform& worldTrans )
{
	m_worldToPositionTransform = btTransform_to_Mat( worldTrans );
}






/////////////////////////////////////////////////////////////////////////////


CLEAR_PHYSICS_API BulletPhysics::BulletPhysics()
{
	// auto_ptr<> will automatically initialize themselves to NULL
	m_bDebugDrawWorld = false;
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 555
//
CLEAR_PHYSICS_API BulletPhysics::~BulletPhysics()
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
	for (ActorIDToBulletActorMap::iterator it = m_actorBodies.begin(); it != m_actorBodies.end(); ++it)
	{
		BulletActor* pBulletActor = it->second;
		delete pBulletActor;
	}
	m_actorBodies.clear();

	// auto_ptrs will handle deletion of m_dynamicsWorld et. al.

	for (ActorIDToBulletCSCActor::iterator it = m_actorCSC.begin(); it != m_actorCSC.end(); ++it)
	{
		BulletCompoundShapeChildActor* pChild = it->second;
		delete pChild;
	}
	m_actorCSC.clear();

}


/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 548
//
bool CLEAR_PHYSICS_API BulletPhysics::VInitialize()
{
	// VInitialize creates the components that Bullet uses

	// this controls how Bullet does internal memory management during the collision pass
	m_collisionConfiguration.reset( DEBUG_CLIENTBLOCK btDefaultCollisionConfiguration() );

	// this manages how Bullet detects precise collisions between pairs of objects
	m_dispatcher.reset( DEBUG_CLIENTBLOCK btCollisionDispatcher( m_collisionConfiguration.get() ) );

	// Bullet uses this to quickly (imprecisely) detect collisions between objects.
	//   Once a possible collision passes the broad phase, it will be passed to the
	//   slower but more precise narrow-phase collision detection (btCollisionDispatcher).
	m_broadphase.reset( DEBUG_CLIENTBLOCK btDbvtBroadphase() );

	// Manages constraints which apply forces to the physics simulation.  Used
	//  for e.g. springs, motors.  We don't use any constraints right now.
	m_solver.reset( DEBUG_CLIENTBLOCK btSequentialImpulseConstraintSolver );

	// This is the main Bullet interface point.  Pass in all these components to customize its behavior.
	m_dynamicsWorld.reset( DEBUG_CLIENTBLOCK btDiscreteDynamicsWorld( m_dispatcher.get(), 
		m_broadphase.get(), 
		m_solver.get(), 
		m_collisionConfiguration.get() ) );

	// and set the internal tick callback to our own method "BulletInternalTickCallback"
	m_dynamicsWorld->setInternalTickCallback( BulletInternalTickCallback );
	m_dynamicsWorld->setWorldUserInfo( this );


	m_debugDrawer.reset( DEBUG_CLIENTBLOCK BulletDebugDrawer());
	if(!m_debugDrawer->Init())
		return false;

	m_dynamicsWorld->setDebugDrawer(m_debugDrawer.get());



	m_pListener = EventListenerPtr( DEBUG_CLIENTBLOCK PhysicsEventListener( this ) );

	m_bRun = true;

	safeAddListener(m_pListener, EvtData_PhysTrigger_Enter::sk_EventType);
	safeAddListener(m_pListener, EvtData_PhysTrigger_Leave::sk_EventType);
	safeAddListener(m_pListener, EvtData_Phys_TogglePause::sk_EventType);
	safeAddListener(m_pListener, EvtData_PhysSeparation::sk_EventType);
	safeAddListener(m_pListener, EvtData_Phys_RenderDiagnostic::sk_EventType);


	m_fStepSimulationTime = 0.0f;
	m_fSyncVisualSceneTime = 0.0f;
	m_fUpdateKinematicControllerTime = 0.0f;
	m_fDebugDrawWorldTime = 0.0f;

	return true;
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 551
//
void CLEAR_PHYSICS_API BulletPhysics::VOnUpdate( float const deltaSeconds )
{
	// Bullet uses an internal fixed timestep (default 1/60th of a second)
	//   We pass in 4 as a max number of sub steps.  Bullet will run the simulation
	//   in increments of the fixed timestep until "deltaSeconds" amount of time has
	//   passed, but will only run a maximum of 4 steps this way.

	if (m_bRun)
	{
		m_dynamicsWorld->stepSimulation(deltaSeconds);
	}

	for(std::map<ActorId, shared_ptr<CKinematicController>>::iterator it = m_kinematicControllers.begin(); it != m_kinematicControllers.end(); it++) 
	{
		it->second->Update(deltaSeconds);
	}

	//m_fUpdateKinematicControllerTime = m_timer->get();
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 552
//
void CLEAR_PHYSICS_API BulletPhysics::VSyncVisibleScene()
{
	// Keep physics & graphics in sync

	// check all the existing actor's bodies for changes. 
	//  If there is a change, send the appropriate event for the game system.
	for ( ActorIDToBulletActorMap::const_iterator it = m_actorBodies.begin(); it != m_actorBodies.end(); ++it )
	{ 
		ActorId const id = it->first;

		// get the MotionState.  this object is updated by Bullet.
		// it's safe to cast the btMotionState to ActorMotionState, because all the bodies in m_actorBodies
		//   were created through AddShape()
		btMotionState *motionState = it->second->m_pRigidBody->getMotionState();
		assert( motionState );


		btTransform trans;
		motionState->getWorldTransform(trans);

		float a[16];
		float b[16];

		Mat current = btTransform_to_Mat(trans);

		current.GetArray(a);
		it->second->m_pActorMat->GetArray(b);

		Vec pos = MatGetPosition(*it->second->m_pActorMat);

		for (unsigned int i = 0; i < 16 ; i++)
		{
			if ( a[i] != b[i] )
			{
				// bullet has moved the actor's physics object.  update the actor.
				safeQueEvent( IEventDataPtr( DEBUG_CLIENTBLOCK EvtData_Move_Actor( id, current ) ) );
				break;
			}
		}
	}

	for ( ActorIDToBulletCSCActor::const_iterator it = m_actorCSC.begin(); it != m_actorCSC.end(); ++it )
	{ 
		ActorId const id = it->first;

		btCompoundShape* cpShape = it->second->m_pCompoundShape;
		unsigned int childindex  = it->second->m_compoundShapeIndex;

		btTransform trans = cpShape->getChildTransform(childindex);
			
		float a[16];
		float b[16];

		Mat current = btTransform_to_Mat(trans);

		current.GetArray(a);
		it->second->m_pActorMat->GetArray(b);

		Vec pos = MatGetPosition(*it->second->m_pActorMat);

		for (unsigned int i = 0; i < 16 ; i++)
		{
			if ( a[i] != b[i] )
			{
				// bullet has moved the actor's physics object.  update the actor.
				safeQueEvent( IEventDataPtr( DEBUG_CLIENTBLOCK EvtData_Move_Actor( id, current ) ) );
				break;
			}
		}
	}

	//m_fSyncVisualSceneTime = m_timer->get();
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 553
//
void CLEAR_PHYSICS_API BulletPhysics::AddShape( IActor * const actor, 
	btCollisionShape * const shape, 
	btScalar const mass, 
	enum PhysicsMaterial const mat )
{
	// actors get one body apiece
	optional<ActorId> const maybeID = actor->VGetID();
	assert( maybeID.valid() && "Actor with invalid ID?" );

	ActorId const actorID = *maybeID;
	assert( m_actorBodies.find( actorID ) == m_actorBodies.end() && "Actor with more than one physics body?" );

	// localInertia defines how the object's mass is distributed
	btVector3 localInertia( 0.f, 0.f, 0.f );
	if ( mass > 0.f )
		shape->calculateLocalInertia( mass, localInertia );

	// set the initial position of the body from the actor
	btDefaultMotionState* motionState = DEBUG_CLIENTBLOCK btDefaultMotionState( Mat_to_btTransform( *actor->VGetMat() ));

	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, motionState, shape, localInertia );

	// set up the materal properties
	rbInfo.m_restitution	 = g_PhysicsMaterials[mat].m_restitution;
	rbInfo.m_friction		 = g_PhysicsMaterials[mat].m_friction;

	btRigidBody * const body = new btRigidBody(rbInfo);

	m_dynamicsWorld->addRigidBody( body );

	// create the BulletActor
	BulletActor* pBulletActor = DEBUG_CLIENTBLOCK BulletActor( body, actor->VGetMat() );

	// add it to the collection to be checked for changes in VSyncVisibleScene
	m_actorBodies[actorID] = pBulletActor;
	m_rigidBodyToActorId[body] = actorID;
}

/////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::RemoveCollisionObject( btCollisionObject * const removeMe )
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

/////////////////////////////////////////////////////////////////////////////
btRigidBody CLEAR_PHYSICS_API * BulletPhysics::FindActorBody( ActorId const id ) const
{
	BulletActor* pBulletActor = FindBulletActor(id);
	if (pBulletActor)
		return pBulletActor->m_pRigidBody;
	return NULL;
}

/////////////////////////////////////////////////////////////////////////////
BulletActor CLEAR_PHYSICS_API* BulletPhysics::FindBulletActor( ActorId const id ) const
{
	ActorIDToBulletActorMap::const_iterator found = m_actorBodies.find( id );
	if ( found != m_actorBodies.end() )
		return found->second;

	return NULL;
}

/////////////////////////////////////////////////////////////////////////////
optional<ActorId> CLEAR_PHYSICS_API BulletPhysics::FindActorID( btRigidBody const * const body ) const
{
	RigidBodyToActorIDMap::const_iterator found = m_rigidBodyToActorId.find( body );
	if ( found != m_rigidBodyToActorId.end() )
		return found->second;

	return optional<ActorId>();
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 553
//
void CLEAR_PHYSICS_API BulletPhysics::VAddSphere( float const radius, IActor * const actor, float const specificGravity, enum PhysicsMaterial const mat)
{
	assert( actor );

	// create the collision body, which specifies the shape of the object
	btSphereShape * const collisionShape = new btSphereShape( radius );

	// calculate absolute mass from specificGravity
	float const volume = (4.f / 3.f) * D3DX_PI * radius * radius * radius;
	btScalar const mass = volume * specificGravity;

	AddShape( actor, collisionShape, mass, mat );
}

void BulletPhysics::VAddPlane( Vec planeNormal, float planeConstante, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat )
{
	assert( gameActor );

	btCollisionShape* planeShape = new btStaticPlaneShape(Vec_to_btVector3(planeNormal),planeConstante);

	AddShape( gameActor, planeShape, 0.0f, mat );
}
/////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::VAddBox(Vec dimensions, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat)
{
	assert( gameActor );

	// 	// create the collision body, which specifies the shape of the object
	btBoxShape * const boxShape = new btBoxShape( Vec_to_btVector3( dimensions ) );

	// calculate absolute mass from specificGravity
	float const volume = dimensions.GetX() * dimensions.GetY() * dimensions.GetZ();
	btScalar const mass = volume * specificGravity;

	AddShape( gameActor, boxShape, mass, mat );
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 555
//
void CLEAR_PHYSICS_API BulletPhysics::VAddPointCloud(Vec *verts, int numPoints, IActor *actor, float specificGravity, enum PhysicsMaterial mat)
{
	assert( actor );

	btConvexHullShape * const shape = new btConvexHullShape();

	// add the points to the shape one at a time
	for ( int ii=0; ii<numPoints; ++ii )
		shape->addPoint(  Vec_to_btVector3( verts[ii] ) );

	// approximate absolute mass using bounding box
	btVector3 aabbMin(0,0,0), aabbMax(0,0,0);
	shape->getAabb( btTransform::getIdentity(), aabbMin, aabbMax );

	btVector3 const aabbExtents = aabbMax - aabbMin;

	float const volume = aabbExtents.x() * aabbExtents.y() * aabbExtents.z();
	btScalar const mass = volume * specificGravity;

	AddShape( actor, shape, mass, mat );
}

void CLEAR_PHYSICS_API BulletPhysics::VAddTriangleMesh( btCollisionShape* shape, IActor *actor, float specificGravity, enum PhysicsMaterial mat )
{
	assert( actor );

	//Check if the shape is infinite
	//volume/mass calc would be wrong
	if (shape->isInfinite())
	{
		AddShape( actor, shape, 0, mat );
		return;
	}

	// approximate absolute mass using bounding box
	btVector3 aabbMin(0,0,0), aabbMax(0,0,0);
	shape->getAabb( btTransform::getIdentity(), aabbMin, aabbMax );

	btVector3 const aabbExtents = aabbMax - aabbMin;

	float const volume = aabbExtents.x() * aabbExtents.y() * aabbExtents.z();
	btScalar const mass = volume * specificGravity;

	AddShape( actor, shape, mass, mat );
}

/////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::VRemoveActor(ActorId id)
{
	if ( btRigidBody * const body = FindActorBody( id ) )
	{
		// destroy the body and all its components
		RemoveCollisionObject( body );

		// clear the relevant elements from the lookup maps
		ActorIDToBulletActorMap::iterator it = m_actorBodies.find(id);
		if (it != m_actorBodies.end())
		{
			BulletActor* pDead = it->second;
			delete pDead;
			m_actorBodies.erase(it);
		}
		m_rigidBodyToActorId.erase( body );
	}
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 557
//
void CLEAR_PHYSICS_API BulletPhysics::VRenderDiagnostics(IScene* pScene)
{
	//m_timer->Start();
	if (m_bDebugDrawWorld)
	{
		m_debugDrawer->PreRender();
		m_dynamicsWorld->debugDrawWorld();
		m_debugDrawer->Render( pScene );
		
		m_fDebugDrawWorldTime = 0.0f;
		return;
	}
// 	m_timer->Stop();
// 	m_fDebugDrawWorldTime = m_timer->get();
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 556
//
void CLEAR_PHYSICS_API BulletPhysics::VCreateTrigger(Vec pos, const float dim, int triggerID)
{
	// create the collision body, which specifies the shape of the object
	btBoxShape * const boxShape = new btBoxShape( Vec_to_btVector3( Vec(dim,dim,dim) ) );

	// triggers are immoveable.  0 mass signals this to Bullet.
	btScalar const mass = 0;

	// set the initial position of the body from the actor
	Mat triggerTrans = MatIdentity();
	MatSetPosition(triggerTrans, pos);
	ActorMotionState * const myMotionState = DEBUG_CLIENTBLOCK ActorMotionState( triggerTrans );

	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, boxShape, btVector3(0,0,0) );
	btRigidBody * const body = new btRigidBody(rbInfo);

	m_dynamicsWorld->addRigidBody( body );

	// a trigger is just a box that doesn't collide with anything.  That's what "CF_NO_CONTACT_RESPONSE" indicates.
	body->setCollisionFlags( body->getCollisionFlags() | btRigidBody::CF_NO_CONTACT_RESPONSE );
	body->setUserPointer( DEBUG_CLIENTBLOCK int(triggerID) );
}

/////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::VApplyForce(Vec dir, float newtons, ActorId aid)
{
	if ( btRigidBody * const body = FindActorBody( aid ) )
	{
		btVector3 const force( dir.GetX() * newtons,
			dir.GetY() * newtons,
			dir.GetZ() * newtons );

		body->applyCentralImpulse( force );
	}
}

/////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::VApplyTorque(Vec dir, float magnitude, ActorId aid)
{
	if ( btRigidBody * const body = FindActorBody( aid ) )
	{
		btVector3 const torque( dir.GetX() * magnitude,
			dir.GetY() * magnitude,
			dir.GetZ() * magnitude );

		body->applyTorqueImpulse( torque );
	}
}

cgl::PCGLTimer timer;
/////////////////////////////////////////////////////////////////////////////
bool CLEAR_PHYSICS_API BulletPhysics::VKinematicMove(Mat mat, ActorId aid)
{
	btRigidBody * const body = FindActorBody( aid );

	if ( btRigidBody * const body = FindActorBody( aid ) )
	{
		// warp the body to the new position
		//body->getMotionState()->setWorldTransform( Mat_to_btTransform( mat ) );
		timer->Start();
		btTransform tmp = Mat_to_btTransform( mat );
		timer->Stop();

		timer->Start();
		body->setWorldTransform( tmp );
		timer->Stop();

		return true;
	}
	else if ( BulletCompoundShapeChildActor * const child = FindCompoundShapeChild( aid ) )
	{
		child->m_pCompoundShape->updateChildTransform(child->m_compoundShapeIndex,Mat_to_btTransform( mat ));
		timer->Stop();
		return true;
	}

	return false;
}

/////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::VRotateY( ActorId const actorId, float const deltaAngleRadians, float const time )
{
	if ( BulletActor * const actor = FindBulletActor( actorId ) )
	{
		// set the parameters for the turning that will be handled in BulletInternalTickCallback
		actor->m_desiredDeltaYAngle = deltaAngleRadians;
		actor->m_desiredDeltaYAngleTime = time;
	}
}


/////////////////////////////////////////////////////////////////////////////
float CLEAR_PHYSICS_API BulletPhysics::VGetOrientationY(ActorId actorId)
{
	BulletActor* pBulletActor = FindBulletActor(actorId);
	assert(pBulletActor);

	const btTransform& actorTransform = pBulletActor->m_pRigidBody->getCenterOfMassTransform();
	btMatrix3x3 actorRotationMat(actorTransform.getBasis());  // should be just the rotation information

	btVector3 startingVec(0,0,1);
	btVector3 endingVec = actorRotationMat * startingVec; // transform the vector

	endingVec.setY(0);  // we only care about rotation on the XZ plane

	float const endingVecLength = endingVec.length();
	if (endingVecLength < 0.001)
	{
		// gimbal lock (orientation is straight up or down)
		return 0;
	}

	else
	{
		btVector3 cross = startingVec.cross(endingVec);
		float sign = cross.getY() > 0 ? 1.0f : -1.0f;
		return (acosf(startingVec.dot(endingVec) / endingVecLength) * sign);
	}

	return FLT_MAX;  // fail...
}

/////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::VStopActor(ActorId actorId)
{
	BulletActor* pBulletActor = FindBulletActor(actorId);
	assert(pBulletActor);
	// [rez] None of these actually do what I want....
	//pBulletActor->m_pRigidBody->clearForces();
	//pBulletActor->m_pRigidBody->setFriction(FLT_MAX);  // this doesn't stop the actor.....
	pBulletActor->m_pRigidBody->setLinearVelocity(btVector3(0,0,0));
}

/////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::VSetVelocity(ActorId actorId, Vec vel)
{
	BulletActor* pBulletActor = FindBulletActor(actorId);
	assert(pBulletActor);
	btVector3 btVel = Vec_to_btVector3(vel);
	pBulletActor->m_pRigidBody->setLinearVelocity(btVel);
}

/////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::VTranslate(ActorId actorId, Vec vec)
{
	BulletActor* pBulletActor = FindBulletActor(actorId);
	assert(pBulletActor);
	btVector3 btVec = Vec_to_btVector3(vec);
	pBulletActor->m_pRigidBody->translate(btVec);
}

void CLEAR_PHYSICS_API BulletPhysics::VStaticActor( ActorId actorId )
{
}


/////////////////////////////////////////////////////////////////////////////
// This function is called after bullet performs its internal update.  We
//   use it to detect collisions between objects for Game code.
//
//   - Chapter 15, page 560
//
void CLEAR_PHYSICS_API BulletPhysics::BulletInternalTickCallback( btDynamicsWorld * const world, btScalar const timeStep )
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

	// handle actors that want to turn manually
	for ( ActorIDToBulletActorMap::const_iterator it = bulletPhysics->m_actorBodies.begin();
		it != bulletPhysics->m_actorBodies.end();
		++it )
	{
		if ( it->second->m_desiredDeltaYAngleTime > 0 )
		{
			float const amountOfTimeToConsume = std::min( timeStep, it->second->m_desiredDeltaYAngleTime );
			float const deltaAngle = (amountOfTimeToConsume / it->second->m_desiredDeltaYAngleTime) * it->second->m_desiredDeltaYAngle;

			// create a transform to represent the additional turning this frame
			btTransform angleTransform;
			angleTransform.setIdentity();
			angleTransform.getBasis().setEulerYPR( 0, deltaAngle, 0 ); // rotation about body Y-axis

			btRigidBody * const body = it->second->m_pRigidBody;

			// concatenate the transform onto the body's transform
			body->setCenterOfMassTransform( body->getCenterOfMassTransform() * angleTransform );

			// tick down the parameters
			it->second->m_desiredDeltaYAngle -= deltaAngle;
			it->second->m_desiredDeltaYAngleTime -= amountOfTimeToConsume;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::SendCollisionPairAddEvent( btPersistentManifold const * manifold, btRigidBody const * const body0, btRigidBody const * const body1 )
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
		int const triggerId = *static_cast<int*>(triggerBody->getUserPointer());
		safeQueEvent( IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_PhysTrigger_Enter( triggerId, FindActorID( otherBody ) )));
	}
	else
	{
		optional<ActorId> const id0 = FindActorID( body0 );
		optional<ActorId> const id1 = FindActorID( body1 );

		if ( !id0.valid() || !id1.valid() )
		{
			// something is colliding with a non-actor.  we currently don't send events for that
			return;
		}

		// this pair of colliding objects is new.  send a collision-begun event
		VecList collisionPoints;
		Vec sumNormalForce(0,0,0);
		Vec sumFrictionForce(0,0,0);

		for ( int pointIdx = 0; pointIdx < manifold->getNumContacts(); ++pointIdx )
		{
			btManifoldPoint const & point = manifold->getContactPoint( pointIdx );

			collisionPoints.push_back( btVector3_to_Vec( point.getPositionWorldOnB() ) );

			sumNormalForce += btVector3_to_Vec( point.m_combinedRestitution * point.m_normalWorldOnB );
			sumFrictionForce += btVector3_to_Vec( point.m_combinedFriction * point.m_lateralFrictionDir1 );
		}

		// send the event for the game
		safeQueEvent(IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_PhysCollision(*id0, *id1,sumNormalForce,	sumFrictionForce, collisionPoints)));
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void CLEAR_PHYSICS_API BulletPhysics::SendCollisionPairRemoveEvent( btRigidBody const * const body0, btRigidBody const * const body1 )
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
		int const triggerId = *static_cast<int*>(triggerBody->getUserPointer());
		safeQueEvent(IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_PhysTrigger_Leave( triggerId, FindActorID( otherBody ))));
	}
	else
	{
		optional<ActorId> const id0 = FindActorID( body0 );
		optional<ActorId> const id1 = FindActorID( body1 );

		if ( !id0.valid() || !id1.valid() )
		{
			// collision is ending between some object(s) that don't have actors.  we don't send events for that.
			return;
		}

		safeQueEvent(IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_PhysSeparation( *id0, *id1 )));
	}
}

void CLEAR_PHYSICS_API BulletPhysics::VSetDebugMode( int mode )
{
}

Vec CLEAR_PHYSICS_API BulletPhysics::VRayCast( Vec start, Vec end, bool& hit )
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

void BulletPhysics::TogglePause()
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

void BulletPhysics::VSetCompoundShapeChild( ActorId parentID, IActor* childActor, unsigned int index )
{
	if ( btCompoundShape * const parentShape = FindCompoundShape( parentID ) )
	{
		BulletCompoundShapeChildActor* child = new BulletCompoundShapeChildActor();

		child->m_compoundShapeIndex = index;
		child->m_pCompoundShape	    = parentShape;
		child->m_pActorMat          = childActor->VGetMat();

		m_actorCSC[childActor->VGetID()] = child;
	}
}

btCompoundShape* BulletPhysics::FindCompoundShape( ActorId id ) const
{
	ActorIDToBulletbtCompoundShapeActorMap::const_iterator found = m_actorCompoundShapes.find( id );
	if ( found != m_actorCompoundShapes.end() )
		return found->second;

	return NULL;
}

BulletCompoundShapeChildActor* BulletPhysics::FindCompoundShapeChild( ActorId id ) const
{
	ActorIDToBulletCSCActor::const_iterator found = m_actorCSC.find( id );
	if ( found != m_actorCSC.end() )
		return found->second;

	return NULL;
}

void BulletPhysics::VAddCompoundShape( btCompoundShape* shape, IActor *actor, float specificGravity, enum PhysicsMaterial mat )
{
	assert( actor );

	//Check if the shape is infinite
	//volume/mass calc would be wrong
	if (shape->isInfinite())
	{
		AddShape( actor, shape, 0, mat );
		return;
	}

	// approximate absolute mass using bounding box
	btVector3 aabbMin(0,0,0), aabbMax(0,0,0);
	shape->getAabb( btTransform::getIdentity(), aabbMin, aabbMax );

	btVector3 const aabbExtents = aabbMax - aabbMin;

	float const volume = aabbExtents.x() * aabbExtents.y() * aabbExtents.z();
	btScalar const mass = volume * specificGravity;

	AddShape( actor, shape, mass, mat );
	m_actorCompoundShapes[actor->VGetID()] = shape;

}

void BulletPhysics::VAddKinematicController( shared_ptr<CKinematicController> kinematicController, IActor *actor, float specificGravity, enum PhysicsMaterial mat )
{
	m_kinematicControllers[actor->VGetID()] = kinematicController;

	kinematicController->SetDynmamicWorld(m_dynamicsWorld.get());
}

float BulletPhysics::GetStepSimulationTime()
{
	return m_fStepSimulationTime;
}

float BulletPhysics::GetSyncVisualSceneTime()
{
	return m_fSyncVisualSceneTime;
}

float BulletPhysics::GetKinematicControllerTime()
{
	return m_fUpdateKinematicControllerTime;
}

float BulletPhysics::GetDebugDrawWorldTime()
{
	return m_fDebugDrawWorldTime;
}


IGamePhysics CLEAR_PHYSICS_API * CreateGamePhysics()
{
	std::auto_ptr<IGamePhysics> gamePhysics;
	gamePhysics.reset( DEBUG_CLIENTBLOCK BulletPhysics );

	if (gamePhysics.get() && !gamePhysics->VInitialize())
	{
		// physics failed to initialize.  delete it.
		gamePhysics.reset();
	}

	return gamePhysics.release();
}

IGamePhysics CLEAR_PHYSICS_API * CreateNullPhysics()
{
	std::auto_ptr<IGamePhysics> gamePhysics;
	gamePhysics.reset( DEBUG_CLIENTBLOCK NullPhysics );
	if (gamePhysics.get() && !gamePhysics->VInitialize())
	{
		// physics failed to initialize.  delete it.
		gamePhysics.reset();
	}

	return gamePhysics.release();
}
