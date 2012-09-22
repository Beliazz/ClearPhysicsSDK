#include "Bloco.h"
#include "Physics.h"
#include "PhysicsEventListener.h"

struct BLOCO_API MaterialData
{
	float m_restitution;
	float m_friction;
};

MaterialData BLOCO_API g_PhysicsMaterials[] = 
{
	// restitution       friction
	{     0.05f,           0.9f     },			// playdough
	{     0.25f,           0.5f     },			// a 'normal' material
	{     0.95f,           0.5f     },          // a 'bouncy' material
	{     0.25f,           0.0f     },          // a 'slippery' material
};


//
static float g_DensityTable[] = 
{
	// specific gravity (these numbers are easier to find

	.0013f,		// air
	1.000f,		// water

	// Synthetics
	.0100f,		// styrofoam

	// Woods
	.0170f,		// balsa
	.3500f,		// bamboo
	.5000f,		// pine
	.8300f,		// oak
	1.100f,		// ebony

	// Biologic
	1.060f,		// blood
	1.800f,		// bone

	// Metals and Stone
	2.400f,		// silicon
	2.650f,		// aluminum
	2.450f,		// asbestos
	4.500f,		// barite
	3.350f,		// basalt
	9.800f,		// bismuth
	1.750f,		// borax
	2.320f,		// boron
	8.550f,		// brass
	8.640f,		// brick
	8.400f,		// bronze
	4.580f,		// calcium
	1.950f,		// carbon
	7.100f,		// chromium
	2.200f,		// clay - average
	0.800f,		// coal - average
	8.900f,		// cobalt
	8.750f,		// copper - average
	3.510f,		// diamond 
	2.900f,		// dolomite
	1.800f,		// epoxy
	2.600f,		// glass
	2.950f,		// crystal
	2.550f,		// granite
	19.30f,		// gold
	5.200f,		// hematite
	21.60f,		// iridium
	7.200f,		// cast iron
	7.750f,		// wrought iron
	2.400f,		// limestone
	11.34f,		// lead
    3.200f,		// Magnetite
	7.420f,		// Manganese
	1.740f,		// Magnesium
	2.720f,		// Marble
	13.54f,		// Mercury
	10.20f,		// Molybdenum
	8.900f,		// Nickel
	21.45f,		// Platinum
	0.860f,		// Potassium
	2.650f,		// Quartz
	2.300f,		// Sandstone
	2.750f,		// Serpentine
	10.50f,		// Silver
	0.970f,		// Sodium
	7.800f,		// Steel
	2.700f,		// Talc
	1.200f,		// Tar
	6.120f,		// Tellurium
	7.350f,		// Tin
	4.500f,		// Titanium
	19.22f,		// Tungsten
	18.70f,		// Uranium
	5.960f, 	// Vanadium
	1.800f,		// Vinyl
	1.320f,		// Wool
	7.050f,		// Zinc

	0.000f		// Infinite Density - objects will never move
};


static btIDebugDraw::DebugDrawModes g_debugDrawModes[] =
{
	btIDebugDraw::DebugDrawModes::DBG_NoDebug,
	btIDebugDraw::DebugDrawModes::DBG_DrawAabb,
	btIDebugDraw::DebugDrawModes::DBG_MAX_DEBUG_DRAW_MODE
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// a physics implemenation which does nothing.  used if physics is disabled.
class BLOCO_API NullPhysics : public IGamePhysics
{
public:
	NullPhysics() { }
	virtual ~NullPhysics() { }

	// Initialiazation and Maintenance of the Physics World
	virtual bool VInitialize() { return true; }
	virtual void VSyncVisibleScene() { };
	virtual void VOnUpdate( float ) { }

	void AddShape( IActor * actor, btCollisionShape * shape, btScalar mass, enum PhysicsMaterial mat ) {}

	// Initialization of Physics Objects
	virtual void VAddSphere(float radius, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat) { }
	virtual void VAddBox( Vec dimensions, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat) { }
	virtual void VAddPointCloud(Vec *verts, int numPoints, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat) { }
	virtual void VAddTriangleMesh(btCollisionShape* shape, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat) {}
	virtual void VRemoveActor(ActorId id) { }
	virtual void VRayCast( Vec start, Vec end ){}

	// Debugging
	virtual void VRenderDiagnostics(Scene* pScene) { }

	// Physics world modifiers
	virtual void VCreateTrigger(Vec pos, const float dim, int triggerID) { }
	virtual void VApplyForce(Vec dir, float newtons, ActorId aid) { }
	virtual void VApplyTorque(Vec dir, float newtons, ActorId aid) { }
	virtual bool VKinematicMove(Mat mat, ActorId aid) { return true; }

	// Physics actor states
	virtual void VRotateY(ActorId actorId, float angleRadians, float time) { }
	virtual float VGetOrientationY(ActorId actorId) { return 0.0f; }
	virtual void VStopActor(ActorId actorId) { }
	virtual void VSetVelocity(ActorId actorId, Vec vel) { }
	virtual void VTranslate(ActorId actorId, Vec vec) { }
	virtual void VStaticActor( ActorId actorId ) { }
};

#ifndef DISABLE_PHYSICS


/////////////////////////////////////////////////////////////////////////////
// helpers for conversion to and from Bullet's data types
btVector3 BLOCO_API Vec_to_btVector3( Vec vec )
{
	return btVector3( vec.GetX(), vec.GetY(), vec.GetZ() );
}

Vec BLOCO_API btVector3_to_Vec( btVector3 const & btvec )
{
	return Vec( btvec.x(), btvec.y(), btvec.z() );
}

btTransform BLOCO_API Mat_to_btTransform( Mat mat )
{
	// convert from Mat (GameCode) to btTransform (Bullet)
	btMatrix3x3 bulletRotation;
	btVector3 bulletPosition;
	
	// copy rotation matrix
	for ( int row=0; row<3; ++row )
		for ( int column=0; column<3; ++column )
			bulletRotation[row][column] = mat.GetStorage().m[column][row];	// note the reversed indexing (row/column vs. column/row)
																			//  this is because Mats are row-major matrices and
																			//  btMatrix3x3 are column-major.  This reversed indexing
																			//  implicitly transposes (flips along the diagonal) 
																			//  the matrix when it is copied.
	// copy position
	for ( int column=0; column<3; ++column )
		bulletPosition[column] = mat.GetStorage().m[3][column];
		
	return btTransform( bulletRotation, bulletPosition );
}

Mat BLOCO_API btTransform_to_Mat( btTransform const & trans )
{
	Mat mat;
	mat = MatIdentity();

	// convert from btTransform (Bullet) to Mat (GameCode)
	btMatrix3x3 const & bulletRotation = trans.getBasis();
	btVector3 const & bulletPosition = trans.getOrigin();
	
	float tmp[4][4];
	mat.GetArray((float*)&tmp);

	// copy rotation matrix
	for ( int row=0; row<3; ++row )
		for ( int column=0; column<3; ++column )
			tmp[row][column] = bulletRotation[column][row]; 
			          // note the reversed indexing (row/column vs. column/row)
			          //  this is because Mats are row-major matrices and
			          //  btMatrix3x3 are column-major.  This reversed indexing
			          //  implicitly transposes (flips along the diagonal) 
			          //  the matrix when it is copied.
	
	// copy position
	for ( int column=0; column<3; ++column )
		tmp[3][column] = bulletPosition[column];
		
	return	Mat((float*)tmp);
}

/////////////////////////////////////////////////////////////////////////////
// interface that Bullet uses to communicate position and orientation changes
//   back to the game.  note:  this assumes that the actor's center of mass
//   and world position are the same point.  If that was not the case,
//   an additional transformation would need to be stored here to represent
//   that difference.
struct BLOCO_API ActorMotionState : public btMotionState
{
	Mat m_worldToPositionTransform;
	
	ActorMotionState( Mat startingTransform );
	
	// btMotionState interface:  Bullet calls these
	virtual void getWorldTransform( btTransform& worldTrans ) const;
	virtual void setWorldTransform( const btTransform& worldTrans );
};

BLOCO_API ActorMotionState::ActorMotionState( Mat startingTransform )
	: m_worldToPositionTransform( startingTransform )
{ }

void BLOCO_API ActorMotionState::getWorldTransform( btTransform & worldTrans ) const
{
	worldTrans = Mat_to_btTransform( m_worldToPositionTransform );
}

void BLOCO_API ActorMotionState::setWorldTransform( const btTransform& worldTrans )
{
	m_worldToPositionTransform = btTransform_to_Mat( worldTrans );
}

/////////////////////////////////////////////////////////////////////////////
// Simple struct to encapsulate all the physics properties an actor needs.
//typedef btSliderConstraint RotatorConstraint;  // makes life easier when trying out different constraints
typedef btHingeConstraint BLOCO_API RotatorConstraint;
struct BLOCO_API BulletActor
{
	btRigidBody* m_pRigidBody;  // the rigid body associted with this actor (should never be NULL)
	
	float m_desiredDeltaYAngle;
	float m_desiredDeltaYAngleTime;

	explicit BulletActor(btRigidBody* pRigidBody) 
	{ 
		m_pRigidBody = pRigidBody; 
		
		m_desiredDeltaYAngle = 0;
		m_desiredDeltaYAngleTime = 0;
	}
};


/************************************************************************/
/* Debug Drawer                                                         */
/************************************************************************/
struct BLOCO_API SimpleVertex
{
	D3DXVECTOR3 Pos;
};


class BLOCO_API BulletDebugDrawer : public btIDebugDraw
{ 
public: 
	BulletDebugDrawer();
	~BulletDebugDrawer();
	bool Init();
	void PreRender();
	void Render(Scene* pScene);

	// btIDebugDraw interface 
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color); 
	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color); 

	virtual void AddRay(Vec start, Vec end);

	virtual void reportErrorWarning(const char* warningString); 
	virtual void draw3dText(const btVector3& location, const char* textString); 
	virtual void setDebugMode( int debugMode ); 
	virtual int getDebugMode() const; 

private:
	//Render stuff
	cgl::PD3D11Effect			m_pEffect;
	cgl::PD3D11InputLayout		m_pInputLayout;
	cgl::PD3D11EffectTechnique	m_pTechnique;
	ID3D11InputLayout*			m_pVertexLayout;
	ID3D11Buffer*				m_pVertexBuffer;

	cgl::PD3D11EffectVariable  m_pevMatWorld;
	cgl::PD3D11EffectVariable  m_pevMatView;
	cgl::PD3D11EffectVariable  m_pevMatProj;

	vector<D3DXVECTOR3> m_data;

	vector<D3DXVECTOR3> m_rayData;

	DebugDrawModes m_currentDebugMode;
};

 BLOCO_API BulletDebugDrawer::~BulletDebugDrawer()
{
	SAFE_RELEASE(m_pVertexLayout);
	SAFE_RELEASE(m_pVertexBuffer);
}

 BLOCO_API BulletDebugDrawer::BulletDebugDrawer()
{
	m_currentDebugMode = DBG_NoDebug;

	m_pVertexLayout = NULL;
	m_pVertexBuffer = NULL;
}

bool BLOCO_API BulletDebugDrawer::Init()
{
	//Effect
	ifstream myfile;
	int size = 0;
	shared_ptr<char> pData = NULL;


	myfile.open ("simple.fxc", ios::in|ios::binary|ios::ate);
	if (!myfile.is_open())
	{
		return false;
	}

	size = (int)myfile.tellg();
	pData = shared_ptr<char>(new char[size]);
	myfile.seekg(0, std::ios_base::beg);
	myfile.read(pData.get(), size);
	myfile.close();

	m_pEffect = cgl::CD3D11EffectFromMemory::Create(pData.get(), size);
	if (!m_pEffect)
		return false;



	// Obtain the technique
	m_pTechnique = cgl::CD3D11EffectTechniqueFromIndex::Create(m_pEffect, 2);


	if (!m_pTechnique->restore())
		return false;

	// Define the input layout
	D3D11_INPUT_ELEMENT_DESC layout[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	};
	UINT numElements = sizeof( layout ) / sizeof( layout[0] );

	// Create the input layout
	D3DX11_PASS_DESC PassDesc;
	m_pTechnique->get()->GetPassByIndex( 0 )->GetDesc( &PassDesc );
	HRESULT hr = g_pApp->GetDevice()->CreateInputLayout( layout, numElements, PassDesc.pIAInputSignature,
		PassDesc.IAInputSignatureSize, &m_pVertexLayout );
	if( FAILED( hr ) )
		return false;

	m_pevMatWorld = cgl::CD3D11EffectVariableFromName::Create(m_pEffect, "matWorld");
	m_pevMatView  = cgl::CD3D11EffectVariableFromName::Create(m_pEffect, "matView");
	m_pevMatProj  = cgl::CD3D11EffectVariableFromName::Create(m_pEffect, "matProj");

	if ( !m_pevMatWorld->restore() ||
		 !m_pevMatView->restore() ||
		 !m_pevMatProj->restore() )
	{
		return false;
	}

	return true;
}

void BLOCO_API BulletDebugDrawer::drawLine( const btVector3& from, const btVector3& to, const btVector3& color )
{
	m_data.push_back( D3DXVECTOR3( from.getX(), from.getY(), from.getZ() ) );
	m_data.push_back( D3DXVECTOR3( to.getX(), to.getY(), to.getZ() ) );
}

void BLOCO_API BulletDebugDrawer::drawContactPoint( const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color )
{
	btVector3 const startPoint = PointOnB;
	btVector3 const endPoint = PointOnB + normalOnB * distance;

	drawLine( startPoint, endPoint, color );
}

void BLOCO_API BulletDebugDrawer::reportErrorWarning( const char* warningString )
{
	printf("BulletDebugDrawer: %s",warningString);
}

void BLOCO_API BulletDebugDrawer::draw3dText( const btVector3& location, const char* textString )
{
	printf("BulletDebugDrawer: %s",textString);
}

void BLOCO_API BulletDebugDrawer::setDebugMode( int debugMode )
{
	m_currentDebugMode = g_debugDrawModes[ debugMode ];
}

int BLOCO_API BulletDebugDrawer::getDebugMode() const
{
	return DBG_MAX_DEBUG_DRAW_MODE;
}

float pos1 = 100.0f;

void BLOCO_API BulletDebugDrawer::Render(Scene* pScene)
{
	HRESULT hr;

	m_pevMatWorld->get()->AsMatrix()->SetMatrix(MatIdentity().GetArray());
	m_pevMatProj->get()->AsMatrix()->SetMatrix(pScene->GetCamera()->GetProjection().GetArray());
	m_pevMatView->get()->AsMatrix()->SetMatrix(pScene->GetCamera()->GetView().GetArray());


	// Set the input layout
	g_pApp->GetDeviceContext()->IASetInputLayout( m_pVertexLayout );

	D3D11_BUFFER_DESC bd;
	bd.Usage = D3D11_USAGE_DEFAULT;
	bd.ByteWidth = sizeof( float ) * 3 * m_data.size();
	bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	bd.CPUAccessFlags = 0;
	bd.MiscFlags = 0;
	D3D11_SUBRESOURCE_DATA InitData;
	InitData.pSysMem = m_data.data();
	hr = g_pApp->GetDevice()->CreateBuffer( &bd, &InitData, &m_pVertexBuffer );
	if( FAILED( hr ) )
		return;

	// Set vertex buffer
	UINT stride = sizeof( float ) * 3;
	UINT offset = 0;
	g_pApp->GetDeviceContext()->IASetVertexBuffers( 0, 1, &m_pVertexBuffer, &stride, &offset );

	// Set primitive topology
	g_pApp->GetDeviceContext()->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_LINELIST );



	// Render a triangle
	D3DX11_TECHNIQUE_DESC techDesc;
	m_pTechnique->get()->GetDesc( &techDesc );
	for( UINT p = 0; p < techDesc.Passes; ++p )
	{
		m_pTechnique->get()->GetPassByIndex( p )->Apply( 0, g_pApp->GetDeviceContext() );
		g_pApp->GetDeviceContext()->Draw( m_data.size(), 0 );
	}
}

void BLOCO_API BulletDebugDrawer::PreRender()
{
	m_data.clear();

	for (int i = 0; i < m_rayData.size() ; i++)
	{
		m_data.push_back(m_rayData[i]);
	}
}

void BulletDebugDrawer::AddRay( Vec start, Vec end )
{
	m_rayData.push_back( D3DXVECTOR3( start.GetX(), start.GetY(), start.GetZ() ) );
	m_rayData.push_back( D3DXVECTOR3( end.GetX(), end.GetY(), end.GetZ() ) );
}













/////////////////////////////////////////////////////////////////////////////
// BaseGamePhysics Description
//
//   The implementation of IGamePhysics interface using the Bullet SDK.
//
//       - Chapter 15, page 544
/////////////////////////////////////////////////////////////////////////////

class BLOCO_API BulletPhysics : public IGamePhysics
{
	public:

	// use auto pointers to automatically call delete on these objects
	//   during ~BulletPhysics
	
	// these are all of the objects that Bullet uses to do its work.
	//   see BulletPhysics::VInitialize() for some more info.
	std::auto_ptr<btDynamicsWorld>				   m_dynamicsWorld;
	std::auto_ptr<btBroadphaseInterface>           m_broadphase;
	std::auto_ptr<btCollisionDispatcher>           m_dispatcher;
	std::auto_ptr<btConstraintSolver>              m_solver;
	std::auto_ptr<btDefaultCollisionConfiguration> m_collisionConfiguration;

	shared_ptr<BulletDebugDrawer> m_debugDrawer; 

	EventListenerPtr m_pListener;

	IFW1Factory* m_pFontFactory;
	CD3D11Font*	 m_pFont;

	DWORD m_dProcessTime;

	// keep track of the existing rigid bodies:  To check them for updates
	//   to the actors' positions, and to remove them when their lives are over.
	typedef std::map<ActorId, BulletActor*> ActorIDToBulletActorMap;
	ActorIDToBulletActorMap m_actorBodies;
	
	// also keep a map to get the actor id from the btRigidBody*
	typedef std::map<btRigidBody const *, ActorId> RigidBodyToActorIDMap;
	RigidBodyToActorIDMap m_rigidBodyToActorId;
	
	// data used to store which collision pair (bodies that are touching) need
	//   Collision events sent.  When a new pair of touching bodies are detected,
	//   they are added to m_previousTickCollisionPairs and an event is sent.
	//   When the pair is no longer detected, they are removed and another event
	//   is sent.
	typedef std::pair< btRigidBody const *, btRigidBody const * > CollisionPair;
	typedef std::set< CollisionPair > CollisionPairs;
	CollisionPairs m_previousTickCollisionPairs;
	
	// helpers for sending events relating to collision pairs
	void SendCollisionPairAddEvent( btPersistentManifold const * manifold, btRigidBody const * body0, btRigidBody const * body1 );
	void SendCollisionPairRemoveEvent( btRigidBody const * body0, btRigidBody const * body1 );
	
	
	// helper for cleaning up objects
	void RemoveCollisionObject( btCollisionObject * removeMe );
	
	// find the BulletActor object with the given actor ID
	BulletActor* FindBulletActor(ActorId id) const;
	
	// find the actor ID associated with the given body
	optional<ActorId> FindActorID( btRigidBody const * ) const;
	
	// callback from bullet for each physics time step.  set in VInitialize
	static void BulletInternalTickCallback( btDynamicsWorld * const world, btScalar const timeStep );
	
public:
	BulletPhysics();
	virtual ~BulletPhysics();

	// Initialiazation and Maintenance of the Physics World
	virtual bool VInitialize();
	virtual void VSyncVisibleScene();
	virtual void VOnUpdate( float deltaSeconds ); 

	// common functionality used by VAddSphere, VAddBox, etc
	void AddShape( IActor * actor, btCollisionShape * shape, btScalar mass, enum PhysicsMaterial mat );

	// find the rigid body associated with the given actor ID
	btRigidBody * FindActorBody( ActorId id ) const;

	// Initialization of Physics Objects
	virtual void VAddSphere(float radius, IActor *actor, float specificGravity, enum PhysicsMaterial mat);
	virtual void VAddBox( Vec dimensions, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat);
	virtual void VAddPointCloud(Vec *verts, int numPoints, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat);
	virtual void VRemoveActor(ActorId id);
	virtual void VRayCast( Vec start, Vec end );
	virtual void VAddTriangleMesh(btCollisionShape* shape, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat);

	// Debugging
	virtual void VRenderDiagnostics(Scene* pScene);
	virtual void VSetDebugMode( int mode );

	// Physics world modifiers
	virtual void VCreateTrigger(Vec pos, const float dim, int triggerID);
	virtual void VApplyForce(Vec dir, float newtons, ActorId aid);
	virtual void VApplyTorque(Vec dir, float newtons, ActorId aid);
	virtual bool VKinematicMove(Mat mat, ActorId aid);
	
	virtual void VRotateY(ActorId actorId, float angleRadians, float time);
	virtual float VGetOrientationY(ActorId actorId);
	virtual void VStopActor(ActorId actorId);
	virtual void VSetVelocity(ActorId actorId, Vec vel);
	virtual void VTranslate(ActorId actorId, Vec vec);
	virtual void VStaticActor( ActorId actorId );
};

/////////////////////////////////////////////////////////////////////////////
 BLOCO_API BulletPhysics::BulletPhysics()
{
	m_pFont = NULL;
	m_pFontFactory = NULL;

	m_pListener =  EventListenerPtr( DEBUG_CLIENTBLOCK PhysicsEventListener( this ) );

	safeAddListener( m_pListener, EvtData_PhysTrigger_Enter::sk_EventType );
	safeAddListener( m_pListener, EvtData_PhysTrigger_Leave::sk_EventType );
	safeAddListener( m_pListener, EvtData_PhysCollision::sk_EventType );
	safeAddListener( m_pListener, EvtData_PhysSeparation::sk_EventType );
	safeAddListener( m_pListener, EvtData_RayCast::sk_EventType );

	// auto_ptr<> will automatically initialize themselves to NULL
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 5550
//
 BLOCO_API BulletPhysics::~BulletPhysics()
{
	// delete any physics objects which are still in the world
	
	SAFE_DELETE(m_pFont);


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
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 548
//
bool BLOCO_API BulletPhysics::VInitialize()
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
	                                                    
	// also set up the functionality for debug drawing
 	m_debugDrawer.reset( new BulletDebugDrawer );
 	m_dynamicsWorld->setDebugDrawer( m_debugDrawer.get() );
	
	if(!m_debugDrawer->Init())
		return false;



	m_pFontFactory = NULL;
	FW1CreateFactory(FW1_VERSION, &m_pFontFactory);
	if(!m_pFontFactory)
	{
		return S_FALSE;
	}

	if(!m_pFont)
	{
		m_pFont = DEBUG_CLIENTBLOCK CD3D11Font(L"Arial");
	}
	if(!m_pFont->VCreate(m_pFontFactory))
	{
		return S_FALSE;
	}


	// and set the internal tick callback to our own method "BulletInternalTickCallback"
	m_dynamicsWorld->setInternalTickCallback( BulletInternalTickCallback );
	m_dynamicsWorld->setWorldUserInfo( this );
	
	return true;
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 551
//
void BLOCO_API BulletPhysics::VOnUpdate( float const deltaSeconds )
{
	// Bullet uses an internal fixed timestep (default 1/60th of a second)
	//   We pass in 4 as a max number of sub steps.  Bullet will run the simulation
	//   in increments of the fixed timestep until "deltaSeconds" amount of time has
	//   passed, but will only run a maximum of 4 steps this way.

	DWORD startTickCount = GetTickCount();

	m_dynamicsWorld->stepSimulation( deltaSeconds,10 );

	m_dProcessTime = GetTickCount() - startTickCount;
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 552
//
void BLOCO_API BulletPhysics::VSyncVisibleScene()
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


		btVector3 bulletPosition;
		shared_ptr<IActor> gameActor = g_pApp->m_pGame->VGetActor( id );
		if ( gameActor )
		{
			btTransform trans;
			motionState->getWorldTransform(trans);

			float a[16];
			float b[16];

			Mat current = btTransform_to_Mat(trans);
			
			current.GetArray(a);
			gameActor->VGetMat()->GetArray(b);

			for (unsigned int i = 0; i < 16 ; i++)
			{
				if ( a[i] != b[i] )
				{
					// bullet has moved the actor's physics object.  update the actor.
					safeQueEvent( IEventDataPtr( new EvtData_Move_Actor( id, current ) ) );
					break;
				}
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 553
//
void BLOCO_API BulletPhysics::AddShape( IActor * const actor, 
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
	btDefaultMotionState* motionState = new btDefaultMotionState( Mat_to_btTransform( *actor->VGetMat() ));

	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, motionState, shape, localInertia );
	
	// set up the materal properties
	rbInfo.m_restitution = g_PhysicsMaterials[mat].m_restitution;
	rbInfo.m_friction    = g_PhysicsMaterials[mat].m_friction;

	btRigidBody * const body = new btRigidBody(rbInfo);
	
	m_dynamicsWorld->addRigidBody( body );

	// create the BulletActor
	BulletActor* pBulletActor = new BulletActor(body);
	
	// add it to the collection to be checked for changes in VSyncVisibleScene
	m_actorBodies[actorID] = pBulletActor;
	m_rigidBodyToActorId[body] = actorID;
}

/////////////////////////////////////////////////////////////////////////////
void BLOCO_API BulletPhysics::RemoveCollisionObject( btCollisionObject * const removeMe )
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
btRigidBody BLOCO_API * BulletPhysics::FindActorBody( ActorId const id ) const
{
	BulletActor* pBulletActor = FindBulletActor(id);
	if (pBulletActor)
		return pBulletActor->m_pRigidBody;
	return NULL;
}

/////////////////////////////////////////////////////////////////////////////
BulletActor BLOCO_API* BulletPhysics::FindBulletActor( ActorId const id ) const
{
	ActorIDToBulletActorMap::const_iterator found = m_actorBodies.find( id );
	if ( found != m_actorBodies.end() )
		return found->second;

	return NULL;
}

/////////////////////////////////////////////////////////////////////////////
optional<ActorId> BLOCO_API BulletPhysics::FindActorID( btRigidBody const * const body ) const
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
void BLOCO_API BulletPhysics::VAddSphere( float const radius, 
                                IActor * const actor, 
                                float const specificGravity, 
                                enum PhysicsMaterial const mat)
{
	assert( actor );
	
	// create the collision body, which specifies the shape of the object
	btSphereShape * const collisionShape = new btSphereShape( radius );
	
	// calculate absolute mass from specificGravity
	float const volume = (4.f / 3.f) * D3DX_PI * radius * radius * radius;
	btScalar const mass = volume * specificGravity;
	
	AddShape( actor, collisionShape, mass, mat );
}

/////////////////////////////////////////////////////////////////////////////
void BLOCO_API BulletPhysics::VAddBox(Vec dimensions, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat)
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
void BLOCO_API BulletPhysics::VAddPointCloud(Vec *verts, int numPoints, IActor *actor, float specificGravity, enum PhysicsMaterial mat)
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

void BLOCO_API BulletPhysics::VAddTriangleMesh( btCollisionShape* shape, IActor *actor, float specificGravity, enum PhysicsMaterial mat )
{
	assert( actor );

	// approximate absolute mass using bounding box
	btVector3 aabbMin(0,0,0), aabbMax(0,0,0);
	shape->getAabb( btTransform::getIdentity(), aabbMin, aabbMax );

	btVector3 const aabbExtents = aabbMax - aabbMin;

	float const volume = aabbExtents.x() * aabbExtents.y() * aabbExtents.z();
	btScalar const mass = volume * specificGravity;

	AddShape( actor, shape, mass, mat );
}

/////////////////////////////////////////////////////////////////////////////
void BLOCO_API BulletPhysics::VRemoveActor(ActorId id)
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
void BLOCO_API BulletPhysics::VRenderDiagnostics(Scene* pScene)
{
	m_debugDrawer->PreRender();
	m_dynamicsWorld->debugDrawWorld();
	m_debugDrawer->Render(pScene);

	stringstream stm;

	stm << "Physics Update Time:  " << m_dProcessTime << " ms\n" ;
	stm << "Physics Object Count: " << m_dynamicsWorld->getNumCollisionObjects() << "\n" ;

	//Print out some Debug stuff
	m_pFont->VRender(g_pApp->GetDeviceContext(), stm.str(), 14, Vec(g_pApp->GetScreenWidth() - 300, 50), Col(1, 1, 1, 1));
}

/////////////////////////////////////////////////////////////////////////////
//
//   - Chapter 15, page 556
//
void BLOCO_API BulletPhysics::VCreateTrigger(Vec pos, const float dim, int triggerID)
{
	// create the collision body, which specifies the shape of the object
	btBoxShape * const boxShape = new btBoxShape( Vec_to_btVector3( Vec(dim,dim,dim) ) );
	
	// triggers are immoveable.  0 mass signals this to Bullet.
	btScalar const mass = 0;

	// set the initial position of the body from the actor
	Mat triggerTrans = MatIdentity();
	MatSetPosition(triggerTrans, pos);
	ActorMotionState * const myMotionState = new ActorMotionState( triggerTrans );
	
	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, boxShape, btVector3(0,0,0) );
	btRigidBody * const body = new btRigidBody(rbInfo);
	
	m_dynamicsWorld->addRigidBody( body );
	
	// a trigger is just a box that doesn't collide with anything.  That's what "CF_NO_CONTACT_RESPONSE" indicates.
	body->setCollisionFlags( body->getCollisionFlags() | btRigidBody::CF_NO_CONTACT_RESPONSE );
	body->setUserPointer( new int(triggerID) );
}

/////////////////////////////////////////////////////////////////////////////
void BLOCO_API BulletPhysics::VApplyForce(Vec dir, float newtons, ActorId aid)
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
void BLOCO_API BulletPhysics::VApplyTorque(Vec dir, float magnitude, ActorId aid)
{
	if ( btRigidBody * const body = FindActorBody( aid ) )
	{
		btVector3 const torque( dir.GetX() * magnitude,
		                        dir.GetY() * magnitude,
		                        dir.GetZ() * magnitude );

		body->applyTorqueImpulse( torque );
	}
}

/////////////////////////////////////////////////////////////////////////////
bool BLOCO_API BulletPhysics::VKinematicMove(Mat mat, ActorId aid)
{
	if ( btRigidBody * const body = FindActorBody( aid ) )
	{
		// warp the body to the new position
		body->getMotionState()->setWorldTransform( Mat_to_btTransform( mat ) );
		body->setWorldTransform( Mat_to_btTransform( mat ) );
		return true;
	}
	
	return false;
}

/////////////////////////////////////////////////////////////////////////////
void BLOCO_API BulletPhysics::VRotateY( ActorId const actorId, float const deltaAngleRadians, float const time )
{
	if ( BulletActor * const actor = FindBulletActor( actorId ) )
	{
		// set the parameters for the turning that will be handled in BulletInternalTickCallback
		actor->m_desiredDeltaYAngle = deltaAngleRadians;
		actor->m_desiredDeltaYAngleTime = time;
	}
}


/////////////////////////////////////////////////////////////////////////////
float BLOCO_API BulletPhysics::VGetOrientationY(ActorId actorId)
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
void BLOCO_API BulletPhysics::VStopActor(ActorId actorId)
{
	BulletActor* pBulletActor = FindBulletActor(actorId);
	assert(pBulletActor);
	// [rez] None of these actually do what I want....
	//pBulletActor->m_pRigidBody->clearForces();
	//pBulletActor->m_pRigidBody->setFriction(FLT_MAX);  // this doesn't stop the actor.....
	pBulletActor->m_pRigidBody->setLinearVelocity(btVector3(0,0,0));
}

/////////////////////////////////////////////////////////////////////////////
void BLOCO_API BulletPhysics::VSetVelocity(ActorId actorId, Vec vel)
{
	BulletActor* pBulletActor = FindBulletActor(actorId);
	assert(pBulletActor);
	btVector3 btVel = Vec_to_btVector3(vel);
	pBulletActor->m_pRigidBody->setLinearVelocity(btVel);
}

/////////////////////////////////////////////////////////////////////////////
void BLOCO_API BulletPhysics::VTranslate(ActorId actorId, Vec vec)
{
	BulletActor* pBulletActor = FindBulletActor(actorId);
	assert(pBulletActor);
	btVector3 btVec = Vec_to_btVector3(vec);
	pBulletActor->m_pRigidBody->translate(btVec);
}

void BLOCO_API BulletPhysics::VStaticActor( ActorId actorId )
{
}


/////////////////////////////////////////////////////////////////////////////
// This function is called after bullet performs its internal update.  We
//   use it to detect collisions between objects for Game code.
//
//   - Chapter 15, page 560
//
void BLOCO_API BulletPhysics::BulletInternalTickCallback( btDynamicsWorld * const world, btScalar const timeStep )
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
void BLOCO_API BulletPhysics::SendCollisionPairAddEvent( btPersistentManifold const * manifold, btRigidBody const * const body0, btRigidBody const * const body1 )
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
		safeQueEvent( IEventDataPtr(new EvtData_PhysTrigger_Enter( triggerId, FindActorID( otherBody ) )));

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
		safeQueEvent(IEventDataPtr(new EvtData_PhysCollision(*id0, *id1,sumNormalForce,	sumFrictionForce, collisionPoints)));
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void BLOCO_API BulletPhysics::SendCollisionPairRemoveEvent( btRigidBody const * const body0, btRigidBody const * const body1 )
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
		safeQueEvent
		(
			IEventDataPtr
			(
				new EvtData_PhysTrigger_Leave( triggerId, FindActorID( otherBody ) )
			)
		);
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
		
		safeQueEvent(IEventDataPtr(new EvtData_PhysSeparation( *id0, *id1 )));
	}
}

void BLOCO_API BulletPhysics::VSetDebugMode( int mode )
{
	//m_debugDrawer->setDebugMode( mode );
}

void BulletPhysics::VRayCast( Vec start, Vec end )
{
	btVector3 btStart = Vec_to_btVector3(start);
	btVector3 btEnd   = Vec_to_btVector3(end);

	btCollisionWorld::ClosestRayResultCallback RayCallback(btStart, btEnd);

	// Perform raycast
	m_dynamicsWorld->rayTest(btStart, btEnd, RayCallback);

	m_debugDrawer->AddRay(start,end);

	if(RayCallback.hasHit()) 
	{
		Vec hitPos    = btVector3_to_Vec(RayCallback.m_hitPointWorld);
		Vec hitNormal = btVector3_to_Vec(RayCallback.m_hitNormalWorld);


		btRigidBody* body = btRigidBody::upcast(RayCallback.m_collisionObject);

		safeQueEvent( IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_RayCast_Result( *FindActorID( body ),hitPos,hitNormal )));

		// Do some clever stuff here
	}
}

#endif // #ifndef DISABLE_PHYSICS

/////////////////////////////////////////////////////////////////////////////
//
// CreateGamePhysics 
//   The free function that creates an object that implements the IGamePhysics interface.
//
IGamePhysics BLOCO_API *CreateGamePhysics()
{
	std::auto_ptr<IGamePhysics> gamePhysics;
	gamePhysics.reset( new BulletPhysics );

	if (gamePhysics.get() && !gamePhysics->VInitialize())
	{
		// physics failed to initialize.  delete it.
		gamePhysics.reset();
	}

	return gamePhysics.release();
}

IGamePhysics BLOCO_API *CreateNullPhysics()
{
	std::auto_ptr<IGamePhysics> gamePhysics;
	gamePhysics.reset( new NullPhysics );
	if (gamePhysics.get() && !gamePhysics->VInitialize())
	{
		// physics failed to initialize.  delete it.
		gamePhysics.reset();
	}

	return gamePhysics.release();
}

/////////////////////////////////////////////////////////////////////////////
//
// SpecificGravity - return the specific gravity value for the given density
//
float BLOCO_API SpecificGravity(enum PhysicsDensity substance)
{
	assert(substance<PhysDens_MaxDensities && _T("Parameter out of range"));
	return g_DensityTable[substance];
}
