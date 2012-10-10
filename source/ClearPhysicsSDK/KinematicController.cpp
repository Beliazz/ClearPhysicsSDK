#include "ClearPhysicsSDK.h"


btRigidBody* CreateLocalRigidBody(btCollisionShape* shape, float mass, Mat local )
{
	btVector3 localInertia( 0.f, 0.f, 0.f );
	if ( mass > 0.f )
		shape->calculateLocalInertia( mass, localInertia );

	// set the initial position of the body from the actor
	btDefaultMotionState* motionState = DEBUG_CLIENTBLOCK btDefaultMotionState( Mat_to_btTransform( local ));

	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, motionState, shape, localInertia );

	// set up the materal properties
	rbInfo.m_restitution	 = 0.05f;
	rbInfo.m_friction		 = 0.9f;

	return new btRigidBody(rbInfo);
}


CKinematicController::CKinematicController(shared_ptr<IActor> actor) :
	m_actor(actor)
{
	m_pCompoundShape		  = NULL;
	m_pCurrentAnimationsTrack = NULL;

	m_fTime = 0.0f;
	m_fAnimationsSpeed = 1.0f;
	m_iLastAnimationsKey = 0;

	m_fProcessTime = 0.0f;
}

CKinematicController::~CKinematicController()
{

}

void CKinematicController::Update(float const deltaSeconds)
{
	m_fTime += deltaSeconds * m_fAnimationsSpeed;

	if (m_fTime > m_pCurrentAnimationsTrack->GetEndTime() )
	{
		//m_fTime = Start Time
		m_iLastAnimationsKey = 0;
		m_fTime = m_pCurrentAnimationsTrack->GetStartTime();
	}

	BuildAnimation();
}

void CKinematicController::BuildAnimation()
{
	DWORD firstKeyIndex = 0;
	DWORD nextKeyIndex  = 0;

	DWORD boneCount = GetBoneCount();

	if (m_fTime < m_pCurrentAnimationsTrack->GetKey(m_iLastAnimationsKey)->GetTime())
		m_iLastAnimationsKey = 0;

	for (unsigned int i = m_iLastAnimationsKey; i < m_pCurrentAnimationsTrack->GetKeyCount() ; i+=m_pBones.size())
	{
		if (m_fTime < m_pCurrentAnimationsTrack->GetKey(i)->GetTime())
		{
			firstKeyIndex = i;
			m_iLastAnimationsKey = i;

			//Check if its the Last Key.
			if( i + boneCount  >= m_pCurrentAnimationsTrack->GetKeyCount()  )
			{
				nextKeyIndex = i;
			}
			else
			{
				nextKeyIndex  = i + boneCount;
			}

			break;
		}
	}

	for (unsigned int i = 0; i < boneCount ; i++)
	{
		Mat a = m_pCurrentAnimationsTrack->GetKey( firstKeyIndex + i )->GetBoneSpaceMatrix();
		Mat b = m_pCurrentAnimationsTrack->GetKey( nextKeyIndex + i )->GetBoneSpaceMatrix();

		float T1 = m_pCurrentAnimationsTrack->GetKey( firstKeyIndex + i )->GetTime();
		float T2 = m_pCurrentAnimationsTrack->GetKey( nextKeyIndex + i )->GetTime();

		//Last key
		if ( firstKeyIndex == nextKeyIndex || T1 == T2 )
		{
			//m_BoneGlobals[i] = a;
			*m_pBones[i]->GetGlobalMatrix() = a;
			continue;
		}

		float s = (m_fTime - T1) / (T2 - T1);

		*m_pBones[i]->GetGlobalMatrix() = a;
		m_pRigidBodysJoints[i]->setWorldTransform(m_pCurrentAnimationsTrack->GetKey( firstKeyIndex + i )->GetbtTransform());
	} 
}

void CKinematicController::AddBone( CBone* pBone, Vec size )
{
	m_pBones.push_back(pBone);

	btBoxShape *    const boxShape    = new btBoxShape( Vec_to_btVector3( size ) );
	btCapsuleShape* const capsulShape = new btCapsuleShape( size.GetY(),size.GetX() );

	m_BoneGlobals.push_back(*pBone->GetGlobalMatrix());


	btSphereShape* sphereShape = new btSphereShape(10.0f);
	btRigidBody*   body = CreateLocalRigidBody(sphereShape, 0.0f, pBone->GetBindPose()*(*pBone->GetGlobalMatrix()) );

	body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE );


	m_pRigidBodysJoints.push_back( body );
	m_dynamicsWorld->addRigidBody( body );

	//m_pCompoundShape->addChildShape(Mat_to_btTransform(pBone->GetBindPose()*pBone->GetGlobalMatrix()),boxShape);
	//m_pCompoundShape->updateChildTransform(m_pCompoundShape->getNumChildShapes()-1,Mat_to_btTransform(pBone->GetGlobalMatrix()));
}

void CKinematicController::AddAnimationTrack( CAnimationTrack* track )
{
	std::map<string,CAnimationTrack*>::iterator it = m_animationTracks.find(track->GetName());

	if (it != m_animationTracks.end())
	{
		printf("[PHYSIC] CKinematicController::AddAnimationTrack( CAnimationTrack* track ) track already exists (%s)",track->GetName().c_str());
		return;
	}

	m_animationTracks[track->GetName()] = track;

	if (!m_pCurrentAnimationsTrack)
		m_pCurrentAnimationsTrack = track;
}
	
void CKinematicController::SetDynmamicWorld( btDynamicsWorld* world )
{
//	if (!m_pCompoundShape)
//		m_pCompoundShape = new btCompoundShape();

	m_dynamicsWorld = world;

// 	float mass = 0.0f;
// 	btVector3 localInertia( 0.f, 0.f, 0.f );
// 	if ( mass > 0.f )
// 		m_pCompoundShape->calculateLocalInertia( mass, localInertia );
// 
// 	// set the initial position of the body from the actor
// 	btDefaultMotionState* motionState = DEBUG_CLIENTBLOCK btDefaultMotionState( Mat_to_btTransform( MatIdentity() ));
// 
// 	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, motionState, m_pCompoundShape, localInertia );
// 
// 	// set up the materal properties
// 	rbInfo.m_restitution	 = 0.05f;
// 	rbInfo.m_friction		 = 0.9f;
// 
// 	m_pRigidBody = new btRigidBody(rbInfo);

//	m_dynamicsWorld->addRigidBody( m_pRigidBody );
}

void CKinematicController::SetLocalMatrix( Mat matLocal )
{
	//m_pRigidBody->setWorldTransform(Mat_to_btTransform(matLocal));
}





CBone::CBone( Mat bindposeMatrix, Mat* globalMatrix, string name, string parentName, BoneCollisionShapeType type ):
	m_matBindPose(bindposeMatrix),
	m_matGlobalMatrix(globalMatrix),
	m_btTransformBindPose(Mat_to_btTransform(m_matBindPose)),
	m_sName(name),
	m_sParentName(parentName),
	m_Type(type)
{

}


CAnimationTrack::CAnimationTrack( string name, float startTime, float endTime ):
	m_sName(name),
	m_fStartTime(startTime),
	m_fEndTime(endTime)	
{
}

CAnimationTrack::~CAnimationTrack()
{

}

void CAnimationTrack::SetAnimationKeysFromBinary( void* keyData, int count )
{
	m_animationsKeys.resize(count);

	for (int i = 0; i < count ; i++)
	{
		m_animationsKeys[i] = new CAnimationsKey();
	}

	memcpy(m_animationsKeys.data(),keyData,sizeof(CAnimationsKey)*count);
}


CAnimationsKey::CAnimationsKey( int boneIndex, float timeStamp, Mat bonespaceMatrix, Mat globalMatrix ):
	m_iBoneIndex(boneIndex),
	m_fTimeStamp(timeStamp),
	m_matBoneSpace(bonespaceMatrix),
	m_btTransform(Mat_to_btTransform(globalMatrix))
{
}

CAnimationsKey::~CAnimationsKey()
{

}
