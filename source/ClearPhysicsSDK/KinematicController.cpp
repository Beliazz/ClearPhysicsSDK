#include "ClearPhysicsSDK.h"


CKinematicController::CKinematicController(shared_ptr<IActor> actor) :
	m_actor(actor)
{
}

CKinematicController::~CKinematicController()
{

}

void CKinematicController::Update(float const deltaSeconds)
{

}

void CKinematicController::AddBone( CBone* pBone )
{
	m_pBones.push_back(pBone);
}

void CKinematicController::SetDynmamicWorld( btDynamicsWorld* world )
{
	m_dynamicsWorld = world;
}

CBone::CBone( Mat bindposeMatrix, Mat globalMatrix, int index, int parentIndex, BoneCollisionShapeType type ):
	m_matBindPose(bindposeMatrix),
	m_matGlobalMatrix(globalMatrix),
	m_Index(index),
	m_ParentIndex(parentIndex),
	m_Type(type)
{
}
