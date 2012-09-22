#ifndef PhyDebugDrawer_H__
#define PhyDebugDrawer_H__

#include "ClearPhysicsSDK.h"

class BulletDebugDrawer : public btIDebugDraw
{ 
public: 
	BulletDebugDrawer();
	~BulletDebugDrawer();
	bool Init();
	void PreRender();
	void Render(IScene* pScene);

	// btIDebugDraw interface 
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color); 
	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color); 

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

	ID3D11Device*		 m_pDevice;
	ID3D11DeviceContext* m_pContext;

	DebugDrawModes m_currentDebugMode;
};

#endif //PhyMathConversion_H__