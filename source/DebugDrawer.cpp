#include "ClearPhysicsSDK.h"



BulletDebugDrawer::~BulletDebugDrawer()
{
	SAFE_RELEASE(m_pVertexLayout);
	SAFE_RELEASE(m_pVertexBuffer);
}

BulletDebugDrawer::BulletDebugDrawer()
{
	m_currentDebugMode = DBG_DrawWireframe;

	m_pVertexLayout = NULL;
	m_pVertexBuffer = NULL;
}

bool BulletDebugDrawer::Init()
{
	//Effect
	ifstream myfile;
	int size = 0;
	shared_ptr<char> pData = NULL;

	myfile.open ("DebugDraw.fxc", ios::in|ios::binary|ios::ate);
	if (!myfile.is_open())
	{
		return false;
	}

	size = (int)myfile.tellg();
	pData = shared_ptr<char>(DEBUG_CLIENTBLOCK char[size]);
	myfile.seekg(0, std::ios_base::beg);
	myfile.read(pData.get(), size);
	myfile.close();

	m_pEffect = cgl::CD3D11EffectFromMemory::Create(pData.get(), size);
	if (!m_pEffect)
		return false;

	m_pDevice  = m_pEffect->getDevice().get()->GetDevice();
	m_pContext = m_pEffect->getDevice().get()->GetContext();


	// Obtain the technique
	m_pTechnique = cgl::CD3D11EffectTechniqueFromIndex::Create(m_pEffect, 0);

	if (!m_pTechnique->restore())
		return false;

	// Define the input layout
	D3D11_INPUT_ELEMENT_DESC layout[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	};
	UINT numElements = sizeof( layout ) / sizeof( layout[0] );

	// Create the input layout
	D3DX11_PASS_DESC PassDesc;
	m_pTechnique->get()->GetPassByIndex( 0 )->GetDesc( &PassDesc );
	HRESULT hr = m_pDevice->CreateInputLayout( layout, numElements, PassDesc.pIAInputSignature,
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

void BulletDebugDrawer::drawLine( const btVector3& from, const btVector3& to, const btVector3& color )
{
	PosColVertex vertexA;
	vertexA.position = D3DXVECTOR3( from.getX(), from.getY(), from.getZ() );
	vertexA.color	 = D3DXVECTOR3( color.getX(), color.getY(), color.getZ() );

	PosColVertex vertexB;
	vertexB.position = D3DXVECTOR3( to.getX(), to.getY(), to.getZ() );
	vertexB.color	 = D3DXVECTOR3( color.getX(), color.getY(), color.getZ() );

	m_data.push_back( vertexA );
	m_data.push_back( vertexB );
}

void BulletDebugDrawer::drawContactPoint( const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color )
{
	btVector3 const startPoint = PointOnB;
	btVector3 const endPoint = PointOnB + normalOnB * distance;

	drawLine( startPoint, endPoint, color );
}

void BulletDebugDrawer::reportErrorWarning( const char* warningString )
{
	printf( warningString );
}

void BulletDebugDrawer::draw3dText( const btVector3& location, const char* textString )
{

}

void BulletDebugDrawer::setDebugMode( int debugMode )
{
	m_currentDebugMode = DBG_DrawAabb;
}

int BulletDebugDrawer::getDebugMode() const
{
	return m_currentDebugMode;
}

void BulletDebugDrawer::Render(IScene* pScene)
{
	m_pevMatWorld->get()->AsMatrix()->SetMatrix(MatIdentity().GetArray());
	m_pevMatView->get()->AsMatrix()->SetMatrix(pScene->GetCamera()->GetView().GetArray());
	m_pevMatProj->get()->AsMatrix()->SetMatrix(pScene->GetCamera()->GetProjection().GetArray());

	// Set the input layout
	m_pContext->IASetInputLayout( m_pVertexLayout );
	
	D3D11_BUFFER_DESC bd;
	bd.Usage = D3D11_USAGE_DEFAULT;
	bd.ByteWidth = sizeof( float ) * 6 * m_data.size();
	bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	bd.CPUAccessFlags = 0;
	bd.MiscFlags = 0;
	D3D11_SUBRESOURCE_DATA InitData;
	InitData.pSysMem = m_data.data();
	HRESULT hr = m_pDevice->CreateBuffer( &bd, &InitData, &m_pVertexBuffer );
	if( FAILED( hr ) )
		return;

	// Set vertex buffer
	UINT stride = sizeof( float ) * 6;
	UINT offset = 0;
	m_pContext->IASetVertexBuffers( 0, 1, &m_pVertexBuffer, &stride, &offset );

	// Set primitive topology
	m_pContext->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_LINELIST );

	// Render lines
	D3DX11_TECHNIQUE_DESC techDesc;
	m_pTechnique->get()->GetDesc( &techDesc );
	for( UINT p = 0; p < techDesc.Passes; ++p )
	{
		m_pTechnique->get()->GetPassByIndex( p )->Apply( 0, m_pContext );
		m_pContext->Draw( m_data.size(), 0 );
	}
}

void BulletDebugDrawer::PreRender()
{
	m_data.clear();
}