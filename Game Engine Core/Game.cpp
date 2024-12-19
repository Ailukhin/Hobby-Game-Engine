#include <d3d11.h>
#include <d3dcompiler.h>
#include <WinUser.h>
#include "Game.h"
#include "Engine.h"
#include "MathEngine.h"
#include "Colors.h"
#include <cassert>

using namespace EngineCore;

namespace EngineCore
{
	Game::Game(const char* const pName, int width, int height)
		: Engine(pName, width, height)
	{

	}

	Game::~Game()
	{

	}

	bool Game::LoadContent()
	{
		return false;
	}

	void Game::UnloadContent()
	{

	}

	void Game::Update(float deltaTime)
	{

	}

	void Game::Render()
	{
		this->SetDefaultTargetMode();

	}

	void Game::ClearDepthStencilBuffer()
	{
		#ifdef _DEBUG
			const Vec4 ClearColor = Colors::CornflowerBlue;
		#else
			const Vec4 ClearColor = Colors::Wheat;
		#endif

		float clearDepth = 1.0f;
		uint8_t clearStencil = 0;
		g_d3dDeviceContext->ClearRenderTargetView(g_d3dRenderTargetView, (const float*)&ClearColor);
		g_d3dDeviceContext->ClearDepthStencilView(g_d3dDepthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, clearDepth, clearStencil);
	}

	float Game::GetAspectRatio() const
	{
		RECT clientRect;
		GetClientRect(g_WindowHandle, &clientRect);

		// Compute the exact client dimensions.
		// This is required for a correct projection matrix.
		float clientWidth = static_cast<float>(clientRect.right - clientRect.left);
		float clientHeight = static_cast<float>(clientRect.bottom - clientRect.top);

		float ratio = clientWidth / clientHeight;

		return ratio;
	}

	void Game::SetDefaultTargetMode()
	{
		assert(g_d3dDevice);
		assert(g_d3dDeviceContext);
		//--------------------------------------------------------
		// Set (point to ) the Rasterizers functions to be used
		//--------------------------------------------------------
		g_d3dDeviceContext->RSSetState(g_d3dRasterizerState);

		//--------------------------------------------------------
		// Set (point to ) the Viewport to be used
		//--------------------------------------------------------
		g_d3dDeviceContext->RSSetViewports(1, &g_Viewport);

		//--------------------------------------------------------
		// Set (point to ) render target
		//      Only one Target, this maps to Pixel shader
		// --------------------------------------------------------
		g_d3dDeviceContext->OMSetRenderTargets(1, &g_d3dRenderTargetView, g_d3dDepthStencilView);

		// Set (point to) blend target
		float blendFactor[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
		UINT sampleMask = 0xffffffff;
		g_d3dDeviceContext->OMSetBlendState(g_d3dBlendStateOff, blendFactor, sampleMask);

		//--------------------------------------------------------
		// Set (point to ) the Depth functions to be used
		//--------------------------------------------------------
		g_d3dDeviceContext->OMSetDepthStencilState(g_d3dDepthStencilState, 1);
	}
}