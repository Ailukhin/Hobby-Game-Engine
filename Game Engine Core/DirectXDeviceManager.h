#ifndef DIRECT_X_DEVICE_MANAGER_H
#define DIRECT_X_DEVICE_MANAGER_H

#include <d3d11.h>

#define SafeRelease(x) { if(x){ x->Release(); x = 0; } }

namespace EngineCore
{
	class DirectXDeviceManager
	{
	public:

		//----------------------------------------------------------------------
		// Static Methods
		//----------------------------------------------------------------------

		static void Create(ID3D11Device* pDevice, ID3D11DeviceContext* pContext);
		static void Destroy();

		static ID3D11Device* GetDevice();
		static ID3D11DeviceContext* GetContext();

		~DirectXDeviceManager();

		//----------------------------------------------------------------------
		// Private methods
		//----------------------------------------------------------------------
	private:
		static DirectXDeviceManager* privGetInstance();

		DirectXDeviceManager() = delete;
		DirectXDeviceManager(const DirectXDeviceManager&) = delete;
		DirectXDeviceManager& operator = (const DirectXDeviceManager&) = delete;

		DirectXDeviceManager(ID3D11Device* _pDevice, ID3D11DeviceContext* _pContext);


		//----------------------------------------------------------------------
		// Data: unique data for this manager 
		//----------------------------------------------------------------------
	private:
		static DirectXDeviceManager* posInstance;
		ID3D11Device* pDevice;
		ID3D11DeviceContext* pContext;

	};
}

#endif
