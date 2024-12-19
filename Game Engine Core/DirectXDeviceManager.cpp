#include "DirectXDeviceManager.h"
#include <cassert>

namespace EngineCore
{
	DirectXDeviceManager* DirectXDeviceManager::posInstance = nullptr;

	//----------------------------------------------------------------------
	// Static Methods
	//----------------------------------------------------------------------
	void DirectXDeviceManager::Create(ID3D11Device* pDevice, ID3D11DeviceContext* pContext)
	{
		// make sure values are ressonable 
		assert(pDevice);
		assert(pContext);

		// intialize the singleton here
		assert(posInstance == nullptr);

		// Do the initialization
		if (posInstance == nullptr)
		{
			posInstance = new DirectXDeviceManager(pDevice, pContext);
		}
	}

	ID3D11Device* DirectXDeviceManager::GetDevice()
	{
		DirectXDeviceManager* pMan = DirectXDeviceManager::privGetInstance();
		assert(pMan);

		return pMan->pDevice;
	}

	ID3D11DeviceContext* DirectXDeviceManager::GetContext()
	{
		DirectXDeviceManager* pMan = DirectXDeviceManager::privGetInstance();
		assert(pMan);

		return pMan->pContext;
	}

	void DirectXDeviceManager::Destroy()
	{
		delete posInstance;
	}

	DirectXDeviceManager::DirectXDeviceManager(ID3D11Device* _pDevice, ID3D11DeviceContext* _pContext)
	{
		this->pDevice = _pDevice;
		this->pContext = _pContext;
	}

	DirectXDeviceManager::~DirectXDeviceManager()
	{

	}

	//----------------------------------------------------------------------
	// Private methods
	//----------------------------------------------------------------------
	DirectXDeviceManager* DirectXDeviceManager::privGetInstance()
	{
		// Safety - this forces users to call Create() first before using class
		assert(posInstance != nullptr);

		return posInstance;
	}
}