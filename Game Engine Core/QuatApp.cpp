#include "MathEngine.h"
#include "MathApp.h"

//----------------------------------------------------------------------------- 
// Mutates result to be a slerp between the source and target by the
// specified factor.
// For a factor of 0.0, result == source.
// For a factor of 1.0, result == target.
//----------------------------------------------------------------------------- 

namespace EngineCore
{
	void QuatApp::Slerp(Quat& result, const Quat& source, const Quat& target, const float slerpFactor)
	{
		float angle = source.dot(target);
		float tmpCos = angle;

		Quat t = target;

		if (angle < 0.0f)
		{
			tmpCos = -angle;
			t = -t;
		}

		angle = Trig::acos(tmpCos);

		result = source * (Trig::sin((1 - slerpFactor) * angle) / Trig::sin(angle)) + t * (Trig::sin(slerpFactor * angle) / Trig::sin(angle));

		if (source.isIdentity() && target.isIdentity())
		{
			result.set(Special::Identity);
		}
	}

	void QuatApp::SlerpArray(Quat* result, const Quat* source, const Quat* target, const float slerpFactor, const int numQuats)
	{
		for (int i = 0; i < numQuats; i++)
		{
			float angle = source[i].dot(target[i]);
			float tmpCos = angle;

			Quat t = target[i];

			if (angle < 0.0f)
			{
				tmpCos = -angle;
				t = -t;
			}

			angle = Trig::acos(tmpCos);

			result[i] = source[i] * (Trig::sin((1 - slerpFactor) * angle) / Trig::sin(angle)) + t * (Trig::sin(slerpFactor * angle) / Trig::sin(angle));

			if (source[i].isIdentity() && target[i].isIdentity())
			{
				result[i].set(Special::Identity);
			}
		}
	}
}