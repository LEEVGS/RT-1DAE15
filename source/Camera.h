#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>
#include <iostream>

#include "Math.h"
#include "Timer.h"

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle):
			origin{_origin},
			fovAngle{_fovAngle}
		{
		}


		Vector3 origin{};
		float fovAngle{90.f};

		Vector3 forward{Vector3::UnitZ};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		float totalPitch{0.f};
		float totalYaw{0.f};

		Matrix cameraToWorld{};


		Matrix CalculateCameraToWorld()
		{
			right = Vector3::Cross(Vector3::UnitY, forward);
			up = Vector3::Cross(forward, right);
			Matrix data{};
			data[0] = {right.x, right.y, right.z, 0.f};
			data[1] = {up.x, up.y, up.z, 0.f};
			data[2] = {forward.x, forward.y, forward.z, 0.f};
			data[3] = {origin.x, origin.y, origin.z, 1.f};
			return data;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();
			const float cameraSpeed{ 10.f };

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);


			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY); //1 left 4 right
			
			if (mouseState == SDL_BUTTON_LEFT)
			{
				origin.z -= mouseY * deltaTime;
			}
			if (mouseState == SDL_BUTTON_RIGHT)
			{
				origin.y -= mouseY * deltaTime;
			}

			if (pKeyboardState[SDL_SCANCODE_W] || pKeyboardState[SDL_SCANCODE_UP])
			{
				origin += cameraSpeed * deltaTime * forward;
			}
			if (pKeyboardState[SDL_SCANCODE_S] || pKeyboardState[SDL_SCANCODE_DOWN])
			{
				origin -= cameraSpeed * deltaTime * forward;
			}
			if (pKeyboardState[SDL_SCANCODE_A] || pKeyboardState[SDL_SCANCODE_LEFT])
			{
				origin -= cameraSpeed * deltaTime * right;
			}
			if (pKeyboardState[SDL_SCANCODE_D] || pKeyboardState[SDL_SCANCODE_RIGHT])
			{
				origin += cameraSpeed * deltaTime * right;
			}
			if (mouseState == 4 || mouseState == 1)
			{
				totalPitch -= cameraSpeed * TO_RADIANS * mouseY * deltaTime;
				totalYaw -= cameraSpeed * TO_RADIANS * mouseX * deltaTime;
			}

			forward = Matrix::CreateRotation(totalPitch, totalYaw, 0.f).TransformVector(Vector3::UnitZ);
			forward.Normalize();
		}
	};
}
