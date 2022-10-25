//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

using namespace dae;

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
	//SDL_SetRelativeMouseMode(SDL_TRUE);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	float fov{ tanf(camera.fovAngle / 2.f) }; //Will be const (move later)
	float aspectRatio{ (float)m_Width / (float)m_Height }; //Wil be const (move later)

	Matrix cameraWorld = camera.CalculateCameraToWorld();

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			Vector3 rayDirection
			{
				((((2*(px+0.5f)/m_Width))-1)*aspectRatio*fov),		//X
				((1 - ((2*(py+0.5f))/ m_Height))*fov),				//Y
				1.f													//Z
			};
			rayDirection.Normalize();
			rayDirection = cameraWorld.TransformVector(rayDirection);

			Ray hitRay({ camera.origin }, rayDirection);


			ColorRGB finalColor{};
			HitRecord closestHit{};

			pScene->GetClosestHit(hitRay, closestHit);
			
			if (closestHit.didHit)
			{
				closestHit.origin = closestHit.origin + (closestHit.normal * 0.0001f);
				for (Light light : lights)
				{
					Vector3 lightDirection = LightUtils::GetDirectionToLight(light, closestHit.origin);
					float magnitude = lightDirection.Normalize();
					Ray shadowRay{ closestHit.origin, lightDirection };
					shadowRay.max = magnitude;
					
					if (pScene->DoesHit(shadowRay) && m_ShadowsEnabled)
					{
						//finalColor = finalColor * 0.95f;
					}
					else
					{
						float observedArea{ Vector3::Dot(closestHit.normal, lightDirection) };
						switch (m_CurrentLightingMode)
						{
						case dae::Renderer::LightingMode::ObservedArea:
							
							if (observedArea >= 0.f)
							{
								finalColor += ColorRGB{ 1.f,1.f,1.f } *observedArea;
							}
							break;
						case dae::Renderer::LightingMode::Radiance:
							finalColor += LightUtils::GetRadiance(light, closestHit.origin);
							break;
						case dae::Renderer::LightingMode::BRDF:
							finalColor += materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, -rayDirection);
							break;
						case dae::Renderer::LightingMode::Combined:
						{
							ColorRGB areaColor{};
							if (observedArea >= 0.f)
							{
								areaColor += ColorRGB{ 1.f,1.f,1.f } *observedArea;
							}
							finalColor += (LightUtils::GetRadiance(light, closestHit.origin)) * (areaColor) * (materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, -rayDirection));
							break;
						}
						default:
							break;
						}
					}
				}
			}

			//Update Color in Buffer
			finalColor.MaxToOne();													//Chonks lots of cpu time???

			m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

void Renderer::CycleLightingMode()
{
	m_CurrentLightingMode = (LightingMode)((int)m_CurrentLightingMode + 1);
	if ((int)m_CurrentLightingMode > 3)
	{
		m_CurrentLightingMode = (LightingMode)0;
	}
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}
