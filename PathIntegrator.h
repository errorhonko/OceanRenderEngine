#pragma once
#include "Integrator.h"
#include "UniformLightSampler.h"

class PathIntegrator : public RayIntegrator
{
public :
	PathIntegrator(
		int maxDepth,
		const Hittable& world,
		Camera camera,
		std::shared_ptr<Sampler> sampler,
		std::vector <std::shared_ptr<Light>> lights)
		:RayIntegrator(world, std::move(camera), std::move(sampler)),
		maxDepth(maxDepth),
		lightSampler(std::move(lights))

	{
	}

	Spectrum Li(Ray ray, Sampler& sampler) const override
	{
		float p_b = 1.0f;
		LightSampleContext prevLightCtx{};
		Spectrum L(.0f);
		Spectrum beta(1.0f);
		bool specularBounce = true;
		int depth = 0;
		float etaScale = 1.0f;
		while (!beta.IsBlack())
		{
			HitRecord rec;
			if (!Intersect(ray, rec))
			{
				
				for (const auto& light : lightSampler.Lights())
				{
					if (!light ||
						light->Type() != LightType::Infinite)
					{
						continue;
					}
					Spectrum Le = light->Le(ray);
					if (Le.IsBlack())
						continue;
					if (depth == 0 || specularBounce)
						L = L + beta * Le;
					else
					{
						float p_l = lightSampler.PMF(prevLightCtx, light)*light->PDF_Li(prevLightCtx,ray.dir);

						float w_b = BRDFUtils::PowerHeuristic(1, p_b, 1, p_l);

						L = L + beta * Le*w_b;
					}
				}
				break;
			}

			Vector3f wo = -ray.dir;
			if ( rec.areaLight)
			{
				Spectrum Le = rec.areaLight->L(
					rec.point, rec.geometricNormal, wo);
				if (!Le.IsBlack())
				{
					if (depth == 0 || specularBounce)
					{
						L = L + beta * Le;

					}
					else
					{
						float p_l =
							lightSampler.PMF(
								prevLightCtx,
								rec.areaLight) *
							rec.areaLight->PDF_Li(
								prevLightCtx,
								ray.dir);

						float w_b =
							BRDFUtils::PowerHeuristic(
								1, p_b,
								1, p_l);

						L = L + beta * Le * w_b;
					}
				}
			}
			if (depth++ == maxDepth)
				break;
			if (!rec.material)
				break;
			MaterialEvalContext ctx{
				rec.point,
				wo,
				rec.normal,
				rec.dpdu,
				rec.u,
				rec.v
			};
			BSDF bsdf = rec.material->GetBSDF(ctx);
			LightSampleContext lightCtx{
				rec.point,
				rec.geometricNormal,
				rec.normal
			};
			if (IsNonSpecular(bsdf.Flags()))
			L = L + beta * SampleLd(
				lightCtx,
				bsdf,
				wo,
				sampler);
			
			auto bs = bsdf.Sample_f(
				wo, sampler.Get2D(),
				TransportMode::Radiance
			);
			if (!bs)
				break;
			float cosTheta = std::abs(bs->wi.dot(rec.normal));
			if (cosTheta == .0f || bs->pdf <= .0f)
			{
				break;
			}
			prevLightCtx = lightCtx;
			p_b = bs->pdf;
			if (bs->pdfIsProportional)
			{
				p_b = bsdf.Pdf(
					wo,
					bs->wi,
					TransportMode::Radiance
				);
			}
			beta = beta * bs->f * (cosTheta / bs->pdf);
			specularBounce =
				(bs->flags & BSDF_SPECULAR) != 0;

			ray = Ray(OffsetRayOrigin(rec.point, rec.geometricNormal, bs->wi),
				bs->wi);


			if ((bs->flags & BSDF_TRANSMISSION) != 0)
				etaScale *= Sqr(bs->eta);
			Spectrum rrBeta = beta * etaScale;

			if (rrBeta.MaxComponentValue() < 1.0f && depth > 1)
			{
				float q = std::max(.0f, 
					1.0f - rrBeta.MaxComponentValue());
				if (sampler.Get1D() < q)
					break;
				beta = beta / (1.0f - q);
			}

		}
		return L;
	}
private:
	Spectrum SampleLd(
		const LightSampleContext& ctx,
		const BSDF& bsdf,
		const Vector3f& wo,
		Sampler& sampler
	) const
	{
		auto sampledLight = lightSampler.Sample(ctx, sampler.Get1D());

		if (!sampledLight || !sampledLight->light || sampledLight->p <= .0f)
			return Spectrum{ .0f };

		auto lightSample = sampledLight->light->SampleLi(ctx, sampler.Get2D());
		if (!lightSample ||
			lightSample->pdf <= 0.0f ||
			lightSample->L.IsBlack())
		{
			return Spectrum(0.0f);
		}
		Spectrum f = bsdf.f(
			wo,
			lightSample->wi,
			TransportMode::Radiance
		);
		float cosTheta =
			std::abs(ctx.ns.dot(lightSample->wi));
		f = f * cosTheta;

		if (f.IsBlack())
			return Spectrum(0.0f);
		if (!Unoccluded(ctx.p, ctx.n, lightSample->wi,
			lightSample->distance))
			return Spectrum(0.0f);

		float p_l = sampledLight->p * lightSample->pdf;
		if (p_l <= .0f || !std::isfinite(p_l))
		{
			return Spectrum(0.0f);
		}
		float w_l = 1.0f;

		if (!IsDeltaLight(sampledLight->light->Type()))
		{
			float p_b = bsdf.Pdf(wo, lightSample->wi,
				TransportMode::Radiance,
				BxDFReflectionType::All);
			if (p_b < .0f || !std::isfinite(p_b))
			{
				return Spectrum(0.0f);
			}
			w_l = BRDFUtils::PowerHeuristic(1,p_l,1,p_b);
		}
		return f * lightSample->L * (w_l / p_l);

	}
	int maxDepth;
	UniformLightSampler lightSampler;
};