#pragma once
#include "Integrator.h"
#include "UniformLightSampler.h"
class SimplePathIntegrator
	:public RayIntegrator
{
public:
    SimplePathIntegrator(
        int maxDepth,
        bool sampleLights,
        bool sampleBSDF,
        const Hittable& world,
        Camera camera,
        std::shared_ptr<Sampler> sampler,
        std::vector<std::shared_ptr<Light>> lights)
        : RayIntegrator(
            world,
            std::move(camera),
            std::move(sampler)),
        maxDepth(maxDepth),
        sampleLights(sampleLights),
        sampleBSDF(sampleBSDF),
        lightSampler(std::move(lights))
    {
    }

    Spectrum Li(
        Ray ray,
        Sampler& sampler) const override
    {
        Spectrum L(.0f);
		Spectrum beta(1.0f);
        bool specularBounce = true;
		int depth = 0;
        while (!beta.IsBlack())
        {
            HitRecord rec;
            if (!Intersect(ray, rec))
            {
                if (!sampleLights || specularBounce)
                {
					for (const auto& light : lightSampler.Lights())
					{
                        if(light)
						L = L + beta * light->Le(ray);
					}
                }
				break;
            }

            Vector3f wo = -ray.dir;
			if ((!sampleLights || specularBounce) && rec.areaLight)
			{
				L = L + beta * rec.areaLight->L(
					rec.point, rec.geometricNormal, wo);
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
			BSDF bsdf=rec.material->GetBSDF(ctx);
            if (sampleLights)
            {
				LightSampleContext lightCtx{
					rec.point,
					rec.geometricNormal,
					rec.normal
				};

				L = L + beta * SampleLd(lightCtx, bsdf, wo, sampler);
            }
            if (!sampleBSDF)
            {
                break;
            }
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
            beta = beta * bs->f * (cosTheta / bs->pdf);
            specularBounce =
                (bs->flags & BSDF_SPECULAR) != 0;

			ray = Ray(OffsetRayOrigin(rec.point, rec.geometricNormal, bs->wi)  ,
                 bs->wi);
        }
		return L;
    };

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
			return Spectrum{.0f};

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
        return f * lightSample->L /
            (sampledLight->p * lightSample->pdf);

    }
    int maxDepth;
    bool sampleLights;
    bool sampleBSDF;

    UniformLightSampler lightSampler;
};
