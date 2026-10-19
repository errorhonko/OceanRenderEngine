// Diagnostic only: never linked into the renderer or acceptance executable.
#include "reflection/layered/CoatedDiffuseReflection.h"
#include "LegacyFloatReference.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace {
struct FresnelTrace { float reflection,q; };
FresnelTrace Trace(float c,int variant)
{
    c=std::clamp(c,-1.f,1.f);
    float eta=1.5f;if(c<0){eta=1/eta;c=-c;}
    float q;
    if(variant==0)
    {
        const float si=std::sqrt(std::max(0.f,1-c*c));
        const float st=(eta<1?1.5f:1.f/1.5f)*si;
        q=1-st*st;
        return {CriticalLegacy::Fresnel(-c),q}; // This study's incidence is from inside.
    }
    if(variant==1)q=1-(1-c*c)/(eta*eta); // PBRT squared-sine form.
    else
    {
        const float sin2=std::fma(-c,c,1.f);
        const float ratio=1/eta;
        q=std::fma(-(ratio*ratio),sin2,1.f); // Proposed FMA ablation, NOT PBRT code.
    }
    if(q<=0)return {1,q};
    const float ct=std::sqrt(q);
    const float rp=(eta*c-ct)/(eta*c+ct),rs=(c-eta*ct)/(c+eta*ct);
    return {(rp*rp+rs*rs)/2,q};
}
float EvaluateFloat(const TrowbridgeReitzDistribution& distribution,Vector3f wo,Vector3f wi,Vector3f wm,
                    float etaEffective,int variant,bool sampleOrder)
{
    const float T=1-Trace(wo.dot(wm),variant).reflection;
    const float d=distribution.D(wm),g=distribution.G(wo,wi),a=wi.dot(wm),b=wo.dot(wm);
    float denom=BRDFUtils::Sqr(a+b/etaEffective),value;
    if(sampleOrder)value=T*d*g*std::abs(a*b/(wi.z*wo.z*denom));
    else {denom=denom*wi.z*wo.z;value=d*g*T*std::abs(a*b/denom);}
    return value/(etaEffective*etaEffective);
}
double Relative(double a,double b){return std::abs(a-b)/std::max({std::abs(a),std::abs(b),1e-30});}
void Vector(std::ostream& out,const Vector3f& v){out<<'['<<v.x<<','<<v.y<<','<<v.z<<']';}
void Field(std::ostream& out,const char* key,double value){out<<",\""<<key<<"\":"<<value;}
}

int wmain(int argc,wchar_t** argv)
{
    try
    {
        if(argc!=2)throw std::runtime_error("Pass a fresh output JSONL path.");
        const std::filesystem::path path(argv[1]);
        if(std::filesystem::exists(path))throw std::runtime_error("Refusing to overwrite an existing diagnostic.");
        std::ofstream out(path);if(!out)throw std::runtime_error("Cannot open diagnostic output.");
        out<<std::setprecision(17);
        const Vector3f wo=MicrofacetReflectionDetail::NormalizeDirection({.8f,.1f,-.6f});
        const TrowbridgeReitzDistribution distribution(.6f,.85f);
        const DielectricBxDF bxdf(1.5f,distribution);
        LayeredReflectionDetail::RandomStream rng(42);
        constexpr std::size_t count=131072;
        std::size_t accepted=0,critical=0,exported=0,worstIndex=0;
        double worst=0,maximumReplayError=0,worstSquared=0,worstFma=0;
        double maxMixedF=0,maxMixedPdf=0;
        std::size_t mixedAccepted=0,acceptanceChanges=0;
        out<<"{\"kind\":\"config\",\"trials\":"<<count<<",\"seed\":42,\"eta\":1.5,\"alphaX\":"<<double(.6f)<<",\"alphaY\":"<<double(.85f)<<",\"wo\":";
        Vector(out,wo);out<<"}\n";
        for(std::size_t i=0;i<count;++i)
        {
            const auto u=rng.Next2D();
            const auto s=CriticalLegacy::Sample(distribution,wo,u);
            const auto mixed=bxdf.Sample_f(wo,u,TransportMode::Radiance,BxDFReflectionType::Transmission);
            if(bool(s)!=bool(mixed))++acceptanceChanges;
            if(mixed) {
                ++mixedAccepted;
                const double fq=bxdf.f(wo,mixed->wi,TransportMode::Radiance).c[0];
                maxMixedF=std::max(maxMixedF,Relative(mixed->f.c[0],fq));
                maxMixedPdf=std::max(maxMixedPdf,Relative(mixed->pdf,bxdf.Pdf(wo,mixed->wi,TransportMode::Radiance,BxDFReflectionType::Transmission)));
            }
            if(!s)continue;++accepted;
            const Vector3f wm=distribution.Sample_Wm(wo,u);
            Vector3f reproduced;float etaEffective;
            if(!CriticalLegacy::Refract(wo,wm,&etaEffective,&reproduced))throw std::runtime_error("Accepted sample did not replay.");
            if(reproduced.x!=s->wi.x||reproduced.y!=s->wi.y||reproduced.z!=s->wi.z)
                throw std::runtime_error("VNDF/refraction replay direction mismatch.");
            const auto rec=BRDFUtils::FaceForward((s->wi*etaEffective+wo).normalize(),{0,0,1});
            const double f0=s->f.c[0],f1=EvaluateFloat(distribution,wo,s->wi,rec,etaEffective,0,false);
            const double error=Relative(f0,f1);
            const double ct=std::abs(s->wi.dot(rec));
            const bool isCritical=ct<.02;if(isCritical)++critical;
            maximumReplayError=std::max(maximumReplayError,Relative(f0,EvaluateFloat(distribution,wo,s->wi,wm,etaEffective,0,true)));
            const double sq0=EvaluateFloat(distribution,wo,s->wi,wm,etaEffective,1,true);
            const double sq1=EvaluateFloat(distribution,wo,s->wi,rec,etaEffective,1,false);
            const double fm0=EvaluateFloat(distribution,wo,s->wi,wm,etaEffective,2,true);
            const double fm1=EvaluateFloat(distribution,wo,s->wi,rec,etaEffective,2,false);
            if(error>worst){worst=error;worstIndex=i;}
            worstSquared=std::max(worstSquared,Relative(sq0,sq1));worstFma=std::max(worstFma,Relative(fm0,fm1));
            // All critical cases, all noticeable discrepancies, ordinary controls.
            // This selection is DIAGNOSTIC, not a fresh Monte Carlo energy estimator.
            if(!isCritical&&error<.0001&&i%2048!=0)continue;
            ++exported;
            out<<"{\"kind\":\"sample\",\"index\":"<<i<<",\"u\":["<<u.x<<','<<u.y<<"],\"wo\":";Vector(out,wo);
            out<<",\"wm\":";Vector(out,wm);out<<",\"wi\":";Vector(out,s->wi);out<<",\"wmReconstructed\":";Vector(out,rec);
            Field(out,"etaEffectiveFloat",etaEffective);Field(out,"fSample",f0);Field(out,"fQuery",f1);Field(out,"pdf",s->pdf);
            Field(out,"fSquaredSample",sq0);Field(out,"fSquaredQuery",sq1);Field(out,"fFmaSample",fm0);Field(out,"fFmaQuery",fm1);
            Field(out,"cosSampleFloat",wo.dot(wm));Field(out,"cosQueryFloat",wo.dot(rec));Field(out,"cosTransFloat",ct);
            for(int k=0;k<3;++k)
            {
                const std::string suffix=std::to_string(k);
                Field(out,("qSample"+suffix).c_str(),Trace(wo.dot(wm),k).q);
                Field(out,("qQuery"+suffix).c_str(),Trace(wo.dot(rec),k).q);
                Field(out,("RSample"+suffix).c_str(),Trace(wo.dot(wm),k).reflection);
                Field(out,("RQuery"+suffix).c_str(),Trace(wo.dot(rec),k).reflection);
            }
            if(mixed) {
                out<<",\"mixedWi\":";Vector(out,mixed->wi);
                Field(out,"mixedFSample",mixed->f.c[0]);
                Field(out,"mixedFQuery",bxdf.f(wo,mixed->wi,TransportMode::Radiance).c[0]);
            }
            out<<"}\n";
        }
        out<<"{\"kind\":\"summary\",\"accepted\":"<<accepted<<",\"critical\":"<<critical<<",\"exported\":"<<exported<<",\"worstIndex\":"<<worstIndex;
        Field(out,"mixedAccepted",double(mixedAccepted));Field(out,"acceptanceChanges",double(acceptanceChanges));
        Field(out,"maxMixedFRelative",maxMixedF);Field(out,"maxMixedPdfRelative",maxMixedPdf);
        Field(out,"worstLegacyPairRelative",worst);Field(out,"worstSquaredPairRelative",worstSquared);
        Field(out,"worstFmaPairRelative",worstFma);Field(out,"maximumFloatReplayError",maximumReplayError);out<<"}\n";
        // Fixed-cosine critical sweep: compare operations on identical inputs,
        // separately from quantizing the incident cosine to float.
        for(int exponent=2;exponent<=14;++exponent)for(double sign:{1.,-1.})
        {
            const double targetQ=sign*std::pow(10.,-exponent);
            const double ratio=1/1.5,cd=std::sqrt(1-ratio*ratio+ratio*ratio*targetQ);
            const float cf=float(cd);
            out<<"{\"kind\":\"criticalSweep\",\"targetQ\":"<<targetQ<<",\"cosDouble\":"<<cd<<",\"cosFloat\":"<<cf;
            for(int k=0;k<3;++k){const auto t=Trace(-cf,k);Field(out,("q"+std::to_string(k)).c_str(),t.q);Field(out,("R"+std::to_string(k)).c_str(),t.reflection);}
            out<<"}\n";
        }
        if(!out)throw std::runtime_error("Diagnostic output write failed.");
        std::cout<<"accepted="<<accepted<<" critical="<<critical<<" exported="<<exported<<" worstIndex="<<worstIndex<<
            " pairRelative="<<worst<<" replayError="<<maximumReplayError<<'\n';
        if(accepted==0||critical==0||maximumReplayError>1e-5)throw std::runtime_error("Replay verification failed.");
        return 0;
    }
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
