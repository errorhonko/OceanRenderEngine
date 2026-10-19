"""Independent diagnostic, stdlib only. Decimal reference is NOT production code."""
import argparse
from decimal import Decimal as Dec, localcontext
import json
import math
from pathlib import Path
import struct

PI = Dec('3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679')
def exact(x):
    return Dec.from_float(float(x))  # recover the exact exported IEEE value, not a short decimal approximation
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def norm(a): return [x/dot(a,a).sqrt() for x in a]
def vec(a): return [exact(x) for x in a]
def plus(a,b): return [x+y for x,y in zip(a,b)]
def scale(a,s): return [x*s for x in a]
def rec(wo,wi,eta):
    m=norm(plus(wo,scale(wi,eta)))
    return scale(m,Dec(-1)) if m[2]<0 else m
def fresnel(c):
    c=abs(c);q=1-Dec('2.25')*(1-c*c)
    if q<=0:return Dec(1),Dec(0),q
    ct=q.sqrt();ni=Dec('1.5');nt=Dec(1)
    rp=(nt*c-ni*ct)/(nt*c+ni*ct);rs=(ni*c-nt*ct)/(ni*c+nt*ct)
    # Stable power transmittance, avoids subtracting a rounded R close to 1.
    t=2*ni*nt*c*ct*(1/(nt*c+ni*ct)**2+1/(ni*c+nt*ct)**2)
    return (rp*rp+rs*rs)/2,t,q
def refract(wo,m,eta):
    ci=dot(wo,m);normal=m
    if ci<0:ci=-ci;normal=scale(normal,Dec(-1))
    q=1-(1-ci*ci)/(eta*eta)
    if q<=0:return None
    return plus(scale(wo,-1/eta),scale(normal,ci/eta-q.sqrt()))
def bsdf(wo,wi,m,eta,ax,ay):
    _,t,_=fresnel(dot(wo,m))
    d=1/(PI*ax*ay*(m[0]*m[0]/(ax*ax)+m[1]*m[1]/(ay*ay)+m[2]*m[2])**2)
    def lam(w):return ((1+(ax*ax*w[0]*w[0]+ay*ay*w[1]*w[1])/(w[2]*w[2])).sqrt()-1)/2
    g=1/(1+lam(wo)+lam(wi));a=dot(wi,m);b=dot(wo,m)
    return d*g*t*abs(a*b/(wo[2]*wi[2]*(a+b/eta)**2))/(eta*eta)
def rel(a,b):return abs(a-b)/max(abs(a),abs(b),1e-300)
def f32(x):return struct.unpack('f',struct.pack('f',x))[0]
def dnorm(v):
    l=math.hypot(*v);return [x/l for x in v]
def ddot(a,b):return sum(x*y for x,y in zip(a,b))
def drec(wo,wi,eta):
    m=dnorm([x+eta*y for x,y in zip(wo,wi)])
    return [-x for x in m] if m[2]<0 else m
def drefract(wo,m,eta):
    c=ddot(wo,m);n=m
    if c<0:c=-c;n=[-x for x in n]
    q=1-(1-c*c)/(eta*eta)
    if q<=0:return None
    return [-x/eta+(c/eta-math.sqrt(q))*y for x,y in zip(wo,n)]
def dbsdf(wo,wi,m,eta,ax,ay):
    c=abs(ddot(wo,m));q=1-2.25*(1-c*c)
    if q<=0:return 0.
    ct=math.sqrt(q);ni=1.5
    rp=(c-ni*ct)/(c+ni*ct);rs=(ni*c-ct)/(ni*c+ct)
    t=1-(rp*rp+rs*rs)/2
    d=1/(math.pi*ax*ay*(m[0]**2/ax**2+m[1]**2/ay**2+m[2]**2)**2)
    def lam(w):return (math.sqrt(1+(ax*ax*w[0]**2+ay*ay*w[1]**2)/w[2]**2)-1)/2
    g=1/(1+lam(wo)+lam(wi));a=ddot(wi,m);b=ddot(wo,m)
    return d*g*t*abs(a*b/(wo[2]*wi[2]*(a+b/eta)**2))/(eta*eta)

def reference(row,config,precision):
    with localcontext() as ctx:
        ctx.prec=precision
        wo=norm(vec(row['wo']));m=norm(vec(row['wm']));wi=norm(vec(row['wi']));eta=Dec(2)/3
        ax,ay=exact(config['alphaX']),exact(config['alphaY'])
        rebuilt=rec(wo,wi,eta)
        known=bsdf(wo,wi,m,eta,ax,ay);query=bsdf(wo,wi,rebuilt,eta,ax,ay)
        accurateWi=refract(wo,m,eta)
        if accurateWi is None: raise RuntimeError('Reference classified an accepted diagnostic sample as TIR')
        accurateKnown=bsdf(wo,accurateWi,m,eta,ax,ay)
        accurateQuery=bsdf(wo,accurateWi,rec(wo,accurateWi,eta),eta,ax,ay)
        # Telescoping q decomposition for the SAME original float vectors.
        # Separates unit-length residual, reciprocal-index rounding, dot-product
        # rounding, then remaining float Snell arithmetic. Terms can cancel.
        rawWo,rawM=vec(row['wo']),vec(row['wm'])
        rawC=abs(dot(rawWo,rawM));unitC=abs(dot(wo,m));floatC=abs(exact(row['cosSampleFloat']))
        etaF=exact(row['etaEffectiveFloat'])
        qStages=[1-(1-unitC*unitC)/(eta*eta),1-(1-rawC*rawC)/(eta*eta),
                 1-(1-rawC*rawC)/(etaF*etaF),1-(1-floatC*floatC)/(etaF*etaF),exact(row['qSample1'])]
        qTerms=[qStages[i+1]-qStages[i] for i in range(4)]
        if abs(sum(qTerms)-(qStages[-1]-qStages[0]))>Dec('1e-65'):raise RuntimeError('q decomposition failed to close')
        recEtaF=rec(wo,wi,etaF)
        floatRecNormalized=norm(vec(row['wmReconstructed']))
        return {'known':known,'query':query,'accurateKnown':accurateKnown,'accurateQuery':accurateQuery,
                'qKnown':fresnel(dot(wo,m))[2],'qQuery':fresnel(dot(wo,rebuilt))[2],
                'cosKnown':dot(wo,m),'cosQuery':dot(wo,rebuilt),
                'qNormalizedExactEta':qStages[0],'qRawVectorsExactEta':qStages[1],
                'qRawVectorsFloatEta':qStages[2],'qFloatDotFloatEta':qStages[3],'qLegacyRefractFloat':qStages[4],
                'qTermUnitLengthResidual':qTerms[0],'qTermReciprocalEtaRounding':qTerms[1],
                'qTermDotRounding':qTerms[2],'qTermRemainingSnellArithmetic':qTerms[3],
                'cosQueryDoubleRebuildFloatEta':dot(wo,recEtaF),
                'cosQueryFloatRebuildNormalized':dot(wo,floatRecNormalized),
                'woRawLengthSquared':dot(rawWo,rawWo),'wmRawLengthSquared':dot(rawM,rawM)}

def analyze(raw,output):
    rows=[json.loads(x) for x in raw.read_text(encoding='utf-8').splitlines()]
    config=next(r for r in rows if r['kind']=='config');summary=next(r for r in rows if r['kind']=='summary')
    samples=[r for r in rows if r['kind']=='sample'];result=[]
    assert summary['critical']==12 and summary['maximumFloatReplayError']<1e-5
    convergence=0.;floatFresnelErrors=[0.,0.,0.];tirMismatches=[0,0,0];maxDoublePair=0.
    for row in samples:
        ref=reference(row,config,80)
        # 80 vs 120 decimal digits on every critical sample, not just the worst.
        if row['cosTransFloat']<.02:
            finer=reference(row,config,120)
            for key in ('known','query','accurateKnown','accurateQuery'):
                with localcontext() as ctx:
                    ctx.prec=120
                    convergence=max(convergence,float(abs(ref[key]-finer[key])/max(abs(finer[key]),Dec('1e-200'))))
        wo=dnorm(row['wo']);m=dnorm(row['wm']);wi=dnorm(row['wi']);eta=1/1.5
        ax,ay=config['alphaX'],config['alphaY']
        def pair(direction,rebuildEta=eta):
            if direction is None:return None
            a=dbsdf(wo,direction,m,eta,ax,ay);b=dbsdf(wo,direction,drec(wo,direction,rebuildEta),eta,ax,ay)
            return {'sample':a,'query':b,'relative':rel(a,b)}
        regenerated=drefract(wo,m,eta)
        if regenerated is None:raise RuntimeError('double refraction unexpectedly TIR')
        stages={
            'double_on_stored_float_directions':pair(wi),
            'double_rebuild_with_float_reciprocal_eta':pair(wi,row['etaEffectiveFloat']),
            'double_regenerate_no_direction_cast':pair(regenerated),
            'double_regenerate_then_cast_wi_float':pair(dnorm([f32(x) for x in regenerated])),
            'double_refract_using_float_reciprocal_eta':pair(drefract(wo,m,row['etaEffectiveFloat']))}
        stages['double_refract_raw_float_vectors_float_eta']=pair(
            dnorm(drefract(row['wo'],row['wm'],row['etaEffectiveFloat'])))
        stages['double_refract_raw_float_vectors_exact_eta']=pair(
            dnorm(drefract(row['wo'],row['wm'],eta)))
        maxDoublePair=max(maxDoublePair,stages['double_regenerate_no_direction_cast']['relative'])
        scalar=[]
        for side,ckey,rkey in (('sample','cosSampleFloat','RSample'),('query','cosQueryFloat','RQuery')):
            with localcontext() as ctx:
                ctx.prec=80
                R,T,Q=fresnel(exact(row[ckey]))
            entry={'side':side,'cosFloat':row[ckey],'referenceQ':float(Q),'referenceT':float(T),
                   'floatT':[1-row[rkey+str(k)] for k in range(3)]}
            for k in range(3):
                if Q>0:floatFresnelErrors[k]=max(floatFresnelErrors[k],rel(1-row[rkey+str(k)],float(T)))
                if (row['q'+('Sample' if side=='sample' else 'Query')+str(k)]<=0)!=(Q<=0):tirMismatches[k]+=1
            scalar.append(entry)
        t0=1-row['RSample0'];t1=1-row['RQuery0']
        result.append({'index':row['index'],'critical':row['cosTransFloat']<.02,
            'currentSample':row['fSample'],'currentQuery':row['fQuery'],'currentRelative':rel(row['fSample'],row['fQuery']),
            'squaredFloatRelative':rel(row['fSquaredSample'],row['fSquaredQuery']),
            'fmaFloatRelative':rel(row['fFmaSample'],row['fFmaQuery']),
            'legacyTransmissionRatio_query_over_sample':t1/t0,
            'otherFactorsRatio_query_over_sample':(row['fQuery']/t1)/(row['fSample']/t0),
            'referenceFixedDirections':{k:str(v) for k,v in ref.items()},
            'referenceFixedDirectionsPairRelative':rel(float(ref['known']),float(ref['query'])),
            'sampleRelativeToNormalizedStoredDirectionReference':rel(row['fSample'],float(ref['query'])),
            'queryRelativeToNormalizedStoredDirectionReference':rel(row['fQuery'],float(ref['query'])),
            'doubleStages':stages,'scalarFresnel':scalar})
    sweep=[]
    for r in (r for r in rows if r['kind']=='criticalSweep'):
        with localcontext() as ctx:
            ctx.prec=80
            _,td,qd=fresnel(exact(r['cosDouble']));_,tf,qf=fresnel(exact(r['cosFloat']))
        sweep.append({**r,'referenceQ_at_float_cos':float(qf),'referenceQ_at_double_cos':float(qd),
                      'referenceT_at_float_cos':float(tf),'referenceT_at_double_cos':float(td),
                      'input_quantization_changes_TIR':(qf<=0)!=(qd<=0),
                      'fixed_float_input_formula_TIR_mismatches':[(r['q'+str(k)]<=0)!=(qf<=0) for k in range(3)]})
    # New production outputs, verified against an independent Cartesian GGX /
    # Smith / Fresnel Decimal reference for the SAME stored float direction pair.
    # Do not compare new directions against the old-direction reference.
    mixed=[]
    for row in samples:
        if 'mixedWi' not in row:continue
        refs=[]
        for precision in (80,120):
            with localcontext() as ctx:
                ctx.prec=precision
                wo=norm(vec(row['wo']));wi=norm(vec(row['mixedWi']));eta=Dec(2)/3
                m=rec(wo,wi,eta)
                refs.append(bsdf(wo,wi,m,eta,exact(config['alphaX']),exact(config['alphaY'])))
        with localcontext() as ctx:
            ctx.prec=120
            stable=float(abs(refs[0]-refs[1])/max(abs(refs[1]),Dec('1e-200')))
        entry={'index':row['index'],'reference':str(refs[1]),'referenceConvergence':stable,
               'fSample':row['mixedFSample'],'fQuery':row['mixedFQuery'],
               'pairRelative':rel(row['mixedFSample'],row['mixedFQuery']),
               'queryRelativeToReference':rel(row['mixedFQuery'],float(refs[1]))}
        if stable>1e-60 or entry['queryRelativeToReference']>1e-5:
            raise RuntimeError('Mixed precision independent reference verification failed')
        mixed.append(entry)
    if summary['maxMixedFRelative']>1e-5 or summary['maxMixedPdfRelative']>1e-5:
        raise RuntimeError('Mixed precision sample/query regression')
    worst=next(r for r in result if r['index']==summary['worstIndex'])
    report={'config':config,'replay':summary,'selectedSampleCount':len(result),
            'reference80_vs_120_maxRelative':convergence,'regeneratedDouble_maxPairRelative':maxDoublePair,
            'selectedFixedCosine_maxTransmissionRelativeErrors_legacy_squared_fma':floatFresnelErrors,
            'selectedFixedCosine_TIR_mismatches_legacy_squared_fma':tirMismatches,
            'sweepInputQuantizationTIRChanges':sum(r['input_quantization_changes_TIR'] for r in sweep),
            'sweepFixedFloatInputFormulaTIRMismatches':[sum(r['fixed_float_input_formula_TIR_mismatches'][k] for r in sweep) for k in range(3)],
            'worst':worst,'samples':result,'criticalSweep':sweep,
            'mixedPrecision':{'accepted':summary['mixedAccepted'],
                'acceptanceChanges':summary['acceptanceChanges'],
                'allAcceptedMaxPairRelative':summary['maxMixedFRelative'],
                'allAcceptedMaxPdfRelative':summary['maxMixedPdfRelative'],
                'selectedMaxQueryRelativeToIndependentReference':max(r['queryRelativeToReference'] for r in mixed),
                'selectedMaxReference80vs120':max(r['referenceConvergence'] for r in mixed),
                'worstLegacyIndexAfter':next(r for r in mixed if r['index']==summary['worstIndex']),
                'selectedSamples':mixed}}
    critical_results=[r for r in result if r['critical']]
    ordinary_results=[r for r in result if not r['critical']]
    report['criticalMaxPairRelativeByDoubleStage']={
        name:max(r['doubleStages'][name]['relative'] for r in critical_results)
        for name in worst['doubleStages']}
    report['selectedOrdinaryMaxLegacyPairRelative']=max(r['currentRelative'] for r in ordinary_results)
    if convergence>1e-60 or maxDoublePair>1e-7:raise RuntimeError('Reference/roundtrip verification failed')
    output.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    compact={k:v for k,v in report.items() if k not in ('samples','criticalSweep')}
    print(json.dumps(compact,indent=2,ensure_ascii=False,allow_nan=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('raw',type=Path);p.add_argument('output',type=Path);args=p.parse_args()
    if args.output.exists():raise SystemExit('Refusing to overwrite an existing analysis.')
    analyze(args.raw,args.output)
