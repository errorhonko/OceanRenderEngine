// Experiment-only implementation, included inside main.cpp's anonymous namespace.
// State codes: 0=back-facing/not traced, 1=blocked, 2=clear, 3=censored.
struct PairedSnapshot {
    std::vector<Point> points;
    std::vector<std::array<int,2>> states;
};
V PairedView(int view) {
    const double t=85*pi/180,f=view*90*pi/180;
    return V::From(Ray(Vector3f(),V{std::sin(t)*std::cos(f),std::cos(t),std::sin(t)*std::sin(f)}.Float()).dir).Unit();
}
PairedSnapshot CapturePaired(const Config& c,std::uint32_t seed) {
    const auto mesh=BuildMesh(c,seed);
    std::vector<std::shared_ptr<Hittable>> primitives;
    primitives.reserve(mesh->triangles.size());
    for(std::size_t i=0;i<mesh->triangles.size();++i)
        primitives.push_back(std::make_shared<StudyTriangle>(mesh,i));
    const BVHAccel bvh(std::move(primitives));
    PairedSnapshot result;
    result.points.reserve(c.samples);result.states.reserve(c.samples);
    std::mt19937_64 rng(Hash(std::uint64_t(seed)^0x632BE59BD9B4E019ULL));
    for(int i=0;i<c.samples;++i) {
        const auto p=SamplePoint(*mesh,c,rng);
        std::array<int,2> state{};
        for(int v=0;v<2;++v) {
            const auto w=PairedView(v);
            if(w.Dot(p.normal)<=0) continue;
            const auto visibility=Trace(p,w,*mesh,bvh,c);
            state[v]=visibility==Visibility::Blocked?1:(visibility==Visibility::Clear?2:3);
        }
        result.points.push_back(p);result.states.push_back(state);
    }
    return result; // BVH and mesh released before the other resolution is built.
}
void RunPaired(const Config& c,int otherResolution,const std::filesystem::path& directory) {
    Config other=c;other.resolution=otherResolution;Validate(other);
    if(std::filesystem::exists(directory)) throw std::runtime_error("Choose a new paired output directory.");
    CheckIntersectionKernel();
    std::filesystem::create_directories(directory);
    auto manifest=Output(directory/"parameters.txt");
    manifest<<"OceanVisibilityStudy paired v1; status=started\nN_a="<<c.resolution<<"\nN_b="<<other.resolution
        <<"\nL_m="<<c.length<<"\nobservation_length_m="<<ObservationLength(c)
        <<"\nk_min="<<c.kMin<<"\nk_max_exclusive="<<c.kMax<<"\nsamples="<<c.samples
        <<"\nfirst_seed="<<c.firstSeed<<"\nseeds="<<c.seeds<<"\noffset_world_y_m="<<c.offset
        <<"\nmax_horizontal_distance_m="<<c.maxDistance
        <<"\nU10=10; friction_velocity=.38; inverse_wave_age=.84; wind_direction=0\n"
        <<"gravity=9.81; water_density=1000; surface_tension=.074\n"
        <<"theta=85; azimuth=0,90; normal bins=10deg polar x 30deg azimuth\n"
        <<"state: 0=back-facing/not traced,1=blocked,2=clear,3=censored\n"
        <<"a/b use identical horizontal RNG; native weight=1/normal.y\n"
        <<"reference bins/weights are diagnostic, not exact truth\n";
    manifest.flush();
    auto out=Output(directory/"paired_samples.csv");
    out<<"seed,sample,azimuth_deg,x_a,z_a,x_b,z_b,y_a,y_b,nx_a,ny_a,nz_a,nx_b,ny_b,nz_b,bin_a,bin_b,state_a,state_b,weight_a,weight_b\n";
    for(int s=0;s<c.seeds;++s) {
        const auto seed=c.firstSeed+std::uint32_t(s);
        std::cout<<"Paired seed "<<seed<<" N="<<c.resolution<<"/"<<other.resolution<<std::endl;
        const auto a=CapturePaired(c,seed),b=CapturePaired(other,seed);
        for(int i=0;i<c.samples;++i) {
            const auto& p=a.points[i];const auto& q=b.points[i];
            if(std::abs(p.position.x-q.position.x)>1e-10*c.length ||
                std::abs(p.position.z-q.position.z)>1e-10*c.length)
                throw std::runtime_error("Paired horizontal positions differ.");
            if(c.resolution==other.resolution &&
                (p.position.y!=q.position.y || p.normal.x!=q.normal.x || p.normal.y!=q.normal.y ||
                 p.normal.z!=q.normal.z || a.states[i]!=b.states[i]))
                throw std::runtime_error("Equal-resolution identity check failed.");
            for(int v=0;v<2;++v) {
                out<<seed<<','<<i<<','<<v*90<<','<<p.position.x<<','<<p.position.z<<','<<q.position.x<<','<<q.position.z
                    <<','<<p.position.y<<','<<q.position.y<<','<<p.normal.x<<','<<p.normal.y<<','<<p.normal.z
                    <<','<<q.normal.x<<','<<q.normal.y<<','<<q.normal.z<<','<<NormalBin(p.normal)<<','<<NormalBin(q.normal)
                    <<','<<a.states[i][v]<<','<<b.states[i][v]<<','<<1/p.normal.y<<','<<1/q.normal.y<<'\n';
            }
        }
        out.flush();
    }
    out.close();manifest<<"status=completed\n";manifest.close();
}
