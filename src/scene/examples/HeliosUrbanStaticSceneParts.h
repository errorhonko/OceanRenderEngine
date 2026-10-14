#pragma once
#include <array>
#include <cstddef>

// Fixed scene recipe from HELIOS++ urban_dyn_scene.xml at commit
// 2b3585aa766213ae9a46c4790ad4f3594ffa213d. Static initial-state adapter,
// not a general XML interpreter. Rotations are global, in listed order.
struct HeliosUrbanRotationSpec { char sourceAxis; double degrees; };
struct HeliosUrbanPartSpec
{
    const char* file;
    double scale;
    std::array<double,3> sourceOffset;
    std::array<HeliosUrbanRotationSpec,2> rotations;
    std::size_t rotationCount;
    bool onGround;
    bool originallyDynamic;
};
inline constexpr std::array<HeliosUrbanPartSpec,42> HeliosUrbanStaticParts{{
    {"vivaro.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,true},
    {"vivaro.001.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,true},
    {"Inliner-Bahn.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.003.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.005.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.006.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.008.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.013.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.016.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.017.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.018.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.019.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.020.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.021.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.028.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.029.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.030.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.031.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.032.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.033.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.036.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"element.037.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"small_maple.obj",1,{2.16509,69.8026,6.94842},{{{'z',1.11},{'x',0}}},1,true,false},
    {"small_maple.obj",1,{8.13497,53.3513,7.66822},{{{'z',-23.2},{'x',0}}},1,true,false},
    {"small_maple.obj",1,{8.00714,40.7623,7.66822},{{{'z',-23.2},{'x',0}}},1,true,false},
    {"small_maple.obj",1,{0.747932,88.9463,6.55873},{{{'z',1.11},{'x',0}}},1,true,false},
    {"small_maple.obj",1.271,{-3.42266,108.729,5.80861},{{{'z',-9.59},{'x',0}}},1,true,false},
    {"small_maple.obj",0.322,{-18.5766,84.0126,6.6221},{{{'z',1.11},{'x',0}}},1,true,false},
    {"small_maple.obj",0.322,{-17.9874,83.0696,6.6221},{{{'z',-23.2},{'x',0}}},1,true,false},
    {"small_maple.obj",0.322,{-17.7965,79.5875,6.6029},{{{'z',-23.2},{'x',0}}},1,true,false},
    {"white_birch.obj",1,{35.719,51.7917,7.59115},{{{'z',64},{'x',0}}},1,true,false},
    {"white_birch.obj",1,{24.2854,22.3744,8.30903},{{{'x',0},{'x',0}}},0,true,false},
    {"white_birch.obj",1,{-21.9022,41.0781,5.93214},{{{'z',64},{'x',0}}},1,true,false},
    {"Terrain.obj",1,{0,0,0},{{{'x',0},{'x',0}}},0,false,false},
    {"vivaro.000.obj",1,{-14.3558,62.1707,7},{{{'z',-177},{'x',0}}},1,true,false},
    {"vivaro.000.obj",1,{-14.7988,67.8695,7.12181},{{{'z',-177},{'x',0}}},1,true,false},
    {"vivaro.000.obj",1,{-15.1683,73.5638,7.23869},{{{'z',-356},{'x',0}}},1,true,false},
    {"vivaro.000.obj",1,{-13.6314,50.2513,6.58378},{{{'z',-356},{'x',2.51}}},2,true,false},
    {"vivaro.000.obj",1,{-20.8724,64.6867,7},{{{'z',-177},{'x',0}}},1,true,false},
    {"vivaro.000.obj",1,{-21.3052,71.8556,7},{{{'z',-177},{'x',0}}},1,true,false},
    {"vivaro.000.obj",1,{-20.5888,59.0784,7},{{{'z',-177},{'x',0}}},1,true,false}
}};
