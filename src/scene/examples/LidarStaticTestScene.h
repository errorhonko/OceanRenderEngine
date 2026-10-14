#pragma once
#include "core/ray/HittableList.h"
#include "lidar/scan/LidarScanPlan.h"
#include "geometry/primitives/Triangle.h"
#include <memory>
#include <utility>

// 场景构建与传感器算法分开；世界 Y 向上，单位 m。
// 当前所有物体使用调用方传给驱动的同一个散射模型，不设置逐物体反射率。
struct LidarStaticTestScene
{
    HittableList world;
    LidarScanPoseConfig sensorPose{{0,1.5f,0},{0,0,1},{0,1,0}};
};

namespace lidar_static_scene_detail {
inline void AddQuad(HittableList& world,Vector3f a,Vector3f b,Vector3f c,Vector3f d,const Vector3f& normal)
{
    if ((b-a).cross(c-a).dot(normal)<0) std::swap(b,d);
    world.add(std::make_shared<Triangle>(a,b,c,normal,normal,normal,
        Vector3f(0,0,0),Vector3f(1,0,0),Vector3f(1,1,0),nullptr));
    world.add(std::make_shared<Triangle>(a,c,d,normal,normal,normal,
        Vector3f(0,0,0),Vector3f(1,1,0),Vector3f(0,1,0),nullptr));
}
inline void AddBox(HittableList& world,const Vector3f& low,const Vector3f& high)
{
    const float x=low.x,y=low.y,z=low.z,X=high.x,Y=high.y,Z=high.z;
    AddQuad(world,{x,y,z},{x,y,Z},{x,Y,Z},{x,Y,z},{-1,0,0});
    AddQuad(world,{X,y,z},{X,Y,z},{X,Y,Z},{X,y,Z},{1,0,0});
    AddQuad(world,{x,y,z},{X,y,z},{X,y,Z},{x,y,Z},{0,-1,0});
    AddQuad(world,{x,Y,z},{x,Y,Z},{X,Y,Z},{X,Y,z},{0,1,0});
    AddQuad(world,{x,y,z},{x,Y,z},{X,Y,z},{X,y,z},{0,0,-1});
    AddQuad(world,{x,y,Z},{X,y,Z},{X,Y,Z},{x,Y,Z},{0,0,1});
}
}

inline LidarStaticTestScene CreateLidarStaticTestScene()
{
    LidarStaticTestScene scene;
    using namespace lidar_static_scene_detail;
    // 40 x 40 m 有限地面，正面向上。
    AddQuad(scene.world,{-20,0,-20},{-20,0,20},{20,0,20},{20,0,-20},{0,1,0});
    // 前方 z=12 m 的墙：宽 20 m、高 6 m，正面朝传感器。
    AddQuad(scene.world,{-10,0,12},{-10,6,12},{10,6,12},{10,0,12},{0,0,-1});
    // 中央近箱体与右侧较高箱体，均为封闭三角形表面。
    AddBox(scene.world,{-1,0,5},{1,2,7});
    AddBox(scene.world,{4,0,8},{6,3,10});
    return scene;
}
