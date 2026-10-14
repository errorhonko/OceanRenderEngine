#pragma once
#include "lidar/output/LidarPointCloudFrame.h"
#include "lidar/scan/LidarPoseTrajectory.h"
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <limits>
#include <stdexcept>
#include <string>
#include <sstream>
#include <map>
#include <set>
#include <tuple>

// PCD 0.7 ASCII；扩展字段可被通用 XYZ/XYZI 读取器忽略。
// intensity = 基线扣除后的 ADC 码幅值，绝不是已标定的反射率。
// time = 发射时刻减帧起点，单位 s；ring 保存完整 uint32 channelId。
enum class LidarPcdCoordinates {World,SensorAtEmission};

class LidarPcdWriter
{
public:
    static void Write(std::ostream& output,const LidarPointCloudFrame& frame,
        LidarPcdCoordinates coordinates=LidarPcdCoordinates::World)
    {
        const auto data=Serialize(frame,coordinates); // 所有数据先验证
        output<<data;
        if(!output)throw std::runtime_error("PCD output stream failed.");
    }
    static void Save(const std::filesystem::path& path,const LidarPointCloudFrame& frame,bool overwrite=false,
        LidarPcdCoordinates coordinates=LidarPcdCoordinates::World)
    {
        const auto data=Serialize(frame,coordinates);
        if(!overwrite && std::filesystem::exists(path))
            throw std::runtime_error("Refusing to overwrite existing PCD file.");
        std::ofstream out(path,std::ios::binary|std::ios::trunc);
        if(!out)throw std::runtime_error("Cannot open PCD output path.");
        out<<data;out.close();
        if(!out)throw std::runtime_error("Writing PCD file failed.");
    }
    static void Save(const std::filesystem::path& path,const LidarPointCloudFrame& frame,
        LidarPcdCoordinates coordinates,bool overwrite=false) {Save(path,frame,overwrite,coordinates);}
private:
    static std::string Serialize(const LidarPointCloudFrame& frame,LidarPcdCoordinates coordinates)
    {
        if(coordinates!=LidarPcdCoordinates::World && coordinates!=LidarPcdCoordinates::SensorAtEmission)
            throw std::invalid_argument("Unknown PCD coordinate representation.");
        if(!std::isfinite(frame.startTimeSeconds) || frame.startTimeSeconds<0 ||
            !std::isfinite(frame.endTimeSeconds) || frame.endTimeSeconds<=frame.startTimeSeconds)
            throw std::invalid_argument("PCD frame time interval is invalid.");
        const auto worldRotation=LidarPoseTrajectory::Quaternion(frame.sensorPose);
        const bool raw=coordinates==LidarPcdCoordinates::SensorAtEmission;
        const auto rotation=raw?std::array<double,4>{1,0,0,0}:worldRotation;
        const auto viewPosition=raw?Vector3f(0,0,0):frame.sensorPose.originWorld;
        std::set<std::pair<std::uint64_t, std::uint32_t>> ids;
        std::map<std::uint64_t, std::tuple<std::uint32_t, std::uint32_t, double>> pulseMetadata;
        for(const auto& p:frame.points) {
            if(raw && (!p.positionSensor || !std::isfinite(p.positionSensor->x) ||
                !std::isfinite(p.positionSensor->y) || !std::isfinite(p.positionSensor->z)))
                throw std::invalid_argument("Raw sensor PCD requires finite per point sensor coordinates.");
            if(!std::isfinite(p.positionWorld.x)||!std::isfinite(p.positionWorld.y)||!std::isfinite(p.positionWorld.z)||
                !std::isfinite(p.emissionTimeSeconds)||p.emissionTimeSeconds<frame.startTimeSeconds||p.emissionTimeSeconds>=frame.endTimeSeconds||
                !std::isfinite(p.arrivalTimeSeconds)||p.arrivalTimeSeconds<p.emissionTimeSeconds||
                !std::isfinite(p.rangeMeters)||p.rangeMeters<0||
                !std::isfinite(p.peakAboveBaselineCodes)||p.peakAboveBaselineCodes<0||
                p.peakAboveBaselineCodes>std::numeric_limits<float>::max()||
                (p.rangeSource!=LidarPointRangeSource::Raw && p.rangeSource!=LidarPointRangeSource::Corrected)||
                p.returnNumber == 0 || p.returnCount == 0 || p.returnNumber > p.returnCount ||
                !ids.emplace(p.pulseId, p.returnNumber).second)
                throw std::invalid_argument("PCD point fields are invalid.");
            const auto metadata = std::make_tuple(p.returnCount, p.channelId, p.emissionTimeSeconds);
            const auto [entry, inserted] = pulseMetadata.emplace(p.pulseId, metadata);
            if (!inserted && entry->second != metadata)
                throw std::invalid_argument("PCD returns from one pulse have inconsistent metadata.");
        }
        std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17);
        out<<"# .PCD v0.7\n# coordinate_frame "<<(raw?"sensor_at_emission_x_right_y_up_z_forward_m":"world_y_up_m")
            <<"\n# frame_id "<<frame.frameId
            <<"\n# frame_start_seconds "<<frame.startTimeSeconds<<"\n# frame_end_seconds "<<frame.endTimeSeconds
            <<"\n# intensity ADC_codes_not_reflectivity; time relative_emission_seconds; ring channel_id\n"
            <<"# pulse_id = (pulse_id_high << 32) | pulse_id_low; flags: peak=1 search=2 record=4 plateau=8 local=16\n"
            <<"# return_number: original detected time order 1-based; return_count: detected count before filtering\n"
            <<"VERSION .7\nFIELDS x y z intensity ring time range amplitude_codes peak_code pulse_id_low pulse_id_high flags range_source return_number return_count\n"
            <<"SIZE 4 4 4 4 4 8 8 8 4 4 4 4 4 4 4\nTYPE F F F F U F F F U U U U U U U\nCOUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\nWIDTH "<<frame.points.size()
            <<"\nHEIGHT 1\nVIEWPOINT "<<viewPosition.x<<' '<<viewPosition.y<<' '<<viewPosition.z;
        for(const auto value:rotation)out<<' '<<value;
        out<<"\nPOINTS "<<frame.points.size()<<"\nDATA ascii\n";
        for(const auto& p:frame.points) {
            const auto position=raw?*p.positionSensor:p.positionWorld;
            const unsigned flags=(p.peakOutOfRange?1:0)|(p.searchHasOutOfRangeSamples?2:0)|(p.hasAdcOutOfRangeSamples?4:0)|
                (p.plateauOutOfRange?8:0)|(p.localWindowHasOutOfRangeSamples?16:0);
            out<<position.x<<' '<<position.y<<' '<<position.z<<' '
                <<float(p.peakAboveBaselineCodes)<<' '<<p.channelId<<' '<<p.emissionTimeSeconds-frame.startTimeSeconds<<' '
                <<p.rangeMeters<<' '<<p.peakAboveBaselineCodes<<' '<<p.peakCode<<' '
                <<std::uint32_t(p.pulseId)<<' '<<std::uint32_t(p.pulseId>>32)<<' '<<flags<<' '<<unsigned(p.rangeSource)<<' '
                <<p.returnNumber<<' '<<p.returnCount<<'\n';
        }
        return out.str();
    }
};
