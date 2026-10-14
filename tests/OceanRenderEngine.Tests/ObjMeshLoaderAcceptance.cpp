#include "scene/import/ObjMeshLoader.h"
#include "geometry/mesh/TriangleMeshAggregate.h"

#include <cmath>
#include <algorithm>
#include <array>
#include <filesystem>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void Check(const char* name, bool condition)
{
    if (!condition)
    {
        std::cerr << "[FAIL] " << name << '\n';
        throw std::runtime_error(name);
    }
    std::cout << "[PASS] " << name << '\n';
}

std::string Fixture(const char* name)
{
    // /FC in VS and absolute compiler inputs in the standalone test driver
    // make this independent of the executable's current working directory.
    return (std::filesystem::path(__FILE__).parent_path() /
        "fixtures" / "obj_import" / name).string();
}
}

void RunObjMeshLoaderAcceptanceTests()
{
    bool emptyPathRejected = false;
    try { (void)ObjMeshLoader::Read(""); }
    catch (const std::invalid_argument&) { emptyPathRejected = true; }
    Check("OBJ reader rejects empty path", emptyPathRejected);

    const auto missingPath = Fixture("intentionally_missing.obj");
    bool missingRejected = false, pathInError = false, detailInError = false;
    try { (void)ObjMeshLoader::Read(missingPath); }
    catch (const std::runtime_error& error)
    {
        missingRejected = true;
        const std::string message = error.what();
        pathInError = message.find(missingPath) != std::string::npos;
        const auto newline = message.find('\n');
        detailInError = newline != std::string::npos && newline + 1 < message.size();
    }
    Check("OBJ reader rejects missing file", missingRejected);
    Check("OBJ reader failure includes file path", pathInError);
    Check("OBJ reader failure retains parser detail", detailInError);

    const auto triangle = ObjMeshLoader::Read(Fixture("triangle.obj"));
    Check("OBJ reader returned object owns valid parsed data", triangle.Valid());
    Check("OBJ reader preserves position units and axes",
        triangle.GetAttrib().vertices.size() == 9 &&
        triangle.GetAttrib().vertices[3] == 2 &&
        triangle.GetAttrib().vertices[8] == 3);
    Check("OBJ reader keeps absent normal and UV arrays empty",
        triangle.GetAttrib().normals.empty() && triangle.GetAttrib().texcoords.empty());
    Check("OBJ reader disables unused vertex colors", triangle.GetAttrib().colors.empty());
    Check("OBJ reader retains triangle shape",
        triangle.GetShapes().size() == 1 && triangle.GetShapes()[0].name == "triangle");
    const auto& simple = triangle.GetShapes()[0].mesh;
    Check("OBJ reader retains one triangle", simple.num_face_vertices.size() == 1 &&
        simple.num_face_vertices[0] == 3 && simple.indices.size() == 3);
    bool positiveIndices = true, absentIndices = true;
    for (std::size_t i = 0; i < simple.indices.size(); ++i)
    {
        positiveIndices &= simple.indices[i].vertex_index == static_cast<int>(i);
        absentIndices &= simple.indices[i].normal_index == -1 &&
            simple.indices[i].texcoord_index == -1;
    }
    Check("OBJ reader converts positive indices to zero based", positiveIndices);
    Check("OBJ reader marks absent corner attributes", absentIndices);
    Check("OBJ reader clean geometry has no warnings", triangle.Warning().empty());

    const auto grouped = ObjMeshLoader::Read(Fixture("material_groups.obj"));
    Check("OBJ reader retains multiple named shapes", grouped.GetShapes().size() == 2 &&
        grouped.GetShapes()[0].name == "quad" && grouped.GetShapes()[1].name == "triangle");
    Check("OBJ reader triangulates simple quad",
        grouped.GetShapes()[0].mesh.num_face_vertices.size() == 2 &&
        grouped.GetShapes()[0].mesh.indices.size() == 6);
    bool allTriangles = true, negativeIndices = true;
    bool referenced[4] = {};
    for (const auto& shape : grouped.GetShapes())
    {
        for (const auto count : shape.mesh.num_face_vertices) allTriangles &= count == 3;
        for (const auto& index : shape.mesh.indices)
        {
            const bool valid = index.vertex_index >= 0 && index.vertex_index < 4;
            negativeIndices &= valid && index.normal_index == 0 &&
                index.texcoord_index == index.vertex_index;
            if (valid) referenced[index.vertex_index] = true;
        }
    }
    Check("OBJ reader parsed faces are triangles", allTriangles);
    Check("OBJ reader resolves negative position UV normal indices", negativeIndices);
    Check("OBJ reader triangulation keeps all quad corners",
        referenced[0] && referenced[1] && referenced[2] && referenced[3]);
    Check("OBJ reader retains independent attribute arrays",
        grouped.GetAttrib().vertices.size() == 12 &&
        grouped.GetAttrib().texcoords.size() == 8 && grouped.GetAttrib().normals.size() == 3);
    Check("OBJ reader locates MTL relative to OBJ",
        grouped.GetMaterials().size() == 1 && grouped.GetMaterials()[0].name == "surface");
    const auto& material = grouped.GetMaterials()[0];
    Check("OBJ reader preserves MTL diffuse values",
        std::abs(material.diffuse[0] - 0.2) < 1e-6 &&
        std::abs(material.diffuse[1] - 0.4) < 1e-6 &&
        std::abs(material.diffuse[2] - 0.6) < 1e-6);
    bool assigned = true;
    for (const auto& shape : grouped.GetShapes())
        for (const auto id : shape.mesh.material_ids) assigned &= id == 0;
    Check("OBJ reader retains per face material IDs", assigned);
    Check("OBJ reader complete MTL has no warnings", grouped.Warning().empty());

    const auto incomplete = ObjMeshLoader::Read(Fixture("missing_material.obj"));
    Check("OBJ reader missing MTL keeps usable geometry", incomplete.Valid() &&
        incomplete.GetShapes().size() == 1 && incomplete.GetShapes()[0].mesh.indices.size() == 3);
    Check("OBJ reader preserves missing MTL warning", !incomplete.Warning().empty());
    Check("OBJ reader missing MTL does not invent a material",
        incomplete.GetMaterials().empty() && incomplete.GetShapes()[0].mesh.material_ids[0] == -1);
    const auto empty = ObjMeshLoader::Read(Fixture("empty.obj"));
    Check("OBJ reader leaves empty geometry validation to conversion",
        empty.Valid() && empty.GetAttrib().vertices.empty() && empty.GetShapes().empty());
}

namespace {
tinyobj::ObjReader Parse(const std::string& text, bool triangulate = true)
{
    tinyobj::ObjReaderConfig config;
    config.triangulate = triangulate;
    config.vertex_color = false;
    tinyobj::ObjReader reader;
    if (!reader.ParseFromString(text, "", config))
        throw std::runtime_error(reader.Error());
    return reader;
}
template<class Function>
bool Rejects(Function function)
{
    try { function(); }
    catch (const std::exception&) { return true; }
    return false;
}
bool Near(const Vector3f& a, const Vector3f& b)
{
    return (a - b).norm() < 1e-5f;
}
}

void RunObjMeshConversionAcceptanceTests()
{
    Check("OBJ conversion rejects uninitialized reader", Rejects([] {
        (void)ObjMeshLoader::Convert(tinyobj::ObjReader{}); }));
    Check("OBJ conversion rejects empty geometry", Rejects([] {
        (void)ObjMeshLoader::Convert(ObjMeshLoader::Read(Fixture("empty.obj"))); }));
    auto parts = ObjMeshLoader::Convert(ObjMeshLoader::Read(Fixture("triangle.obj")));
    Check("OBJ conversion retains named mesh", parts.size() == 1 && parts[0].name == "triangle");
    const auto& mesh = *parts[0].mesh;
    Check("OBJ conversion owns data after reader destruction", mesh.Vertices().size() == 3 &&
        Near(mesh.Vertices()[1].position, {2, 0, 0}));
    Check("OBJ conversion retains triangle winding", mesh.Triangles().size() == 1 &&
        mesh.Triangles()[0] == TriangleIndices{0, 1, 2});
    Check("OBJ conversion missing normals use flat fallback", mesh.Vertices()[0].normal.near_zero() &&
        mesh.Vertices()[1].normal.near_zero() && mesh.Vertices()[2].normal.near_zero());
    Check("OBJ conversion missing UVs default to zero", mesh.Vertices()[0].uv.near_zero());
    Check("OBJ conversion unassigned material remains unassigned", parts[0].materialIds == std::vector<int>{-1});
    TriangleMeshAggregate aggregate(parts[0].mesh, nullptr);
    HitRecord hit;
    Check("OBJ converted mesh BVH hits expected distance", aggregate.hit(
        Ray({0.5f, -2, 0.5f}, {0, 1, 0}), 1e-4f, 10, hit) && std::abs(hit.t - 2) < 1e-5f);
    Check("OBJ converted mesh geometric normal follows winding", Near(hit.geometricNormal, {0, -1, 0}));
    Check("OBJ converted mesh missing normals shade with geometric normal", Near(hit.normal, hit.geometricNormal));
    Check("OBJ converted mesh bounds preserve coordinates", Near(aggregate.Bounds().pMin, {0, 0, 0}) &&
        Near(aggregate.Bounds().pMax, {2, 0, 3}));

    const auto groupedReader = ObjMeshLoader::Read(Fixture("material_groups.obj"));
    const auto grouped = ObjMeshLoader::Convert(groupedReader);
    Check("OBJ conversion keeps separate object groups", grouped.size() == 2 &&
        grouped[0].name == "quad" && grouped[1].name == "triangle");
    Check("OBJ conversion shares complete corner tuples", grouped[0].mesh->Vertices().size() == 4 &&
        grouped[0].mesh->Triangles().size() == 2);
    Check("OBJ conversion keeps per triangle material IDs", grouped[0].materialIds == std::vector<int>({0, 0}) &&
        grouped[1].materialIds == std::vector<int>({0}));
    bool uvCorrect = true;
    for (const auto& vertex : grouped[0].mesh->Vertices())
        uvCorrect &= std::abs(vertex.uv.x - vertex.position.x / 2) < 1e-6f &&
            std::abs(vertex.uv.y - vertex.position.z / 3) < 1e-6f;
    Check("OBJ conversion copies corner UVs", uvCorrect);

    const std::string positions = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n";
    const auto seams = ObjMeshLoader::Convert(Parse(positions +
        "vt 0 0\nvt 1 0\nvt 1 1\nvt 0.25 0.25\n"
        "f 1/1 2/2 3/3\nf 1/4 3/3 4/1\n"));
    Check("OBJ conversion splits UV seams without duplicating all corners",
        seams[0].mesh->Vertices().size() == 5);
    const auto hard = ObjMeshLoader::Convert(Parse(positions +
        "vn 0 0 2\nvn 0.6 0 0.8\nf 1//1 2//1 3//1\nf 1//2 3//2 4//2\n"));
    Check("OBJ conversion splits normal seams", hard[0].mesh->Vertices().size() == 6);
    Check("OBJ conversion normalizes supplied normals", Near(hard[0].mesh->Vertices()[0].normal, {0, 0, 1}) &&
        Near(hard[0].mesh->Vertices()[3].normal, {0.6f, 0, 0.8f}));
    const auto smooth = ObjMeshLoader::Convert(Parse(
        "v 0 0 1\nv 1 0 1\nv 0 1 1\nvt 0 0\nvt 1 0\nvt 0 1\n"
        "vn 0.6 0 0.8\nf 1/1/1 2/2/1 3/3/1\n"));
    TriangleMeshAggregate smoothAggregate(smooth[0].mesh, nullptr);
    Check("OBJ conversion smooth mesh remains intersectable", smoothAggregate.hit(
        Ray({0.25f, 0.25f, 3}, {0, 0, -1}), 1e-4f, 10, hit));
    Check("OBJ conversion preserves distinct shading and geometric normals",
        Near(hit.geometricNormal, {0, 0, 1}) && Near(hit.normal, {0.6f, 0, 0.8f}));
    Check("OBJ conversion interpolates imported UVs", std::abs(hit.u - 0.25f) < 1e-6f &&
        std::abs(hit.v - 0.25f) < 1e-6f);

    const auto partial = ObjMeshLoader::Convert(Parse(positions +
        "vn 0.6 0 0.8\nf 1//1 2 3//1\n"));
    Check("OBJ conversion partial normals fall back for whole face",
        partial[0].mesh->Vertices()[0].normal.near_zero() &&
        partial[0].mesh->Vertices()[1].normal.near_zero() &&
        partial[0].mesh->Vertices()[2].normal.near_zero());
    const auto zeroNormals = ObjMeshLoader::Convert(Parse(positions +
        "vn 0 0 0\nf 1//1 2//1 3//1\n"));
    Check("OBJ conversion zero normals use geometric fallback",
        zeroNormals[0].mesh->Vertices()[0].normal.near_zero());
    Check("OBJ conversion rejects nontriangulated input", Rejects([&] {
        (void)ObjMeshLoader::Convert(Parse(positions + "f 1 2 3 4\n", false)); }));
    Check("OBJ conversion rejects zero area triangles", Rejects([] {
        (void)ObjMeshLoader::Convert(Parse("v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3\n")); }));
    Check("OBJ conversion rejects repeated vertex triangles", Rejects([&] {
        (void)ObjMeshLoader::Convert(Parse(positions + "f 1 2 2\n")); }));
    Check("OBJ import rejects out of range position indices", Rejects([&] {
        (void)ObjMeshLoader::Convert(Parse(positions + "f 1 2 99\n")); }));
    Check("OBJ import rejects out of range UV indices", Rejects([&] {
        (void)ObjMeshLoader::Convert(Parse(positions + "vt 0 0\nf 1/1 2/1 3/99\n")); }));
    Check("OBJ import rejects out of range normal indices", Rejects([&] {
        (void)ObjMeshLoader::Convert(Parse(positions + "vn 0 0 1\nf 1//1 2//1 3//99\n")); }));
    Check("OBJ import rejects float overflow positions", Rejects([] {
        (void)ObjMeshLoader::Convert(Parse("v 1e40 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\n")); }));
    const auto unavailable = ObjMeshLoader::Convert(ObjMeshLoader::Read(Fixture("missing_material.obj")));
    Check("OBJ conversion does not invent missing LiDAR materials",
        unavailable[0].materialIds == std::vector<int>{-1});
}

void RunObjUnitAxisAcceptanceTests()
{
    const auto reader = Parse(
        "v 0 0 1000\nv 200 0 1000\nv 0 300 1000\n"
        "vt 0 0\nvt 1 0\nvt 0 1\nvn 0 0 5\nf 1/1/1 2/2/1 3/3/1\n");
    ObjImportOptions zUp;
    zUp.metersPerUnit = 0.01;
    zUp.sourceAxesInEngine = {ObjAxisDirection::PositiveX,
        ObjAxisDirection::NegativeZ, ObjAxisDirection::PositiveY};
    const auto converted = ObjMeshLoader::Convert(reader, zUp);
    const auto& mesh = *converted[0].mesh;
    Check("OBJ cm Z up positions convert to metres Y up",
        Near(mesh.Vertices()[0].position, {0, 10, 0}) &&
        Near(mesh.Vertices()[1].position, {2, 10, 0}) &&
        Near(mesh.Vertices()[2].position, {0, 10, -3}));
    Check("OBJ Z up normals rotate but do not receive unit scale",
        Near(mesh.Vertices()[0].normal, {0, 1, 0}) && std::abs(mesh.Vertices()[0].normal.norm() - 1) < 1e-6f);
    Check("OBJ right handed axis rotation retains winding", mesh.Triangles()[0] == TriangleIndices{0, 1, 2});
    Check("OBJ units and axes preserve UVs", Near(mesh.Vertices()[1].uv, {1, 0, 0}) &&
        Near(mesh.Vertices()[2].uv, {0, 1, 0}));
    Check("OBJ normalization does not mutate parsed source data", reader.GetAttrib().vertices[2] == 1000 &&
        reader.GetAttrib().normals[2] == 5);
    TriangleMeshAggregate aggregate(converted[0].mesh, nullptr);
    HitRecord hit;
    Check("OBJ normalized BVH bounds are in metres", Near(aggregate.Bounds().pMin, {0, 10, -3}) &&
        Near(aggregate.Bounds().pMax, {2, 10, 0}));
    Check("OBJ normalized mesh ray distance is three metres", aggregate.hit(
        Ray({0.5f, 13, -0.75f}, {0, -1, 0}), 1e-4f, 20, hit) && std::abs(hit.t - 3) < 1e-5f);
    Check("OBJ normalized shading and geometric normals agree", Near(hit.normal, {0, 1, 0}) &&
        Near(hit.geometricNormal, {0, 1, 0}));
    Check("OBJ normalized mesh hit UVs are unchanged", std::abs(hit.u - 0.25f) < 1e-6f && std::abs(hit.v - 0.25f) < 1e-6f);
    MeshTriangle triangle(converted[0].mesh, 0, nullptr);
    Check("OBJ normalized triangle area is three square metres", std::abs(triangle.SurfaceArea() - 3) < 1e-5f);

    ObjImportOptions mirror;
    mirror.metersPerUnit = 0.01;
    mirror.sourceAxesInEngine = {ObjAxisDirection::PositiveX,
        ObjAxisDirection::PositiveY, ObjAxisDirection::NegativeZ};
    const auto mirrored = ObjMeshLoader::Convert(reader, mirror);
    Check("OBJ left handed mapping repairs triangle winding",
        mirrored[0].mesh->Triangles()[0] == TriangleIndices{0, 2, 1});
    TriangleMeshAggregate mirroredAggregate(mirrored[0].mesh, nullptr);
    Check("OBJ mirrored mesh ray hits expected metre distance", mirroredAggregate.hit(
        Ray({0.5f, 0.75f, -13}, {0, 0, 1}), 1e-4f, 20, hit) && std::abs(hit.t - 3) < 1e-5f);
    Check("OBJ mirrored geometric normal agrees with transformed normal",
        Near(hit.normal, {0, 0, -1}) && Near(hit.geometricNormal, {0, 0, -1}));
    Check("OBJ winding repair keeps UVs attached to corners", std::abs(hit.u - 0.25f) < 1e-6f && std::abs(hit.v - 0.25f) < 1e-6f);
    const auto noNormal = Parse("v 0 0 1000\nv 200 0 1000\nv 0 300 1000\nf 1 2 3\n");
    const auto flatMirror = ObjMeshLoader::Convert(noNormal, mirror);
    TriangleMeshAggregate flatAggregate(flatMirror[0].mesh, nullptr);
    Check("OBJ axis normalization keeps missing normals as flat markers", flatMirror[0].mesh->Vertices()[0].normal.near_zero());
    Check("OBJ reflected flat face shading keeps physical front", flatAggregate.hit(
        Ray({0.5f, 0.75f, -13}, {0, 0, 1}), 1e-4f, 20, hit) && Near(hit.normal, {0, 0, -1}));
    ObjImportOptions millimetres;
    millimetres.metersPerUnit = 0.001;
    const auto mm = ObjMeshLoader::Convert(reader, millimetres);
    Check("OBJ millimetres convert with caller scale", Near(mm[0].mesh->Vertices()[1].position, {0.2f, 0, 1}));
    Check("OBJ unit conversion does not scale shading normals", Near(mm[0].mesh->Vertices()[0].normal, {0, 0, 1}));
    Check("OBJ axis normalization preserves material IDs", converted[0].materialIds == std::vector<int>{-1});

    const auto base = Parse("v 0 0 1\nv 2 0 1\nv 0 3 1\nvn 0 0 1\nf 1//1 2//1 3//1\n");
    std::array<int, 3> permutation{0, 1, 2};
    const std::array<Vector3f, 3> unitAxes{Vector3f(1, 0, 0), Vector3f(0, 1, 0), Vector3f(0, 0, 1)};
    bool allPositions = true, allNormals = true, allHits = true, allWinding = true;
    std::size_t combinations = 0;
    do {
        for (int signs = 0; signs < 8; ++signs)
        {
            ObjImportOptions options;
            std::array<Vector3f, 3> expectedAxes;
            for (int i = 0; i < 3; ++i)
            {
                const int negative = (signs >> i) & 1;
                options.sourceAxesInEngine[i] = static_cast<ObjAxisDirection>(2 * permutation[i] + negative);
                expectedAxes[i] = unitAxes[permutation[i]] * (negative ? -1.0f : 1.0f);
            }
            const auto part = ObjMeshLoader::Convert(base, options);
            allPositions &= Near(part[0].mesh->Vertices()[0].position, expectedAxes[2]) &&
                Near(part[0].mesh->Vertices()[1].position, expectedAxes[0] * 2 + expectedAxes[2]);
            allNormals &= Near(part[0].mesh->Vertices()[0].normal, expectedAxes[2]);
            const bool reversed = expectedAxes[0].cross(expectedAxes[1]).dot(expectedAxes[2]) < 0;
            allWinding &= part[0].mesh->Triangles()[0] == (reversed ? TriangleIndices{0, 2, 1} : TriangleIndices{0, 1, 2});
            const auto target = expectedAxes[0] * 0.5f + expectedAxes[1] * 0.75f + expectedAxes[2];
            TriangleMeshAggregate object(part[0].mesh, nullptr);
            allHits &= object.hit(Ray(target + expectedAxes[2] * 2, -expectedAxes[2]), 1e-4f, 10, hit) &&
                std::abs(hit.t - 2) < 1e-5f && Near(hit.geometricNormal, expectedAxes[2]);
            ++combinations;
        }
    } while (std::next_permutation(permutation.begin(), permutation.end()));
    Check("OBJ all forty eight signed axis mappings preserve positions", combinations == 48 && allPositions);
    Check("OBJ all forty eight signed axis mappings preserve normals", allNormals);
    Check("OBJ all signed axis mappings repair winding only when needed", allWinding);
    Check("OBJ all signed axis mappings retain metric BVH intersection", allHits);
    for (const auto scale : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        Check("OBJ rejects invalid metres per unit", Rejects([&] {
            ObjImportOptions options; options.metersPerUnit = scale;
            (void)ObjMeshLoader::Convert(base, options); }));
    Check("OBJ rejects duplicate signed source axes", Rejects([&] {
        ObjImportOptions options; options.sourceAxesInEngine = {ObjAxisDirection::PositiveX, ObjAxisDirection::NegativeX, ObjAxisDirection::PositiveZ};
        (void)ObjMeshLoader::Convert(base, options); }));
    Check("OBJ rejects unknown source axis enumeration", Rejects([&] {
        ObjImportOptions options; options.sourceAxesInEngine[0] = static_cast<ObjAxisDirection>(99);
        (void)ObjMeshLoader::Convert(base, options); }));
    Check("OBJ rejects normalized float coordinate overflow", Rejects([&] {
        ObjImportOptions options; options.metersPerUnit = 1e100;
        (void)ObjMeshLoader::Convert(base, options); }));
    Check("OBJ rejects unit scale collapsing triangle geometry", Rejects([&] {
        ObjImportOptions options; options.metersPerUnit = 1e-300;
        (void)ObjMeshLoader::Convert(base, options); }));
    const auto original = ObjMeshLoader::Convert(base);
    const auto explicitDefault = ObjMeshLoader::Convert(base, ObjImportOptions{});
    Check("OBJ default options preserve previous conversion", Near(original[0].mesh->Vertices()[1].position,
        explicitDefault[0].mesh->Vertices()[1].position) && original[0].mesh->Triangles() == explicitDefault[0].mesh->Triangles());

    ObjImportOptions cleanup; cleanup.skipDegenerateTriangles = true;
    ObjImportDiagnostics report;
    const auto mixed = Parse("v 0 0 0\nv 1 0 0\nv 0 1 0\nv 2 0 0\n"
        "g valid\nf 1 2 3\nf 1 2 2\nf 1 2 4\ng empty\nf 1 1 1\n");
    const auto cleaned = ObjMeshLoader::Convert(mixed, cleanup, &report);
    Check("OBJ explicit cleanup reports every zero area triangle", report.skippedDegenerateTriangleCount == 3 && report.importedTriangleCount == 1);
    Check("OBJ explicit cleanup removes degenerate only groups", cleaned.size() == 1 && cleaned[0].mesh->Triangles().size() == 1);
    Check("OBJ cleanup keeps material IDs aligned", cleaned[0].materialIds == std::vector<int>{-1});
    Check("OBJ cleanup preserves source triangle data", mixed.GetShapes()[0].mesh.num_face_vertices.size() == 3);
    Check("OBJ cleanup never becomes the default", Rejects([&] { (void)ObjMeshLoader::Convert(mixed); }));
    Check("OBJ cleanup rejects a wholly empty result", Rejects([&] { (void)ObjMeshLoader::Convert(Parse("v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3\n"), cleanup); }));
    Check("OBJ cleanup still validates skipped corner attributes", Rejects([&] { (void)ObjMeshLoader::Convert(Parse("v 0 0 0\nv 1 0 0\nf 1/99 2 2\n"), cleanup); }));
    Check("OBJ cleanup still rejects scale induced collapse", Rejects([&] { auto tiny=cleanup; tiny.metersPerUnit=1e-300; (void)ObjMeshLoader::Convert(base, tiny); }));
    const auto small = ObjMeshLoader::Convert(Parse("v 0 0 0\nv 1e-8 0 0\nv 0 1e-8 0\nf 1 2 3\n"), cleanup, &report);
    Check("OBJ cleanup retains tiny nonzero area faces without epsilon", small[0].mesh->Triangles().size() == 1 && report.skippedDegenerateTriangleCount == 0);
    report = {123,456};
    (void)Rejects([&] { (void)ObjMeshLoader::Convert(mixed, ObjImportOptions{}, &report); });
    Check("OBJ diagnostics remain unchanged on failed conversion", report.importedTriangleCount == 123 && report.skippedDegenerateTriangleCount == 456);
}
