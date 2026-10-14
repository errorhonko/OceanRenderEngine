#include <filesystem>
#include <iostream>
#include <stdexcept>
void ExportHeliosUrbanStaticImport(const std::filesystem::path& directory);
int wmain(int argc,wchar_t** argv)
{
    try {
        if(argc!=2)throw std::runtime_error("Expected one fresh report directory argument.");
        ExportHeliosUrbanStaticImport(std::filesystem::path(argv[1]));
        std::cout<<"Urban static import verified; summary.txt and scene-layout.json exported.\n";
        return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
