#include <filesystem>
#include <iostream>
#include <stdexcept>

void RunToyblocksImportAcceptanceTests();
void ExportToyblocksImportExperiment(const std::filesystem::path& directory);

// Windows wide arguments preserve non-ASCII output paths.
int wmain(int argc, wchar_t** argv)
{
    try {
        if (argc != 2) throw std::runtime_error("Expected one export directory argument.");
        RunToyblocksImportAcceptanceTests();
        ExportToyblocksImportExperiment(std::filesystem::path(argv[1]));
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
