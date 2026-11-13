#include "state_codec.hpp"

namespace openmac {

std::vector<u8> IifxMachine::saveState() const {
    return IifxStateCodec::save(*this);
}

bool IifxMachine::loadState(const u8* data, std::size_t size,
                            std::string* error) {
    return IifxStateCodec::load(*this, data, size, error);
}

bool IifxMachine::saveStateFile(const std::string& path,
                                std::string* error) const {
    const std::vector<u8> checkpoint = saveState();
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        if (error) *error = "cannot open checkpoint for writing: " + path;
        return false;
    }
    output.write(reinterpret_cast<const char*>(checkpoint.data()),
                 static_cast<std::streamsize>(checkpoint.size()));
    if (!output) {
        if (error) *error = "failed while writing checkpoint: " + path;
        return false;
    }
    if (error) error->clear();
    return true;
}

bool IifxMachine::loadStateFile(const std::string& path, std::string* error) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        if (error) *error = "cannot open checkpoint for reading: " + path;
        return false;
    }
    std::vector<u8> checkpoint{std::istreambuf_iterator<char>(input),
                               std::istreambuf_iterator<char>()};
    return loadState(checkpoint.data(), checkpoint.size(), error);
}

} // namespace openmac
