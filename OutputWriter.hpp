#pragma once
#include "DataModel.hpp"
#include <iosfwd>
#include <string>

class OutputWriter {
public:
    bool write(const std::string& outputPath, const Design& design);

private:
    void writeOutline(std::ostream& os, const Design& design) const;
    void writeBlocks(std::ostream& os, const Design& design) const;
    void writeChannels(std::ostream& os, const Design& design) const;
    void writePaths(std::ostream& os, const Design& design) const;
};
