#include "OutputWriter.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

using namespace std;

namespace {

    // 題目範例是整數就輸出整數，小數才輸出小數。
    // 例如：800.000 -> 800, 300.500 -> 300.5
    string fmt(double v) {
        if (fabs(v) < 1e-9) v = 0.0;
        double rounded = round(v);
        if (fabs(v - rounded) < 1e-6) v = rounded;

        ostringstream oss;
        oss << fixed << setprecision(6) << v;
        string s = oss.str();

        while (!s.empty() && s.back() == '0') s.pop_back();
        if (!s.empty() && s.back() == '.') s.pop_back();
        if (s.empty()) s = "0";

        return s;
    }

} // namespace

bool OutputWriter::write(const string& outputPath, const Design& design) {
    // Always keep the previous complete checkpoint intact while serializing the
    // next one.  The final rename/replace is atomic, so an external timeout can
    // never leave a half-written official cfg in outputPath.
    const string tempPath = outputPath + ".tmp";
    ofstream fout(tempPath, ios::binary | ios::trunc);
    if (!fout) return false;

    // 題目 cfg 格式：
    //
    // Outline <width> <height>
    //
    // BLOCK
    // BLOCK <block_name> <lx> <ly> <width> <height>
    // ...
    // END BLOCK
    //
    // CHANNEL
    // NumChannels <channel_count>
    // CHANNEL <channel_name> <lx> <ly> <width> <height>
    // ...
    // END CHANNEL
    //
    // PATH
    // NumRoutingPattern <routing_pattern_count>
    // PATH <net_count> <rect_name> <edge_idx> <rect_name> <edge_idx> ...
    // ...
    // END PATH

    writeOutline(fout, design);
    fout << "\n";

    writeBlocks(fout, design);
    fout << "\n";

    writeChannels(fout, design);
    fout << "\n";

    writePaths(fout, design);

    fout.flush();
    if (!fout) {
        fout.close();
        std::remove(tempPath.c_str());
        return false;
    }
    fout.close();

#ifdef _WIN32
    if (!MoveFileExA(tempPath.c_str(), outputPath.c_str(),
        MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        std::remove(tempPath.c_str());
        return false;
    }
#else
    if (std::rename(tempPath.c_str(), outputPath.c_str()) != 0) {
        std::remove(tempPath.c_str());
        return false;
    }
#endif

    return true;
}

void OutputWriter::writeOutline(ostream& os, const Design& design) const {
    // 注意：題目範例 Outline 是單行，沒有 END Outline。
    os << "Outline "
        << fmt(design.outlineW) << " "
        << fmt(design.outlineH) << "\n";
}

void OutputWriter::writeBlocks(ostream& os, const Design& design) const {
    os << "BLOCK\n";

    for (const auto& b : design.blocks) {
        os << "BLOCK "
            << b.spec.name << " "
            << fmt(b.rect.x) << " "
            << fmt(b.rect.y) << " "
            << fmt(b.rect.w) << " "
            << fmt(b.rect.h) << "\n";
    }

    os << "END BLOCK\n";
}

void OutputWriter::writeChannels(ostream& os, const Design& design) const {
    os << "CHANNEL\n";
    os << "NumChannels " << design.channels.size() << "\n";

    for (const auto& ch : design.channels) {
        os << "CHANNEL "
            << ch.name << " "
            << fmt(ch.rect.x) << " "
            << fmt(ch.rect.y) << " "
            << fmt(ch.rect.w) << " "
            << fmt(ch.rect.h) << "\n";
    }

    os << "END CHANNEL\n";
}

void OutputWriter::writePaths(ostream& os, const Design& design) const {
    os << "PATH\n";
    os << "NumRoutingPattern " << design.routes.size() << "\n";

    for (const auto& p : design.routes) {
        os << "PATH " << p.netCount;

        for (const auto& st : p.steps) {
            os << " " << st.rectName << " " << st.edge;
        }

        os << "\n";
    }

    os << "END PATH\n";
}
