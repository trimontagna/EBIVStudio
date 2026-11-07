#include "ebiv/export.h"

#include <array>
#include <cstring>
#include <fstream>
#include <sstream>

namespace ebiv {

namespace {

uint32_t crc32(const std::string &data) {
    static const std::array<uint32_t, 256> table = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k)
                c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t[i] = c;
        }
        return t;
    }();
    uint32_t c = 0xFFFFFFFFu;
    for (unsigned char b : data)
        c = table[(c ^ b) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

void put16(std::string &s, uint16_t v) {
    s.push_back(static_cast<char>(v & 0xFF));
    s.push_back(static_cast<char>(v >> 8));
}
void put32(std::string &s, uint32_t v) {
    put16(s, static_cast<uint16_t>(v & 0xFFFF));
    put16(s, static_cast<uint16_t>(v >> 16));
}

// Single-entry ZIP archive without compression (what np.savez writes, minus deflate).
std::string zip_store(const std::string &name, const std::string &data) {
    const uint32_t crc  = crc32(data);
    const uint32_t size = static_cast<uint32_t>(data.size());
    const uint16_t dos_time = 0, dos_date = (1 << 5) | 1; // 1980-01-01
    std::string z;
    // local file header
    put32(z, 0x04034b50);
    put16(z, 20);
    put16(z, 0);
    put16(z, 0);
    put16(z, dos_time);
    put16(z, dos_date);
    put32(z, crc);
    put32(z, size);
    put32(z, size);
    put16(z, static_cast<uint16_t>(name.size()));
    put16(z, 0);
    z += name;
    z += data;
    const uint32_t central_offset = static_cast<uint32_t>(z.size());
    // central directory
    put32(z, 0x02014b50);
    put16(z, 20);
    put16(z, 20);
    put16(z, 0);
    put16(z, 0);
    put16(z, dos_time);
    put16(z, dos_date);
    put32(z, crc);
    put32(z, size);
    put32(z, size);
    put16(z, static_cast<uint16_t>(name.size()));
    put16(z, 0);
    put16(z, 0);
    put16(z, 0);
    put16(z, 0);
    put32(z, 0);
    put32(z, 0); // offset of the local header
    z += name;
    const uint32_t central_size = static_cast<uint32_t>(z.size()) - central_offset;
    // end of central directory
    put32(z, 0x06054b50);
    put16(z, 0);
    put16(z, 0);
    put16(z, 1);
    put16(z, 1);
    put32(z, central_size);
    put32(z, central_offset);
    put16(z, 0);
    return z;
}

bool write_file(const std::filesystem::path &path, const std::string &bytes, std::string *error) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (out)
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    out.close();
    if (!out) {
        if (error)
            *error = "Cannot write " + path.string();
        return false;
    }
    return true;
}

} // namespace

std::string npy_bytes(const double *data, const std::vector<size_t> &shape) {
    std::ostringstream h;
    h << "{'descr': '<f8', 'fortran_order': False, 'shape': (";
    size_t count = 1;
    for (size_t i = 0; i < shape.size(); ++i) {
        h << shape[i] << (shape.size() == 1 || i + 1 < shape.size() ? ", " : "");
        count *= shape[i];
    }
    h << "), }";
    std::string header = h.str();
    // magic (6) + version (2) + length (2) + header, padded with spaces and '\n' to a multiple of 64
    const size_t total = (10 + header.size() + 1 + 63) / 64 * 64;
    header.append(total - 10 - header.size() - 1, ' ');
    header.push_back('\n');
    std::string out("\x93NUMPY\x01\x00", 8);
    put16(out, static_cast<uint16_t>(header.size()));
    out += header;
    out.append(reinterpret_cast<const char *>(data), count * sizeof(double));
    return out;
}

bool save_npz(const std::filesystem::path &path, const Result &r, bool validated, std::string *error) {
    const size_t nt = r.fields.size(), ny = r.grid.ny, nx = r.grid.nx;
    if (nt == 0 || nx == 0 || ny == 0) {
        if (error)
            *error = "No results to save";
        return false;
    }
    std::vector<double> data(nt * ny * nx * 6);
    double *d = data.data();
    for (const Field &f : r.fields) {
        const bool post = validated && f.post.vx.size() == f.v.size();
        for (size_t i = 0; i < f.v.size(); ++i) {
            const Vector &v = f.v[i];
            *d++ = v.x;
            *d++ = r.height - v.y;
            *d++ = post ? f.post.vx[i] : v.vx;
            *d++ = post ? f.post.vy[i] : v.vy;
            *d++ = v.peak;
            *d++ = v.events;
        }
    }
    return write_file(path, zip_store("arr_0.npy", npy_bytes(data.data(), {nt, ny, nx, 6})), error);
}

bool save_csv(const std::filesystem::path &path, const Result &r, std::string *error) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        if (error)
            *error = "Cannot create " + path.string();
        return false;
    }
    const bool phys = r.params.magnification > 0;
    out << "t0_us,x_px,y_px,vx_px_ms,vy_px_ms,vx_raw_px_ms,vy_raw_px_ms,peak,events,valid,outlier";
    if (phys)
        out << ",vx_m_s,vy_m_s";
    out << "\n";
    char line[512];
    for (const Field &f : r.fields) {
        const bool post = f.post.vx.size() == f.v.size();
        for (size_t i = 0; i < f.v.size(); ++i) {
            const Vector &v = f.v[i];
            const double vx = post ? f.post.vx[i] : v.vx, vy = post ? f.post.vy[i] : v.vy;
            int n = std::snprintf(line, sizeof(line), "%lld,%.1f,%.1f,%.6g,%.6g,%.6g,%.6g,%.6g,%.0f,%d,%d",
                                  static_cast<long long>(f.t0), v.x, v.y, vx, vy, v.vx, v.vy, v.peak, v.events,
                                  v.valid ? 1 : 0, post && f.post.outlier[i] ? 1 : 0);
            if (phys)
                n += std::snprintf(line + n, sizeof(line) - n, ",%.6g,%.6g", vx / r.params.magnification,
                                   vy / r.params.magnification);
            out.write(line, n);
            out << "\n";
        }
    }
    out.close();
    if (out.fail()) {
        if (error)
            *error = "Error while writing " + path.string();
        return false;
    }
    return true;
}

} // namespace ebiv
