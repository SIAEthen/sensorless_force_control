#ifndef SFC_UTILTS_LOGGER_H_
#define SFC_UTILTS_LOGGER_H_

#include <cstddef>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "functionlib/utilts/rotation.h"
#include "functionlib/utilts/vector.h"

namespace sfc {

class Logger {
 public:
  explicit Logger(std::string path)
      : path_(std::move(path)) {}

  bool open() {
    stream_.open(path_, std::ios::out | std::ios::app);
    return stream_.is_open();
  }

  void close() {
    if (stream_.is_open()) {
      stream_.close();
    }
  }

  bool isOpen() const { return stream_.is_open(); }

  void beginFrame(double stamp_sec) {
    current_.clear();
    current_["t"] = {stamp_sec};
  }

  void logScalar(const std::string& name, double value) {
    current_[name] = {value};
  }

  void logVector(const std::string& name, const std::vector<double>& values) {
    current_[name] = values;
  }

  template <std::size_t Size>
  void logVector(const std::string& name, const sfc::Vector<Size>& values) {
    std::vector<double> out(Size);
    for (std::size_t i = 0; i < Size; ++i) {
      out[i] = values(i);
    }
    current_[name] = std::move(out);
  }

  void logVector(const std::string& name, const sfc::Quaternion& q) {
    current_[name] = {q.w, q.x, q.y, q.z};
  }

  void endFrame() {
    if (!stream_.is_open()) {
      return;
    }

    if (!header_written_) {
      writeHeader();
      header_written_ = true;
    }

    stream_ << std::fixed << std::setprecision(6);
    for (std::size_t i = 0; i < columns_.size(); ++i) {
      const auto& key = columns_[i];
      const auto it = current_.find(key);
      if (it == current_.end()) {
        stream_ << 0.0;
      } else {
        const auto& vals = it->second;
        stream_ << vals[col_offsets_[i]];
      }
      if (i + 1 < columns_.size()) {
        stream_ << ",";
      }
    }
    stream_ << "\n";
    stream_.flush();
  }

 private:
  void writeHeader() {
    columns_.clear();
    col_offsets_.clear();

    // Stable column ordering: time first, then alphabetical by key.
    columns_.push_back("t");
    col_offsets_.push_back(0);

    std::vector<std::string> keys;
    keys.reserve(current_.size());
    for (const auto& kv : current_) {
      if (kv.first == "t") {
        continue;
      }
      keys.push_back(kv.first);
    }
    std::sort(keys.begin(), keys.end());

    std::vector<std::string> header;
    header.push_back("t");

    for (const auto& k : keys) {
      const auto& vals = current_.at(k);
      for (std::size_t i = 0; i < vals.size(); ++i) {
        header.push_back(k + "_" + std::to_string(i));
        columns_.push_back(k);
        col_offsets_.push_back(i);
      }
    }

    for (std::size_t i = 0; i < header.size(); ++i) {
      stream_ << header[i];
      if (i + 1 < header.size()) {
        stream_ << ",";
      }
    }
    stream_ << "\n";
  }

  std::string path_;
  std::ofstream stream_;
  bool header_written_{false};

  std::unordered_map<std::string, std::vector<double>> current_;
  std::vector<std::string> columns_;
  std::vector<std::size_t> col_offsets_;
};

}  // namespace sfc

#endif  // SFC_UTILTS_LOGGER_H_
