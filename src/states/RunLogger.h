#pragma once

#include <mc_rtc/logging.h>

#include <sys/stat.h>

#include <chrono>
#include <cstdio>
#include <ctime>
#include <memory>
#include <string>
#include <thread>
#include <vector>

/** Per-run CSV recorder for one-button FSM states (2026-10).
 *
 *  mc_rtc's session log is one multi-GB .bin per mc_kortex session; for the
 *  early-release study we want ONE small, self-describing file PER ROBOT MOTION
 *  that opens directly in pandas. Usage from a state:
 *
 *    begin(dir, stem, columns, reserveRows)  in start()   -> picks the file name
 *    meta("key: value")                     any time     -> '# key: value' header lines
 *    row({...})                             every tick   -> buffered in memory
 *    event(t, "text")                       on events    -> '# event: t text' header lines
 *    finish()                               in teardown() -> written by a background thread
 *
 *  Nothing touches the disk inside the control loop: rows are appended to a
 *  preallocated buffer (no formatting, no I/O) and the whole file is written
 *  by a detached thread after the state ends. The writer owns copies of all
 *  data, so the state object may be destroyed immediately (the FSM deletes the
 *  state right after teardown, on the control thread, so we must not join).
 *
 *  File layout: '#'-prefixed metadata lines, then a CSV header row, then rows.
 *  Read with  pandas.read_csv(path, comment='#').
 */
struct RunLogger
{
  /** Choose the file name (<dir>/<YYYYMMDD_HHMMSS>_<stem>.csv), create the
   *  directory, and reserve the buffer. Returns false (and disables the logger)
   *  if the directory cannot be created. */
  bool begin(const std::string & dir,
             const std::string & stem,
             const std::vector<std::string> & columns,
             size_t reserveRows)
  {
    columns_ = columns;
    meta_.clear();
    events_.clear();
    data_ = std::make_shared<std::vector<double>>();
    data_->reserve(reserveRows * columns_.size());
    active_ = makeDirs(dir);
    if(!active_)
    {
      mc_rtc::log::warning("[RunLogger] cannot create {}; run log disabled", dir);
      return false;
    }
    path_ = dir + "/" + timestamp() + "_" + stem + ".csv";
    return true;
  }

  void meta(const std::string & line)
  {
    meta_.push_back(line);
  }

  void event(double t, const std::string & text)
  {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.3f ", t);
    events_.push_back(std::string(buf) + text);
  }

  /** Append one row; v.size() must equal the number of columns. */
  void row(const std::vector<double> & v)
  {
    if(!active_ || v.size() != columns_.size())
    {
      return;
    }
    data_->insert(data_->end(), v.begin(), v.end());
  }

  size_t rows() const
  {
    return columns_.empty() ? 0 : data_->size() / columns_.size();
  }

  bool active() const
  {
    return active_;
  }

  const std::string & path() const
  {
    return path_;
  }

  /** Hand everything to a background writer. Safe to call once per begin(). */
  void finish()
  {
    if(!active_)
    {
      return;
    }
    active_ = false;
    const std::string path = path_;
    const std::vector<std::string> columns = columns_;
    const std::vector<std::string> meta = meta_;
    const std::vector<std::string> events = events_;
    std::shared_ptr<std::vector<double>> data = data_;
    data_ = std::make_shared<std::vector<double>>();
    std::thread(
        [path, columns, meta, events, data]()
        {
          FILE * f = std::fopen(path.c_str(), "w");
          if(!f)
          {
            mc_rtc::log::error("[RunLogger] cannot open {} for writing", path);
            return;
          }
          for(const auto & m : meta)
          {
            std::fprintf(f, "# %s\n", m.c_str());
          }
          for(const auto & e : events)
          {
            std::fprintf(f, "# event: %s\n", e.c_str());
          }
          for(size_t c = 0; c < columns.size(); ++c)
          {
            std::fprintf(f, "%s%s", c ? "," : "", columns[c].c_str());
          }
          std::fputc('\n', f);
          const size_t ncol = columns.size();
          const size_t nrow = ncol ? data->size() / ncol : 0;
          for(size_t r = 0; r < nrow; ++r)
          {
            const double * v = data->data() + r * ncol;
            for(size_t c = 0; c < ncol; ++c)
            {
              std::fprintf(f, "%s%.7g", c ? "," : "", v[c]);
            }
            std::fputc('\n', f);
          }
          std::fclose(f);
          mc_rtc::log::info("[RunLogger] wrote {} rows to {}", nrow, path);
        })
        .detach();
  }

private:
  static bool makeDirs(const std::string & dir)
  {
    std::string cur;
    for(size_t i = 0; i < dir.size(); ++i)
    {
      cur += dir[i];
      if(dir[i] == '/' || i + 1 == dir.size())
      {
        if(cur.size() > 1 && ::mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST)
        {
          return false;
        }
      }
    }
    struct stat st;
    return ::stat(dir.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
  }

  static std::string timestamp()
  {
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm;
    ::localtime_r(&now, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
  }

  bool active_ = false;
  std::string path_;
  std::vector<std::string> columns_;
  std::vector<std::string> meta_;
  std::vector<std::string> events_;
  std::shared_ptr<std::vector<double>> data_ = std::make_shared<std::vector<double>>();
};
