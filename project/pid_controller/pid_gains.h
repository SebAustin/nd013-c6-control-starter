/**
 * Reads the PID gains for a tuning run from a small text file.
 *
 * Rebuilding main.cpp for every change of gain costs VM time, so main.cpp
 * looks for an optional gains file at start-up and falls back to its built-in
 * defaults when the file is absent. File format, one controller per line:
 *
 *     # comments start with a hash
 *     steer    <Kp> <Ki> <Kd>
 *     throttle <Kp> <Ki> <Kd>
 *
 * A controller that is not listed keeps its default gains. A malformed file
 * is an error rather than a silent fallback, so a tuning run can never be
 * recorded under gains it did not actually use.
 *
 * Header-only and free of CARLA, so it is unit tested in
 * tests/pid_gains_test.cpp.
 */

#ifndef PID_GAINS_H
#define PID_GAINS_H

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <istream>
#include <sstream>
#include <string>
#include <vector>

namespace pid_gains {

struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
};

struct ControllerGains {
    PidGains steer;
    PidGains throttle;
};

struct ParseResult {
    bool ok = true;
    // True when the gains were read from a file, false when the file was
    // absent and the defaults were used.
    bool from_file = false;
    ControllerGains gains;
    std::string error;
};

namespace detail {

inline ParseResult Failure(int line_number, const std::string& message) {
    ParseResult result;
    result.ok = false;
    std::ostringstream text;
    text << "gains file line " << line_number << ": " << message;
    result.error = text.str();
    return result;
}

// Parses a whole token as a finite number. Returns false for text such as
// "abc", "1.2x", "nan" or "inf".
inline bool ParseFiniteNumber(const std::string& token, double& value) {
    char* end = nullptr;
    const double parsed = std::strtod(token.c_str(), &end);
    const bool consumed_all = end != token.c_str() && *end == '\0';
    if (!consumed_all || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

inline std::vector<std::string> SplitIntoTokens(const std::string& line) {
    const std::string without_comment = line.substr(0, line.find('#'));
    std::istringstream stream(without_comment);
    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

}  // namespace detail

/**
 * Parses gains from a stream. Controllers that are not listed keep the
 * values given in `defaults`.
 */
inline ParseResult ParseGains(std::istream& input, const ControllerGains& defaults) {
    const std::size_t kTokensPerLine = 4;  // name, Kp, Ki, Kd

    ParseResult result;
    result.gains = defaults;
    result.from_file = true;
    bool seen_steer = false;
    bool seen_throttle = false;

    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const std::vector<std::string> tokens = detail::SplitIntoTokens(line);
        if (tokens.empty()) {
            continue;
        }
        if (tokens.size() != kTokensPerLine) {
            return detail::Failure(line_number, "expected '<steer|throttle> Kp Ki Kd'");
        }

        const std::string& name = tokens[0];
        const bool is_steer = name == "steer";
        const bool is_throttle = name == "throttle";
        if (!is_steer && !is_throttle) {
            return detail::Failure(line_number, "unknown controller '" + name + "'");
        }
        if ((is_steer && seen_steer) || (is_throttle && seen_throttle)) {
            return detail::Failure(line_number, "controller '" + name + "' is listed twice");
        }

        PidGains parsed;
        if (!detail::ParseFiniteNumber(tokens[1], parsed.kp) ||
            !detail::ParseFiniteNumber(tokens[2], parsed.ki) ||
            !detail::ParseFiniteNumber(tokens[3], parsed.kd)) {
            return detail::Failure(line_number, "Kp, Ki and Kd must be finite numbers");
        }

        if (is_steer) {
            result.gains.steer = parsed;
            seen_steer = true;
        } else {
            result.gains.throttle = parsed;
            seen_throttle = true;
        }
    }
    return result;
}

/**
 * Loads gains from the file at `path`. A missing file is not an error: the
 * defaults are returned and `from_file` is false.
 */
inline ParseResult LoadGains(const std::string& path, const ControllerGains& defaults) {
    std::ifstream file(path);
    if (!file.is_open()) {
        ParseResult result;
        result.gains = defaults;
        return result;
    }
    return ParseGains(file, defaults);
}

/**
 * Formats gains in the same layout the file uses, so the output can be
 * logged and also saved as a gains file.
 */
inline std::string FormatGains(const ControllerGains& gains) {
    std::ostringstream text;
    text.precision(10);
    text << "steer " << gains.steer.kp << " " << gains.steer.ki << " " << gains.steer.kd << "\n";
    text << "throttle " << gains.throttle.kp << " " << gains.throttle.ki << " "
         << gains.throttle.kd << "\n";
    return text.str();
}

}  // namespace pid_gains

#endif  // PID_GAINS_H
