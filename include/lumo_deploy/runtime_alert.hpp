#pragma once

#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>

namespace lumo_deploy {

inline void PrintAlert(const std::string &title, const std::string &detail, bool failure)
{
    const bool color = ::isatty(STDERR_FILENO) && std::getenv("NO_COLOR") == nullptr;
    std::cout.flush();
    if (color) {
        std::cerr << (failure ? "\033[1;31m" : "\033[1;33m");
    }
    std::cerr << "\n============================================================\n"
              << title << '\n' << detail
              << "\n============================================================\n";
    if (color) {
        std::cerr << "\033[0m";
    }
    std::cerr.flush();
}

// Declared before hardware so the summary is printed after SDK object cleanup.
struct ExitReport {
    bool enabled = true;
    int code = 1;
    std::string reason = "Startup failed; see the error above.";
    bool had_state = false;
    int damping_failures = 0;

    ~ExitReport()
    {
        if (!enabled) {
            return;
        }
        std::string detail = "Reason: " + reason + "\nExit code: " + std::to_string(code);
        if (had_state) {
            detail += "\nExit damping: 20 writes attempted, " + std::to_string(damping_failures) +
                      " local write failures.\n"
                      "Robot execution is NOT confirmed; damping may NOT reach the robot after a disconnection.";
        } else {
            detail += "\nNo robot commands were sent.";
        }
        PrintAlert(code == 0 ? "[STOPPED] 程序已退出 / LUMO CONTROL STOPPED"
                             : "[ERROR] 异常退出 / LUMO CONTROL FAILED", detail, code != 0);
    }
};


}  // namespace lumo_deploy
