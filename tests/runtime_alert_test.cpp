#include "lumo_deploy/runtime_alert.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>

void Expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

int main()
{
    ::setenv("NO_COLOR", "1", 1);
    std::ostringstream captured;
    auto *original = std::cerr.rdbuf(captured.rdbuf());
    {
        lumo_deploy::ExitReport report;
        report.reason = "State/connection failure: missing/stale SDK topic(s): lower/state";
        report.had_state = true;
        report.damping_failures = 20;
    }
    auto text = captured.str();
    std::cerr.rdbuf(original);
    Expect(text.find("[ERROR]") != std::string::npos, "failure banner");
    Expect(text.find("lower/state") != std::string::npos, "original failure preserved");
    Expect(text.find("Exit code: 1") != std::string::npos, "failure status");
    Expect(text.find("20 local write failures") != std::string::npos, "failed damping count");
    Expect(text.find("NOT confirmed") != std::string::npos, "no false robot stop confirmation");
    Expect(text.find('\033') == std::string::npos, "plain output without ANSI escapes");

    captured.str("");
    std::cerr.rdbuf(captured.rdbuf());
    {
        lumo_deploy::ExitReport report;
        report.code = 0;
        report.reason = "SIGINT (Ctrl+C) requested shutdown.";
    }
    text = captured.str();
    std::cerr.rdbuf(original);
    Expect(text.find("[STOPPED]") != std::string::npos && text.find("[ERROR]") == std::string::npos,
           "normal shutdown is distinct from failure");
    Expect(text.find("No robot commands were sent") != std::string::npos, "startup cancellation");

    captured.str("");
    std::cerr.rdbuf(captured.rdbuf());
    {
        lumo_deploy::ExitReport report;
        report.enabled = false;
    }
    std::cerr.rdbuf(original);
    Expect(captured.str().empty(), "help and asset checks do not print a shutdown alarm");
    std::cout << "Runtime alert tests passed\n";
}
