#include <string>
#include <chrono>

// DO NOT EDIT
// THIS FILE IS AUTO-GENERATED DURING COMPILER TIME.
// EDITS WILL BE ERASED WHEN PROGRAM IS COMPILED.

// Last compiled on Monday, Jul 01 2024, 01:00PM by tello

auto time_var = std::tm{52,0,13,1,6,124}; 
const auto last_compile_time = std::chrono::system_clock::from_time_t(std::mktime(&time_var));

inline std::string getCompileTime() {
    return "Monday, Jul 01 2024, 01:00PM by tello";
}
std::chrono::minutes minutesSinceLastCompile() {
    auto current_time = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(current_time - (std::chrono::_V2::system_clock::time_point)last_compile_time);
    return elapsed;
}
