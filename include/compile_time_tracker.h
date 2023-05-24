#include <string>
#include <chrono>

// DO NOT EDIT
// THIS FILE IS AUTO-GENERATED DURING COMPILER TIME.
// EDITS WILL BE ERASED WHEN PROGRAM IS COMPILED.

// Last compiled on Tuesday, May 23 2023, 07:00PM by joey

auto time_var = std::tm{43,0,19,23,4,123}; 
const auto last_compile_time = std::chrono::system_clock::from_time_t(std::mktime(&time_var));

inline std::string getCompileTime() {
    return "Tuesday, May 23 2023, 07:00PM by joey";
}
std::chrono::minutes minutesSinceLastCompile() {
    auto current_time = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(current_time - (std::chrono::_V2::system_clock::time_point)last_compile_time);
    return elapsed;
}
