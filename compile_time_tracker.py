import os
import datetime
import getpass

# Open the compile_time_tracker.h file in write mode to erase its contents
with open(os.path.join("include", "compile_time_tracker.h"), "w") as f:
    # Include the necessary headers
    f.write("#include <string>\n")
    f.write("#include <chrono>\n\n")
    # Get the current date and time
    now = datetime.datetime.now()
    now_str = now.strftime("%Y-%m-%d %H:%M:%S")
    # Get the name of the current user
    user = getpass.getuser()
    # Format the date, time, and user as a string
    datetime_str = now.strftime("%A, %b %d %Y, %I:%M%p")
    compile_info_str = f"{datetime_str} by {user}"
    # Write the formatted string to the file
    f.write(f"// DO NOT EDIT\n// THIS FILE IS AUTO-GENERATED DURING COMPILER TIME.\n// EDITS WILL BE ERASED WHEN PROGRAM IS COMPILED.\n\n")
    f.write(f"// Last compiled on {compile_info_str}\n\n")
    # write a variable containing the compile time
    f.write(f"auto time_var = std::tm{{{now.second},{now.minute},{now.hour},{now.day},{now.month-1},{now.year-1900}}}; \n")
    f.write("const auto last_compile_time = std::chrono::system_clock::from_time_t(std::mktime(&time_var));\n\n")
    # Define a C++ function that returns the formatted string
    f.write("inline std::string getCompileTime() {\n")
    f.write(f"    return \"{compile_info_str}\";\n")
    f.write("}\n")
    # Define a C++ function that returns the elapsed time since the Python script was run
    f.write("std::chrono::minutes minutesSinceLastCompile() {\n")
    f.write("    auto current_time = std::chrono::system_clock::now();\n")
    f.write("    auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(current_time - (std::chrono::_V2::system_clock::time_point)last_compile_time);\n")
    f.write("    return elapsed;\n")
    f.write("}\n")