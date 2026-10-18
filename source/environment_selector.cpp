#include "environment_selector.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

std::string selectEnvironment(const std::string& default_scene,
                              std::istream& input,
                              std::ostream& output)
{
    namespace fs = std::filesystem;
    std::vector<fs::path> scenes{default_scene};
    const fs::path directory = fs::path(default_scene).parent_path();
    if (fs::is_directory(directory)) {
        for (const auto& entry : fs::directory_iterator(directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".xml" &&
                !fs::equivalent(entry.path(), scenes.front())) {
                scenes.push_back(entry.path());
            }
        }
        std::sort(scenes.begin() + 1, scenes.end());
    }

    output << "Obstacle environment:\n";
    for (size_t i = 0; i < scenes.size(); ++i) {
        output << "  " << i + 1 << ") " << scenes[i].filename().string()
               << (i == 0 ? " (default)" : "") << '\n';
    }

    while (true) {
        output << "Select a number or enter an XML path [Enter = 1]: " << std::flush;
        std::string choice;
        if (!std::getline(input, choice)) {
            throw std::runtime_error("No environment selection received.");
        }
        const auto first = choice.find_first_not_of(" \t\r");
        choice = first == std::string::npos ? "" :
            choice.substr(first, choice.find_last_not_of(" \t\r") - first + 1);

        fs::path selected;
        if (choice.empty()) {
            selected = scenes.front();
        } else if (choice.find_first_not_of("0123456789") == std::string::npos) {
            try {
                const auto number = std::stoull(choice);
                if (number > 0 && number <= scenes.size()) {
                    selected = scenes[number - 1];
                }
            } catch (const std::exception&) {
                // An oversized number is an invalid menu selection.
            }
        } else {
            selected = choice;
        }

        if (selected.extension() == ".xml" && fs::is_regular_file(selected)) {
            output << "Loading environment: " << selected.string() << '\n';
            return selected.string();
        }
        output << "Please choose a listed number or an existing .xml file.\n";
    }
}
