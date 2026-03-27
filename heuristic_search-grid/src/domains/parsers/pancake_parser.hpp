#pragma once

#include <domains/heavy_pancake.hpp>

#include <array>
#include <ranges>
#include <fstream>

namespace parsers
{
    template<std::size_t size>
    class pancake_parser
    {
    public:
        pancake_parser(std::ifstream& inFile) : inFile(inFile)
        {};

        [[nodiscard]] bool has_next() const
        {
            return !inFile.eof();
        }

        [[nodiscard]] auto get_next() -> search::domains::heavy_pancake<size>
        {
            ++stacks_made;
            std::getline(inFile, currentLine);
            std::istringstream iss(currentLine);

            // read as shorts to skip spaces and handle 2-digit numbers, then convert to array of char
            std::array<char, size> pancakes;
            std::ranges::copy(std::views::istream<short>(iss), pancakes.begin());

            // validate puzzle
            std::array<char, size> test{};
            for(auto t : pancakes) ++test[t];

            if(std::ranges::any_of(test, [](auto val){ return val != 1; }))
            {
                std::cerr << "Invalid stack! Number " << std::to_string(stacks_made);
                throw;
            }

            return search::domains::heavy_pancake<size>(pancakes);
        }

    private:
        int stacks_made = 0;
        std::string currentLine;
        std::ifstream& inFile;
    };
}