#pragma once

#include <domains/n_puzzle.hpp>

#include <fstream>

namespace parsers
{
    class n_puzzle_parser
    {
    public:
        n_puzzle_parser(std::ifstream& inFile) : inFile(inFile)
        {};

        [[nodiscard]] bool has_next() const
        {
            return !inFile.eof();
        }

        [[nodiscard]] auto get_next() -> search::domains::n_puzzle<search::domains::puzzle_15>
        {
            ++puzzlesMade;
            std::array<char, search::domains::puzzle_15::num_tiles> tiles;
            std::getline(inFile, currentLine);
            std::istringstream iss(currentLine);
            std::string tile;
            for (int i = 0; i < 16; i++)
            {
                iss >> tile;
                tiles[i] = std::stoi(tile);
            }

            // validate puzzle
            std::array<char, 16> test{};
            for(auto t : tiles) ++test[t];

            if(std::ranges::any_of(test, [](auto val){ return val != 1; }))
            {
                std::cerr << "Invalid Puzzle! Number " << std::to_string(puzzlesMade);
                throw;
            }

            return search::domains::n_puzzle<search::domains::puzzle_15>(tiles);
        }

    private:
        int puzzlesMade = 0;
        std::string currentLine;
        std::ifstream& inFile;
    };
}