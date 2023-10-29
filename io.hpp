#ifndef IO_H
#define IO_H

#include <cstdint>

// Forward declaration of character class
class Character;

// Typedef for pair_t
using Pair = std::array<int16_t, 2>;

namespace IO {
    // Function prototypes
    void initTerminal();
    void resetTerminal();
    void display();
    void handleInput(Pair& dest);
    void queueMessage(const std::string& format, ...);
    void battle(Character* aggressor, Character* defender);
}

#endif // IO_H
