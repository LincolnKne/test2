#ifndef CHARACTER_HPP
#define CHARACTER_HPP

#include <cstdint>
#include "poke327.hpp"

constexpr int DIJKSTRA_PATH_MAX = (INT32_MAX / 2);

enum class MovementType : uint8_t {
  Hiker,
  Rival,
  Pace,
  Wander,
  Sentry,
  Explore,
  Swim,
  Pc,
  NumMovementTypes
};

enum class CharacterType : uint8_t {
  Pc,
  Hiker,
  Rival,
  Swimmer,
  Other,
  NumCharacterTypes
};

extern const char* char_type_name[static_cast<int>(CharacterType::NumCharacterTypes)];

extern int32_t move_cost[static_cast<int>(CharacterType::NumCharacterTypes)][static_cast<int>(TerrainType::NumTerrainTypes)]; // Assuming TerrainType is another enum class

class Npc {
public:
  CharacterType ctype;
  MovementType mtype;
  int defeated;
  Pair dir; // Assuming Pair is a C++ class or struct
};

class Pc {
public:
  // Add member variables and methods specific to Pc here
};

// Function declarations
int32_t cmp_char_turns(const void* key, const void* with);
void delete_character(void* v);
void pathfind(Map* m); // Assuming Map is a C++ class or struct

extern void (*move_func[static_cast<int>(MovementType::NumMovementTypes)])(Character*, Pair); // Assuming Character and Pair are C++ classes or structs

int pc_move(char);

#endif // CHARACTER_H
