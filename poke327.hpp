#ifndef POKE327_HPP
#define POKE327_HPP

#include <cstdlib>
#include <cassert>
#include <cstdint>
#include <array>
#include "heap.h"  // Include the heap.h header

inline void* custom_malloc(size_t size) {
    void* _tmp;
    assert((_tmp = std::malloc(size)));
    return _tmp;
}

#define malloc(size) custom_malloc(size)

/* Returns true if random float in [0,1] is less than *
 * numerator/denominator.  Uses only integer math.    */
# define rand_under(numerator, denominator) \
  (rand() < ((RAND_MAX / denominator) * numerator))

/* Returns random integer in [min, max]. */
# define rand_range(min, max) ((rand() % (((max) + 1) - (min))) + (min))

# define UNUSED(f) ((void) f)

enum class Dim : int16_t {
    dim_x,
    dim_y,
    num_dims
};

using Pair = std::array<int16_t, static_cast<int>(Dim::num_dims)>;

#define MAP_X              80
#define MAP_Y              21
#define MIN_TREES          10
#define MIN_BOULDERS       10
#define TREE_PROB          95
#define BOULDER_PROB       95
#define WORLD_SIZE         401

#define MIN_TRAINERS     7
#define ADD_TRAINER_PROB 60

#define MOUNTAIN_SYMBOL       '%'
#define BOULDER_SYMBOL        '0'
#define TREE_SYMBOL           '4'
#define FOREST_SYMBOL         '^'
#define GATE_SYMBOL           '#'
#define PATH_SYMBOL           '#'
#define POKEMART_SYMBOL       'M'
#define POKEMON_CENTER_SYMBOL 'C'
#define TALL_GRASS_SYMBOL     ':'
#define SHORT_GRASS_SYMBOL    '.'
#define WATER_SYMBOL          '~'
#define ERROR_SYMBOL          '&'

#define PC_SYMBOL       '@'
#define HIKER_SYMBOL    'h'
#define RIVAL_SYMBOL    'r'
#define EXPLORER_SYMBOL 'e'
#define SENTRY_SYMBOL   's'
#define PACER_SYMBOL    'p'
#define SWIMMER_SYMBOL  'm'
#define WANDERER_SYMBOL 'w'

#define mappair(pair) (m->map[pair[dim_y]][pair[dim_x]])
#define mapxy(x, y) (m->map[y][x])
#define heightpair(pair) (m->height[pair[dim_y]][pair[dim_x]])
#define heightxy(x, y) (m->height[y][x])

enum class TerrainType : uint8_t {
    ter_boulder,
    ter_tree,
    ter_path,
    ter_mart,
    ter_center,
    ter_grass,
    ter_clearing,
    ter_mountain,
    ter_forest,
    ter_water,
    ter_gate,
    num_terrain_types,
    ter_debug
};

class Map {
public:
    TerrainType map[MAP_Y][MAP_X];
    uint8_t height[MAP_Y][MAP_X];
    class Character* cmap[MAP_Y][MAP_X];  // Forward declaration
    heap_t turn;  // Using heap_t from heap.h
    int32_t num_trainers;
    int8_t n, s, e, w;
};

class Npc;  // Forward declaration
class Pc;  // Forward declaration

class Character {
public:
    Npc* npc;
    Pc* pc;
    Pair pos;
    char symbol;
    int next_turn;
    int seq_num;
};

class World {
public:
    Map* world[WORLD_SIZE][WORLD_SIZE];
    Pair cur_idx;
    Map* cur_map;
    int hiker_dist[MAP_Y][MAP_X];
    int rival_dist[MAP_Y][MAP_X];
    Character pc;
    int quit;
    int add_trainer_prob;
    int char_seq_num;
};

extern World world;

extern Pair all_dirs[8];

#define rand_dir(dir) {     \
  int _i = rand() & 0x7;    \
  dir[0] = all_dirs[_i][0]; \
  dir[1] = all_dirs[_i][1]; \
}

class Path {
public:
    heap_node_t* hn;  // Using heap_node_t from heap.h
    uint8_t pos[2];
    uint8_t from[2];
    int32_t cost;
};

int new_map(int teleport);

#endif // POKE327_HPP