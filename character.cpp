#include <cstdint>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>
#include <array>

#include "character.hpp"
#include "poke327.hpp"
#include "io.hpp"

using namespace std;

constexpr int32_t PM = DIJKSTRA_PATH_MAX;

// Using std::array for better type safety and automatic size inference
std::array<std::array<int32_t, num_terrain_types>, num_character_types> move_cost = {
    //  boulder,tree,path,mart,center,grass,clearing,mountain,forest,water,gate
    { PM, PM, 10, 10, 10, 20, 10, PM, PM, PM, 10 },
    { PM, PM, 10, 50, 50, 15, 10, 15, 15, PM, PM },
    { PM, PM, 10, 50, 50, 20, 10, PM, PM, PM, PM },
    { PM, PM,  7, PM, PM, PM, PM, PM, PM,  7, PM },
    { PM, PM, 10, 50, 50, 20, 10, PM, PM, PM, PM },
};

// Using std::array for better type safety
std::array<const char*, num_character_types> char_type_name = {
    "PC",
    "Hiker",
    "Rival",
    "Swimmer",
    "Trainer"
};

// Replacing macro with inline function
inline bool is_adjacent(const std::array<int32_t, 2>& pos, terrain_type ter, const map_t& world_map) {
    return (world_map.map[pos[dim_y] - 1][pos[dim_x] - 1] == ter) ||
           (world_map.map[pos[dim_y] - 1][pos[dim_x]    ] == ter) ||
           (world_map.map[pos[dim_y] - 1][pos[dim_x] + 1] == ter) ||
           (world_map.map[pos[dim_y]    ][pos[dim_x] - 1] == ter) ||
           (world_map.map[pos[dim_y]    ][pos[dim_x] + 1] == ter) ||
           (world_map.map[pos[dim_y] + 1][pos[dim_x] - 1] == ter) ||
           (world_map.map[pos[dim_y] + 1][pos[dim_x]    ] == ter) ||
           (world_map.map[pos[dim_y] + 1][pos[dim_x] + 1] == ter);
}

// Function prototype
void pathfind(map_t *m);

uint32_t can_see(map_t *m, character_t *voyeur, character_t *exhibitionist)
{
    // Application of Bresenham's Line Drawing Algorithm.
    // For the purposes of poke327, can swimmers see the PC adjacent to water or on a bridge?
    // v is always a swimmer, and e is always the player character.

    std::array<int16_t, 2> first, second;
    std::array<int16_t, 2> del, f;
    int16_t a, b, c, i;

    first[dim_x] = voyeur->pos[dim_x];
    first[dim_y] = voyeur->pos[dim_y];
    second[dim_x] = exhibitionist->pos[dim_x];
    second[dim_y] = exhibitionist->pos[dim_y];

    if (second[dim_x] > first[dim_x]) {
        del[dim_x] = second[dim_x] - first[dim_x];
        f[dim_x] = 1;
    } else {
        del[dim_x] = first[dim_x] - second[dim_x];
        f[dim_x] = -1;
    }

    if (second[dim_y] > first[dim_y]) {
        del[dim_y] = second[dim_y] - first[dim_y];
        f[dim_y] = 1;
    } else {
        del[dim_y] = first[dim_y] - second[dim_y];
        f[dim_y] = -1;
    }

    if (del[dim_x] > del[dim_y]) {
        a = del[dim_y] + del[dim_y];
        c = a - del[dim_x];
        b = c - del[dim_x];
        for (i = 0; i <= del[dim_x]; i++) {
            if ((mappair(first) != ter_water && mappair(first) != ter_path) && i && (i != del[dim_x])) {
                return 0;
            }
            first[dim_x] += f[dim_x];
            if (c < 0) {
                c += a;
            } else {
                c += b;
                first[dim_y] += f[dim_y];
            }
        }
        return 1;
    } else {
        a = del[dim_x] + del[dim_x];
        c = a - del[dim_y];
        b = c - del[dim_y];
        for (i = 0; i <= del[dim_y]; i++) {
            if ((mappair(first) != ter_water && mappair(first) != ter_path) && i && (i != del[dim_y])) {
                return 0;
            }
            first[dim_y] += f[dim_y];
            if (c < 0) {
                c += a;
            } else {
                c += b;
                first[dim_x] += f[dim_x];
            }
        }
        return 1;
    }

    return 1; // This line is actually redundant, but kept for consistency with original code.
}

static void move_hiker_func(character_t *c, std::array<int, 2>& dest)
{
    int min;
    int base;
    int i;

    base = std::rand() & 0x7;

    dest[dim_x] = c->pos[dim_x];
    dest[dim_y] = c->pos[dim_y];
    min = DIJKSTRA_PATH_MAX;

    for (i = base; i < 8 + base; i++) {
        if ((world.hiker_dist[c->pos[dim_y] + all_dirs[i & 0x7][dim_y]]
                             [c->pos[dim_x] + all_dirs[i & 0x7][dim_x]] <= min) &&
            !world.cur_map->cmap[c->pos[dim_y] + all_dirs[i & 0x7][dim_y]]
                                [c->pos[dim_x] + all_dirs[i & 0x7][dim_x]] &&
            c->pos[dim_x] + all_dirs[i & 0x7][dim_x] != 0 &&
            c->pos[dim_x] + all_dirs[i & 0x7][dim_x] != MAP_X - 1 &&
            c->pos[dim_y] + all_dirs[i & 0x7][dim_y] != 0 &&
            c->pos[dim_y] + all_dirs[i & 0x7][dim_y] != MAP_Y - 1) 
        {
            dest[dim_x] = c->pos[dim_x] + all_dirs[i & 0x7][dim_x];
            dest[dim_y] = c->pos[dim_y] + all_dirs[i & 0x7][dim_y];
            min = world.hiker_dist[dest[dim_y]][dest[dim_x]];
        }
        if (world.hiker_dist[c->pos[dim_y] + all_dirs[i & 0x7][dim_y]]
                            [c->pos[dim_x] + all_dirs[i & 0x7][dim_x]] == 0) 
        {
            io_battle(c, &world.pc);
            break;
        }
    }
}

static void move_rival_func(character_t *c, std::array<int, 2>& dest)
{
    int min;
    int base;
    int i;

    base = std::rand() & 0x7;

    dest[dim_x] = c->pos[dim_x];
    dest[dim_y] = c->pos[dim_y];
    min = DIJKSTRA_PATH_MAX;

    for (i = base; i < 8 + base; i++) {
        if ((world.rival_dist[c->pos[dim_y] + all_dirs[i & 0x7][dim_y]]
                             [c->pos[dim_x] + all_dirs[i & 0x7][dim_x]] < min) &&
            !world.cur_map->cmap[c->pos[dim_y] + all_dirs[i & 0x7][dim_y]]
                                [c->pos[dim_x] + all_dirs[i & 0x7][dim_x]] &&
            c->pos[dim_x] + all_dirs[i & 0x7][dim_x] != 0 &&
            c->pos[dim_x] + all_dirs[i & 0x7][dim_x] != MAP_X - 1 &&
            c->pos[dim_y] + all_dirs[i & 0x7][dim_y] != 0 &&
            c->pos[dim_y] + all_dirs[i & 0x7][dim_y] != MAP_Y - 1) 
        {
            dest[dim_x] = c->pos[dim_x] + all_dirs[i & 0x7][dim_x];
            dest[dim_y] = c->pos[dim_y] + all_dirs[i & 0x7][dim_y];
            min = world.rival_dist[dest[dim_y]][dest[dim_x]];
        }
        if (world.rival_dist[c->pos[dim_y] + all_dirs[i & 0x7][dim_y]]
                            [c->pos[dim_x] + all_dirs[i & 0x7][dim_x]] == 0) 
        {
            io_battle(c, &world.pc);
            break;
        }
    }
}

static void move_pacer_func(character_t *c, std::array<int, 2>& dest)
{
    terrain_type_t t;

    dest[dim_x] = c->pos[dim_x];
    dest[dim_y] = c->pos[dim_y];

    if (!c->npc->defeated &&
        world.cur_map->cmap[c->pos[dim_y] + c->npc->dir[dim_y]]
                           [c->pos[dim_x] + c->npc->dir[dim_x]] == &world.pc) {
        io_battle(c, &world.pc);
        return;
    }

    t = world.cur_map->map[c->pos[dim_y] + c->npc->dir[dim_y]]
                          [c->pos[dim_x] + c->npc->dir[dim_x]];

    if ((t != ter_path && t != ter_grass && t != ter_clearing) ||
        world.cur_map->cmap[c->pos[dim_y] + c->npc->dir[dim_y]]
                           [c->pos[dim_x] + c->npc->dir[dim_x]]) {
        c->npc->dir[dim_x] *= -1;
        c->npc->dir[dim_y] *= -1;
    }

    if ((t == ter_path || t == ter_grass || t == ter_clearing) &&
        !world.cur_map->cmap[c->pos[dim_y] + c->npc->dir[dim_y]]
                            [c->pos[dim_x] + c->npc->dir[dim_x]]) {
        dest[dim_x] = c->pos[dim_x] + c->npc->dir[dim_x];
        dest[dim_y] = c->pos[dim_y] + c->npc->dir[dim_y];
    }
}

static void move_wanderer_func(character_t *c, std::array<int, 2>& dest)
{
    dest[dim_x] = c->pos[dim_x];
    dest[dim_y] = c->pos[dim_y];

    if (!c->npc->defeated &&
        world.cur_map->cmap[c->pos[dim_y] + c->npc->dir[dim_y]]
                           [c->pos[dim_x] + c->npc->dir[dim_x]] == &world.pc) {
        io_battle(c, &world.pc);
        return;
    }

    if ((world.cur_map->map[c->pos[dim_y] + c->npc->dir[dim_y]]
                           [c->pos[dim_x] + c->npc->dir[dim_x]] !=
         world.cur_map->map[c->pos[dim_y]][c->pos[dim_x]]) ||
        world.cur_map->cmap[c->pos[dim_y] + c->npc->dir[dim_y]]
                           [c->pos[dim_x] + c->npc->dir[dim_x]]) {
        rand_dir(c->npc->dir);
    }

    if ((world.cur_map->map[c->pos[dim_y] + c->npc->dir[dim_y]]
                           [c->pos[dim_x] + c->npc->dir[dim_x]] ==
         world.cur_map->map[c->pos[dim_y]][c->pos[dim_x]]) &&
        !world.cur_map->cmap[c->pos[dim_y] + c->npc->dir[dim_y]]
                            [c->pos[dim_x] + c->npc->dir[dim_x]]) {
        dest[dim_x] = c->pos[dim_x] + c->npc->dir[dim_x];
        dest[dim_y] = c->pos[dim_y] + c->npc->dir[dim_y];
    }
}

static void move_sentry_func(character_t *c, std::array<int, 2>& dest)
{
    // Not a bug. Sentries are non-aggro.
    dest[dim_x] = c->pos[dim_x];
    dest[dim_y] = c->pos[dim_y];
}
static void move_explorer_func(character_t* c, std::array<int, 2>& dest)
{
    dest[dim_x] = c->pos[dim_x];
    dest[dim_y] = c->pos[dim_y];

    if (!c->npc->defeated &&
        world.cur_map->cmap[c->pos[dim_y] + c->npc->dir[dim_y]][c->pos[dim_x] + c->npc->dir[dim_x]] == &world.pc) {
        io_battle(c, &world.pc);
        return;
    }

    if ((move_cost[char_other][world.cur_map->map[c->pos[dim_y] + c->npc->dir[dim_y]][c->pos[dim_x] + c->npc->dir[dim_x]]] == DIJKSTRA_PATH_MAX) ||
        world.cur_map->cmap[c->pos[dim_y] + c->npc->dir[dim_y]][c->pos[dim_x] + c->npc->dir[dim_x]]) {
        rand_dir(c->npc->dir);
    }

    if ((move_cost[char_other][world.cur_map->map[c->pos[dim_y] + c->npc->dir[dim_y]][c->pos[dim_x] + c->npc->dir[dim_x]]] != DIJKSTRA_PATH_MAX) &&
        !world.cur_map->cmap[c->pos[dim_y] + c->npc->dir[dim_y]][c->pos[dim_x] + c->npc->dir[dim_x]]) {
        dest[dim_x] = c->pos[dim_x] + c->npc->dir[dim_x];
        dest[dim_y] = c->pos[dim_y] + c->npc->dir[dim_y];
    }
}

static void move_swimmer_func(character_t* c, std::array<int, 2>& dest)
{
    map_t* m = world.cur_map;
    std::array<int, 2> dir;

    dest[dim_x] = c->pos[dim_x];
    dest[dim_y] = c->pos[dim_y];

    if (is_adjacent(world.pc.pos, ter_water) && can_see(world.cur_map, c, &world.pc)) {
        dir[dim_x] = world.pc.pos[dim_x] - c->pos[dim_x];
        if (dir[dim_x]) {
            dir[dim_x] /= std::abs(dir[dim_x]);
        }
        dir[dim_y] = world.pc.pos[dim_y] - c->pos[dim_y];
        if (dir[dim_y]) {
            dir[dim_y] /= std::abs(dir[dim_y]);
        }

        if ((m->map[dest[dim_y] + dir[dim_y]][dest[dim_x] + dir[dim_x]] == ter_water) ||
            ((m->map[dest[dim_y] + dir[dim_y]][dest[dim_x] + dir[dim_x]] == ter_path) &&
             is_adjacent({ dest[dim_x] + dir[dim_x], dest[dim_y] + dir[dim_y] }, ter_water))) {
            dest[dim_x] += dir[dim_x];
            dest[dim_y] += dir[dim_y];
        } else if ((m->map[dest[dim_y]][dest[dim_x] + dir[dim_x]] == ter_water) ||
                   ((m->map[dest[dim_y]][dest[dim_x] + dir[dim_x]] == ter_path) &&
                    is_adjacent({ dest[dim_x] + dir[dim_x], dest[dim_y] }, ter_water))) {
            dest[dim_x] += dir[dim_x];
        } else if ((m->map[dest[dim_y] + dir[dim_y]][dest[dim_x]] == ter_water) ||
                   ((m->map[dest[dim_y] + dir[dim_y]][dest[dim_x]] == ter_path) &&
                    is_adjacent({ dest[dim_x], dest[dim_y] + dir[dim_y] }, ter_water))) {
            dest[dim_y] += dir[dim_y];
        }
    } else {
        dir[dim_x] = c->npc->dir[dim_x];
        dir[dim_y] = c->npc->dir[dim_y];
        if ((m->map[dest[dim_y] + dir[dim_y]][dest[dim_x] + dir[dim_x]] != ter_water) ||
            !((m->map[dest[dim_y] + dir[dim_y]][dest[dim_x] + dir[dim_x]] == ter_path) &&
              is_adjacent({ dest[dim_x] + dir[dim_x], dest[dim_y] + dir[dim_y] }, ter_water))) {
            rand_dir(dir);
        }

        if ((m->map[dest[dim_y] + dir[dim_y]][dest[dim_x] + dir[dim_x]] == ter_water) ||
            ((m->map[dest[dim_y] + dir[dim_y]][dest[dim_x] + dir[dim_x]] == ter_path) &&
             is_adjacent({ dest[dim_x] + dir[dim_x], dest[dim_y] + dir[dim_y] }, ter_water))) {
            dest[dim_x] += dir[dim_x];
            dest[dim_y] += dir[dim_y];
        }
    }

    if (m->cmap[dest[dim_y]][dest[dim_x]]) {
        dest[dim_x] = c->pos[dim_x];
        dest[dim_y] = c->pos[dim_y];
    }
}

static void move_pc_func(character_t* c, std::array<int, 2>& dest)
{
    io_display();
    io_handle_input(dest);
}

std::function<void(character_t *, std::array<int, 2>)> move_func[num_movement_types] = {
  move_hiker_func,
  move_rival_func,
  move_pacer_func,
  move_wanderer_func,
  move_sentry_func,
  move_explorer_func,
  move_swimmer_func,
  move_pc_func,
};

int32_t cmp_char_turns(const void *key, const void *with)
{
  return ((((character_t *) key)->next_turn ==
           ((character_t *) with)->next_turn)  ?
          (((character_t *) key)->seq_num -
           ((character_t *) with)->seq_num)    :
          (((character_t *) key)->next_turn -
           ((character_t *) with)->next_turn));
}

void delete_character(void *v)
{
  if (v == &world.pc) {
    delete world.pc.pc;
  } else {
    delete ((character_t *) v)->npc;
    delete (character_t *) v;
  }
}

static int32_t hiker_cmp(const void *key, const void *with) {
  return (world.hiker_dist[((path_t *) key)->pos[dim_y]]
                          [((path_t *) key)->pos[dim_x]] -
          world.hiker_dist[((path_t *) with)->pos[dim_y]]
                          [((path_t *) with)->pos[dim_x]]);
}

static int32_t rival_cmp(const void *key, const void *with) {
  return (world.rival_dist[((path_t *) key)->pos[dim_y]]
                          [((path_t *) key)->pos[dim_x]] -
          world.rival_dist[((path_t *) with)->pos[dim_y]]
                          [((path_t *) with)->pos[dim_x]]);
}

void pathfind(Map *m) {
  Heap h;
  uint32_t x, y;
  static Path p[MAP_Y][MAP_X], *c;
  static uint32_t initialized = 0;

  if (!initialized) {
    initialized = 1;
    for (y = 0; y < MAP_Y; y++) {
      for (x = 0; x < MAP_X; x++) {
        p[y][x].pos[dim_y] = y;
        p[y][x].pos[dim_x] = x;
      }
    }
  }

  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      world.hiker_dist[y][x] = world.rival_dist[y][x] = DIJKSTRA_PATH_MAX;
    }
  }
  world.hiker_dist[world.pc.pos[dim_y]][world.pc.pos[dim_x]] = 
    world.rival_dist[world.pc.pos[dim_y]][world.pc.pos[dim_x]] = 0;

  h.init(hiker_cmp, nullptr);

  for (y = 1; y < MAP_Y - 1; y++) {
    for (x = 1; x < MAP_X - 1; x++) {
      if (ter_cost(x, y, char_hiker) != DIJKSTRA_PATH_MAX) {
        p[y][x].hn = h.insert(&p[y][x]);
      } else {
        p[y][x].hn = nullptr;
      }
    }
  }

  while ((c = h.removeMin())) {
    c->hn = nullptr;
    if ((p[c->pos[dim_y] - 1][c->pos[dim_x] - 1].hn) &&
        (world.hiker_dist[c->pos[dim_y] - 1][c->pos[dim_x] - 1] >
         world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker))) {
      world.hiker_dist[c->pos[dim_y] - 1][c->pos[dim_x] - 1] =
        world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] - 1][c->pos[dim_x] - 1].hn);
    }
    if ((p[c->pos[dim_y] - 1][c->pos[dim_x]    ].hn) &&
        (world.hiker_dist[c->pos[dim_y] - 1][c->pos[dim_x]    ] >
         world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker))) {
      world.hiker_dist[c->pos[dim_y] - 1][c->pos[dim_x]    ] =
        world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] - 1][c->pos[dim_x]    ].hn);
    }
    if ((p[c->pos[dim_y] - 1][c->pos[dim_x] + 1].hn) &&
        (world.hiker_dist[c->pos[dim_y] - 1][c->pos[dim_x] + 1] >
         world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker))) {
      world.hiker_dist[c->pos[dim_y] - 1][c->pos[dim_x] + 1] =
        world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] - 1][c->pos[dim_x] + 1].hn);
    }
    if ((p[c->pos[dim_y]    ][c->pos[dim_x] - 1].hn) &&
        (world.hiker_dist[c->pos[dim_y]    ][c->pos[dim_x] - 1] >
         world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker))) {
      world.hiker_dist[c->pos[dim_y]    ][c->pos[dim_x] - 1] =
        world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y]    ][c->pos[dim_x] - 1].hn);
    }
    if ((p[c->pos[dim_y]    ][c->pos[dim_x] + 1].hn) &&
        (world.hiker_dist[c->pos[dim_y]    ][c->pos[dim_x] + 1] >
         world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker))) {
      world.hiker_dist[c->pos[dim_y]    ][c->pos[dim_x] + 1] =
        world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y]    ][c->pos[dim_x] + 1].hn);
    }
    if ((p[c->pos[dim_y] + 1][c->pos[dim_x] - 1].hn) &&
        (world.hiker_dist[c->pos[dim_y] + 1][c->pos[dim_x] - 1] >
         world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker))) {
      world.hiker_dist[c->pos[dim_y] + 1][c->pos[dim_x] - 1] =
        world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] + 1][c->pos[dim_x] - 1].hn);
    }
    if ((p[c->pos[dim_y] + 1][c->pos[dim_x]    ].hn) &&
        (world.hiker_dist[c->pos[dim_y] + 1][c->pos[dim_x]    ] >
         world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker))) {
      world.hiker_dist[c->pos[dim_y] + 1][c->pos[dim_x]    ] =
        world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] + 1][c->pos[dim_x]    ].hn);
    }
    if ((p[c->pos[dim_y] + 1][c->pos[dim_x] + 1].hn) &&
        (world.hiker_dist[c->pos[dim_y] + 1][c->pos[dim_x] + 1] >
         world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker))) {
      world.hiker_dist[c->pos[dim_y] + 1][c->pos[dim_x] + 1] =
        world.hiker_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_hiker);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] + 1][c->pos[dim_x] + 1].hn);
    }
  }
  h.deleteHeap();

  h.init(rival_cmp, nullptr);

  for (y = 1; y < MAP_Y - 1; y++) {
    for (x = 1; x < MAP_X - 1; x++) {
      if (ter_cost(x, y, char_rival) != DIJKSTRA_PATH_MAX) {
        p[y][x].hn = h.insert(&p[y][x]);
      } else {
        p[y][x].hn = nullptr;
      }
    }
  }

  while ((c = h.removeMin())) {
    c->hn = nullptr;
    if ((p[c->pos[dim_y] - 1][c->pos[dim_x] - 1].hn) &&
        (world.rival_dist[c->pos[dim_y] - 1][c->pos[dim_x] - 1] >
         world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival))) {
      world.rival_dist[c->pos[dim_y] - 1][c->pos[dim_x] - 1] =
        world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] - 1][c->pos[dim_x] - 1].hn);
    }
    if ((p[c->pos[dim_y] - 1][c->pos[dim_x]    ].hn) &&
        (world.rival_dist[c->pos[dim_y] - 1][c->pos[dim_x]    ] >
         world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival))) {
      world.rival_dist[c->pos[dim_y] - 1][c->pos[dim_x]    ] =
        world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] - 1][c->pos[dim_x]    ].hn);
    }
    if ((p[c->pos[dim_y] - 1][c->pos[dim_x] + 1].hn) &&
        (world.rival_dist[c->pos[dim_y] - 1][c->pos[dim_x] + 1] >
         world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival))) {
      world.rival_dist[c->pos[dim_y] - 1][c->pos[dim_x] + 1] =
        world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] - 1][c->pos[dim_x] + 1].hn);
    }
    if ((p[c->pos[dim_y]    ][c->pos[dim_x] - 1].hn) &&
        (world.rival_dist[c->pos[dim_y]    ][c->pos[dim_x] - 1] >
         world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival))) {
      world.rival_dist[c->pos[dim_y]    ][c->pos[dim_x] - 1] =
        world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y]    ][c->pos[dim_x] - 1].hn);
    }
    if ((p[c->pos[dim_y]    ][c->pos[dim_x] + 1].hn) &&
        (world.rival_dist[c->pos[dim_y]    ][c->pos[dim_x] + 1] >
         world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival))) {
      world.rival_dist[c->pos[dim_y]    ][c->pos[dim_x] + 1] =
        world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y]    ][c->pos[dim_x] + 1].hn);
    }
    if ((p[c->pos[dim_y] + 1][c->pos[dim_x] - 1].hn) &&
        (world.rival_dist[c->pos[dim_y] + 1][c->pos[dim_x] - 1] >
         world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival))) {
      world.rival_dist[c->pos[dim_y] + 1][c->pos[dim_x] - 1] =
        world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] + 1][c->pos[dim_x] - 1].hn);
    }
    if ((p[c->pos[dim_y] + 1][c->pos[dim_x]    ].hn) &&
        (world.rival_dist[c->pos[dim_y] + 1][c->pos[dim_x]    ] >
         world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival))) {
      world.rival_dist[c->pos[dim_y] + 1][c->pos[dim_x]    ] =
        world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] + 1][c->pos[dim_x]    ].hn);
    }
    if ((p[c->pos[dim_y] + 1][c->pos[dim_x] + 1].hn) &&
        (world.rival_dist[c->pos[dim_y] + 1][c->pos[dim_x] + 1] >
         world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
         ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival))) {
      world.rival_dist[c->pos[dim_y] + 1][c->pos[dim_x] + 1] =
        world.rival_dist[c->pos[dim_y]][c->pos[dim_x]] +
        ter_cost(c->pos[dim_x], c->pos[dim_y], char_rival);
      heap_decrease_key_no_replace(&h,
                                   p[c->pos[dim_y] + 1][c->pos[dim_x] + 1].hn);
    }
  }
   h.deleteHeap();
}
