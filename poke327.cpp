#include <iostream>
#include <cstdint>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <climits>
#include <sys/time.h>
#include <cassert>
#include <unistd.h>
#include <ncurses.h>
#include <queue>
#include <array>
#include <iostream>
#include <chrono> 

#include "heap.h"  // Assuming this is a custom header
#include "poke327.hpp"  // Assuming this is a custom header
#include "character.hpp"  // Assuming this is a custom header
#include "io.hpp"  // Assuming this is a custom header

struct QueueNode {
  int x, y;
  QueueNode* next;
};

// Assuming world_t and Map are defined in "poke327.h"
World world;

std::array<std::array<int16_t, 2>, 8> all_dirs = {{
  {{ -1, -1 }},
  {{ -1,  0 }},
  {{ -1,  1 }},
  {{  0, -1 }},
  {{  0,  1 }},
  {{  1, -1 }},
  {{  1,  0 }},
  {{  1,  1 }},
}};

static int32_t edge_penalty(int8_t x, int8_t y) {
  return (x == 1 || y == 1 || x == MAP_X - 2 || y == MAP_Y - 2) ? 2 : 1;
}

static void dijkstra_path(Map* m, std::array<int16_t, 2> from, std::array<int16_t, 2> to) {
  static std::array<std::array<path_t, MAP_X>, MAP_Y> path;
  static bool initialized = false;
  std::priority_queue<path_t, std::vector<path_t>, decltype(auto) path_cmp = [](const path_t& a, const path_t& b) {
    return a.cost < b.cost;
  }> h(path_cmp);

  if (!initialized) {
    for (int y = 0; y < MAP_Y; ++y) {
      for (int x = 0; x < MAP_X; ++x) {
        path[y][x].pos[static_cast<int>(Dim::dim_y)] = y;
        path[y][x].pos[static_cast<int>(Dim::dim_x)] = x;
      }
    }
    initialized = true;
  }
  
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      path[y][x].cost = INT_MAX;
    }
  }

  path[from[Dim::dim_y]][from[Dim::dim_x]].cost = 0;

  heap_init(&h, path_cmp, NULL);

  for (y = 1; y < MAP_Y - 1; y++) {
    for (x = 1; x < MAP_X - 1; x++) {
      path[y][x].hn = heap_insert(&h, &path[y][x]);
    }
  }

  while ((p = heap_remove_min(&h))) {
    p->hn = NULL;

    if ((p->pos[Dim::dim_y] == to[Dim::dim_y]) && p->pos[Dim::dim_x] == to[Dim::dim_x]) {
      for (x = to[Dim::dim_x], y = to[Dim::dim_y];
           (x != from[Dim::dim_x]) || (y != from[Dim::dim_y]);
           p = &path[y][x], x = p->from[Dim::dim_x], y = p->from[Dim::dim_y]) {
        /* Don't overwrite the gate */
        if (x != to[Dim::dim_x] || y != to[Dim::dim_y]) {
          mapxy(x, y) = TerrainType::ter_path;
          heightxy(x, y) = 0;
        }
      }
      heap_delete(&h);
      return;
    }

    if ((path[p->pos[Dim::dim_y] - 1][p->pos[Dim::dim_x]    ].hn) &&
        (path[p->pos[Dim::dim_y] - 1][p->pos[Dim::dim_x]    ].cost >
         ((p->cost + heightpair(p->pos)) *
          edge_penalty(p->pos[Dim::dim_x], p->pos[Dim::dim_y] - 1)))) {
      path[p->pos[Dim::dim_y] - 1][p->pos[Dim::dim_x]    ].cost =
        ((p->cost + heightpair(p->pos)) *
         edge_penalty(p->pos[Dim::dim_x], p->pos[Dim::dim_y] - 1));
      path[p->pos[Dim::dim_y] - 1][p->pos[Dim::dim_x]    ].from[Dim::dim_y] = p->pos[Dim::dim_y];
      path[p->pos[Dim::dim_y] - 1][p->pos[Dim::dim_x]    ].from[Dim::dim_x] = p->pos[Dim::dim_x];
      heap_decrease_key_no_replace(&h, path[p->pos[Dim::dim_y] - 1]
                                           [p->pos[Dim::dim_x]    ].hn);
    }
    if ((path[p->pos[Dim::dim_y]    ][p->pos[Dim::dim_x] - 1].hn) &&
        (path[p->pos[Dim::dim_y]    ][p->pos[Dim::dim_x] - 1].cost >
         ((p->cost + heightpair(p->pos)) *
          edge_penalty(p->pos[Dim::dim_x] - 1, p->pos[Dim::dim_y])))) {
      path[p->pos[Dim::dim_y]][p->pos[Dim::dim_x] - 1].cost =
        ((p->cost + heightpair(p->pos)) *
         edge_penalty(p->pos[Dim::dim_x] - 1, p->pos[Dim::dim_y]));
      path[p->pos[Dim::dim_y]    ][p->pos[Dim::dim_x] - 1].from[Dim::dim_y] = p->pos[Dim::dim_y];
      path[p->pos[Dim::dim_y]    ][p->pos[Dim::dim_x] - 1].from[Dim::dim_x] = p->pos[Dim::dim_x];
      heap_decrease_key_no_replace(&h, path[p->pos[Dim::dim_y]    ]
                                           [p->pos[Dim::dim_x] - 1].hn);
    }
    if ((path[p->pos[Dim::dim_y]    ][p->pos[Dim::dim_x] + 1].hn) &&
        (path[p->pos[Dim::dim_y]    ][p->pos[Dim::dim_x] + 1].cost >
         ((p->cost + heightpair(p->pos)) *
          edge_penalty(p->pos[Dim::dim_x] + 1, p->pos[Dim::dim_y])))) {
      path[p->pos[Dim::dim_y]][p->pos[Dim::dim_x] + 1].cost =
        ((p->cost + heightpair(p->pos)) *
         edge_penalty(p->pos[Dim::dim_x] + 1, p->pos[Dim::dim_y]));
      path[p->pos[Dim::dim_y]    ][p->pos[Dim::dim_x] + 1].from[Dim::dim_y] = p->pos[Dim::dim_y];
      path[p->pos[Dim::dim_y]    ][p->pos[Dim::dim_x] + 1].from[Dim::dim_x] = p->pos[Dim::dim_x];
      heap_decrease_key_no_replace(&h, path[p->pos[Dim::dim_y]    ]
                                           [p->pos[Dim::dim_x] + 1].hn);
    }
    if ((path[p->pos[Dim::dim_y] + 1][p->pos[Dim::dim_x]    ].hn) &&
        (path[p->pos[Dim::dim_y] + 1][p->pos[Dim::dim_x]    ].cost >
         ((p->cost + heightpair(p->pos)) *
          edge_penalty(p->pos[Dim::dim_x], p->pos[Dim::dim_y] + 1)))) {
      path[p->pos[Dim::dim_y] + 1][p->pos[Dim::dim_x]    ].cost =
        ((p->cost + heightpair(p->pos)) *
         edge_penalty(p->pos[Dim::dim_x], p->pos[Dim::dim_y] + 1));
      path[p->pos[Dim::dim_y] + 1][p->pos[Dim::dim_x]    ].from[Dim::dim_y] = p->pos[Dim::dim_y];
      path[p->pos[Dim::dim_y] + 1][p->pos[Dim::dim_x]    ].from[Dim::dim_x] = p->pos[Dim::dim_x];
      heap_decrease_key_no_replace(&h, path[p->pos[Dim::dim_y] + 1]
                                           [p->pos[Dim::dim_x]    ].hn);
    }
  }
}

static int build_paths(Map* m) {
    std::array<int16_t, 2> from, to;

    if (m->e != -1 && m->w != -1) {
        from[Dim::dim_x] = 1;
        to[Dim::dim_x] = MAP_X - 2;
        from[Dim::dim_y] = m->w;
        to[Dim::dim_y] = m->e;

        dijkstra_path(m, from, to);
    }

    if (m->n != -1 && m->s != -1) {
        from[Dim::dim_y] = 1;
        to[Dim::dim_y] = MAP_Y - 2;
        from[Dim::dim_x] = m->n;
        to[Dim::dim_x] = m->s;

        dijkstra_path(m, from, to);
    }

  if (m->e == -1) {
    if (m->s == -1) {
      from[Dim::dim_x] = 1;
      from[Dim::dim_y] = m->w;
      to[Dim::dim_x] = m->n;
      to[Dim::dim_y] = 1;
    } else {
      from[Dim::dim_x] = 1;
      from[Dim::dim_y] = m->w;
      to[Dim::dim_x] = m->s;
      to[Dim::dim_y] = MAP_Y - 2;
    }

    dijkstra_path(m, from, to);
  }

  if (m->w == -1) {
    if (m->s == -1) {
      from[Dim::dim_x] = MAP_X - 2;
      from[Dim::dim_y] = m->e;
      to[Dim::dim_x] = m->n;
      to[Dim::dim_y] = 1;
    } else {
      from[Dim::dim_x] = MAP_X - 2;
      from[Dim::dim_y] = m->e;
      to[Dim::dim_x] = m->s;
      to[Dim::dim_y] = MAP_Y - 2;
    }

    dijkstra_path(m, from, to);
  }

  if (m->n == -1) {
    if (m->e == -1) {
      from[Dim::dim_x] = 1;
      from[Dim::dim_y] = m->w;
      to[Dim::dim_x] = m->s;
      to[Dim::dim_y] = MAP_Y - 2;
    } else {
      from[Dim::dim_x] = MAP_X - 2;
      from[Dim::dim_y] = m->e;
      to[Dim::dim_x] = m->s;
      to[Dim::dim_y] = MAP_Y - 2;
    }

    dijkstra_path(m, from, to);
  }

  if (m->s == -1) {
    if (m->e == -1) {
      from[Dim::dim_x] = 1;
      from[Dim::dim_y] = m->w;
      to[Dim::dim_x] = m->n;
      to[Dim::dim_y] = 1;
    } else {
      from[Dim::dim_x] = MAP_X - 2;
      from[Dim::dim_y] = m->e;
      to[Dim::dim_x] = m->n;
      to[Dim::dim_y] = 1;
    }

    dijkstra_path(m, from, to);
  }

  return 0;
}

static std::array<std::array<int, 5>, 5> gaussian = {{
    {1, 4, 7, 4, 1},
    {4, 16, 26, 16, 4},
    {7, 26, 41, 26, 7},
    {4, 16, 26, 16, 4},
    {1, 4, 7, 4, 1}
}};

static int smooth_height(Map* m) {
    int32_t i, x, y;
    int32_t s, t, p, q;
    std::unique_ptr<queue_node_t> head, tail, tmp;
    uint8_t height[MAP_Y][MAP_X];

    std::memset(&height, 0, sizeof(height));

    // Seed with some values
    for (i = 1; i < 255; i += 20) {
        do {
            x = std::rand() % MAP_X;
            y = std::rand() % MAP_Y;
        } while (height[y][x]);
        height[y][x] = i;
        if (i == 1) {
            head = std::make_unique<queue_node_t>();
            tail = head.get();
        } else {
            tail->next = std::make_unique<queue_node_t>();
            tail = tail->next.get();
        }
        tail->next = nullptr;
        tail->x = x;
        tail->y = y;
    }

    // Diffuse the values to fill the space
    while (head) {
        x = head->x;
        y = head->y;
        i = height[y][x];

        // ... (the rest of the code is the same, but with malloc replaced)
        if (x - 1 >= 0 && y - 1 >= 0 && !height[y - 1][x - 1]) {
            height[y - 1][x - 1] = i;
            tail->next = std::make_unique<queue_node_t>();
            tail = tail->next.get();
            tail->next = nullptr;
            tail->x = x - 1;
            tail->y = y - 1;
        }

    if (x - 1 >= 0 && !height[y][x - 1]) {
      height[y][x - 1] = i;
tail->next = std::make_unique<queue_node_t>();      tail = tail->next;
      tail->next = NULL;
      tail->x = x - 1;
      tail->y = y;
    }
    if (x - 1 >= 0 && y + 1 < MAP_Y && !height[y + 1][x - 1]) {
      height[y + 1][x - 1] = i;
tail->next = std::make_unique<queue_node_t>();      tail = tail->next;
      tail->next = NULL;
      tail->x = x - 1;
      tail->y = y + 1;
    }
    if (y - 1 >= 0 && !height[y - 1][x]) {
      height[y - 1][x] = i;
tail->next = std::make_unique<queue_node_t>();      tail = tail->next;
      tail->next = NULL;
      tail->x = x;
      tail->y = y - 1;
    }
    if (y + 1 < MAP_Y && !height[y + 1][x]) {
      height[y + 1][x] = i;
tail->next = std::make_unique<queue_node_t>();      tail = tail->next;
      tail->next = NULL;
      tail->x = x;
      tail->y = y + 1;
    }
    if (x + 1 < MAP_X && y - 1 >= 0 && !height[y - 1][x + 1]) {
      height[y - 1][x + 1] = i;
tail->next = std::make_unique<queue_node_t>();      tail = tail->next;
      tail->next = NULL;
      tail->x = x + 1;
      tail->y = y - 1;
    }
    if (x + 1 < MAP_X && !height[y][x + 1]) {
      height[y][x + 1] = i;
tail->next = std::make_unique<queue_node_t>();      tail = tail->next;
      tail->next = NULL;
      tail->x = x + 1;
      tail->y = y;
    }
    if (x + 1 < MAP_X && y + 1 < MAP_Y && !height[y + 1][x + 1]) {
      height[y + 1][x + 1] = i;
tail->next = std::make_unique<queue_node_t>();      tail = tail->next;
      tail->next = NULL;
      tail->x = x + 1;
      tail->y = y + 1;
    }

    tmp = std::move(head);
        head = std::move(tmp->next);
  }

  /* And smooth it a bit with a gaussian convolution */
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      for (s = t = p = 0; p < 5; p++) {
        for (q = 0; q < 5; q++) {
          if (y + (p - 2) >= 0 && y + (p - 2) < MAP_Y &&
              x + (q - 2) >= 0 && x + (q - 2) < MAP_X) {
            s += gaussian[p][q];
            t += height[y + (p - 2)][x + (q - 2)] * gaussian[p][q];
          }
        }
      }
      m->height[y][x] = t / s;
    }
  }
  /* Let's do it again, until it's smooth like Kenny G. */
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      for (s = t = p = 0; p < 5; p++) {
        for (q = 0; q < 5; q++) {
          if (y + (p - 2) >= 0 && y + (p - 2) < MAP_Y &&
              x + (q - 2) >= 0 && x + (q - 2) < MAP_X) {
            s += gaussian[p][q];
            t += height[y + (p - 2)][x + (q - 2)] * gaussian[p][q];
          }
        }
      }
      m->height[y][x] = t / s;
    }
  }

  /*
  out = fopen("diffused.pgm", "w");
  fprintf(out, "P5\n%u %u\n255\n", MAP_X, MAP_Y);
  fwrite(&height, sizeof (height), 1, out);
  fclose(out);

  out = fopen("smoothed.pgm", "w");
  fprintf(out, "P5\n%u %u\n255\n", MAP_X, MAP_Y);
  fwrite(&m->height, sizeof (m->height), 1, out);
  fclose(out);
  */

  return 0;
}

static void find_building_location(Map* m, Pair p) {
  do {
    p[Dim::dim_x] = std::rand() % (MAP_X - 3) + 1;
    p[Dim::dim_y] = std::rand() % (MAP_Y - 3) + 1;

    if ((((mapxy(p[Dim::dim_x] - 1, p[Dim::dim_y]    ) == TerrainType::ter_path)     &&
          (mapxy(p[Dim::dim_x] - 1, p[Dim::dim_y] + 1) == TerrainType::ter_path))    ||
         ((mapxy(p[Dim::dim_x] + 2, p[Dim::dim_y]    ) == TerrainType::ter_path)     &&
          (mapxy(p[Dim::dim_x] + 2, p[Dim::dim_y] + 1) == TerrainType::ter_path))    ||
         ((mapxy(p[Dim::dim_x]    , p[Dim::dim_y] - 1) == TerrainType::ter_path)     &&
          (mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y] - 1) == TerrainType::ter_path))    ||
         ((mapxy(p[Dim::dim_x]    , p[Dim::dim_y] + 2) == TerrainType::ter_path)     &&
          (mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y] + 2) == TerrainType::ter_path)))   &&
        (((mapxy(p[Dim::dim_x]    , p[Dim::dim_y]    ) != ter_mart)     &&
          (mapxy(p[Dim::dim_x]    , p[Dim::dim_y]    ) != ter_center)   &&
          (mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y]    ) != ter_mart)     &&
          (mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y]    ) != ter_center)   &&
          (mapxy(p[Dim::dim_x]    , p[Dim::dim_y] + 1) != ter_mart)     &&
          (mapxy(p[Dim::dim_x]    , p[Dim::dim_y] + 1) != ter_center)   &&
          (mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y] + 1) != ter_mart)     &&
          (mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y] + 1) != ter_center))) &&
        (((mapxy(p[Dim::dim_x]    , p[Dim::dim_y]    ) != TerrainType::ter_path)     &&
          (mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y]    ) != TerrainType::ter_path)     &&
          (mapxy(p[Dim::dim_x]    , p[Dim::dim_y] + 1) != TerrainType::ter_path)     &&
          (mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y] + 1) != TerrainType::ter_path)))) {
          break;
    }
  } while (1);
}

static int place_pokemart(Map* m) {
  Pair p;

  find_building_location(m, p);

  mapxy(p[Dim::dim_x]    , p[Dim::dim_y]    ) = ter_mart;
  mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y]    ) = ter_mart;
  mapxy(p[Dim::dim_x]    , p[Dim::dim_y] + 1) = ter_mart;
  mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y] + 1) = ter_mart;

  return 0;
}

static int place_center(Map* m) {
  Pair p;

  find_building_location(m, p);

  mapxy(p[Dim::dim_x]    , p[Dim::dim_y]    ) = ter_center;
  mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y]    ) = ter_center;
  mapxy(p[Dim::dim_x]    , p[Dim::dim_y] + 1) = ter_center;
  mapxy(p[Dim::dim_x] + 1, p[Dim::dim_y] + 1) = ter_center;

  return 0;
}

/* Chooses tree or boulder for border cell.  Choice is biased by dominance *
 * of neighboring cells.                                                   */
static terrain_type_t border_type(Map* m, int32_t x, int32_t y) {
  int32_t p, q;
  int32_t r, t;
  int32_t miny, minx, maxy, maxx;

  r = t = 0;

  miny = y - 1 >= 0 ? y - 1 : 0;
  maxy = y + 1 <= MAP_Y ? y + 1 : MAP_Y;
  minx = x - 1 >= 0 ? x - 1 : 0;
  maxx = x + 1 <= MAP_X ? x + 1 : MAP_X;

  for (q = miny; q < maxy; q++) {
    for (p = minx; p < maxx; p++) {
      if (q != y || p != x) {
        if (m->map[q][p] == TerrainType::ter_mountain ||
            m->map[q][p] == TerrainType::ter_boulder) {
          r++;
        } else if (m->map[q][p] == ter_forest ||
                   m->map[q][p] == TerrainType::ter_tree) {
          t++;
        }
      }
    }
  }
  
  if (t == r) {
    return std::rand() & 1 ? TerrainType::ter_boulder : TerrainType::ter_tree;
  } else if (t > r) {
    if (std::rand() % 10) {
      return TerrainType::ter_tree;
    } else {
      return TerrainType::ter_boulder;
    }
  } else {
    if (std::rand() % 10) {
      return TerrainType::ter_boulder;
    } else {
      return TerrainType::ter_tree;
    }
  }
}

// The world is global because of its size, so init_world is parameterless
void init_world()
{
  world.quit = 0;
  world.cur_idx[Dim::dim_x] = world.cur_idx[Dim::dim_y] = WORLD_SIZE / 2;
  world.char_seq_num = 0;
  new_map(0); // Assuming new_map is defined elsewhere and compatible with C++
}

void delete_world()
{
  int x, y;

  heap_delete(&world.cur_map->turn); // Assuming heap_delete is defined elsewhere and compatible with C++

  for (y = 0; y < WORLD_SIZE; y++) {
    for (x = 0; x < WORLD_SIZE; x++) {
      if (world.world[y][x]) {
        delete world.world[y][x];
        world.world[y][x] = nullptr;
      }
    }
  }
}

void print_hiker_dist()
{
  int x, y;

  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      if (world.hiker_dist[y][x] == DIJKSTRA_PATH_MAX) {
        std::cout << "   ";
      } else {
        std::printf(" %02d", world.hiker_dist[y][x] % 100);
      }
    }
    std::cout << "\n";
  }
}

void print_rival_dist()
{
  int x, y;

  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      if (world.rival_dist[y][x] == DIJKSTRA_PATH_MAX || world.rival_dist[y][x] < 0) {
        std::cout << "   ";
      } else {
        std::printf(" %02d", world.rival_dist[y][x] % 100);
      }
    }
    std::cout << "\n";
  }
}

Map *get_neighboring_map(Map *current_map, char direction) {
  int cur_x = world.cur_idx[0];
  int cur_y = world.cur_idx[1];

  switch (direction) {
    case 'n':
      cur_y--;
      break;
    case 's':
      cur_y++;
      break;
    case 'e':
      cur_x++;
      break;
    case 'w':
      cur_x--;
      break;
    default:
      return nullptr; // Invalid direction
  }

  // Boundary checks
  if (cur_x < 0 || cur_x >= 100 || cur_y < 0 || cur_y >= 100) {
    return nullptr; // Out of bounds
  }

  return world.world[cur_y][cur_x];
}

void move_Pco_neighboring_map(CharacterType *c, Pair d) {
  int direction;
  if (d[Dim::dim_x] == 0) {
    direction = 'w';
  } else if (d[Dim::dim_x] == MAP_X - 1) {
    direction = 'e';
  } else if (d[Dim::dim_y] == 0) {
    direction = 'n';
  } else if (d[Dim::dim_y] == MAP_Y - 1) {
    direction = 's';
  } else {
    return;
  }

  Map *new_map = get_neighboring_map(world.cur_map, direction);
  world.cur_map = new_map;

  switch (direction) {
    case 'w':
      c->pos[Dim::dim_x] = MAP_X - 1;
      c->pos[Dim::dim_y] = d[Dim::dim_y];
      break;
    case 'e':
      c->pos[Dim::dim_x] = 0;
      c->pos[Dim::dim_y] = d[Dim::dim_y];
      break;
    case 'n':
      c->pos[Dim::dim_x] = d[Dim::dim_x];
      c->pos[Dim::dim_y] = MAP_Y - 1;
      break;
    case 's':
      c->pos[Dim::dim_x] = d[Dim::dim_x];
      c->pos[Dim::dim_y] = 0;
      break;
  }
}


void game_loop()
{
  CharacterType *c;
  Pair d;

  while (!world.quit) {
    c = heap_remove_min(&world.cur_map->turn);
    move_func[c->npc ? c->npc->mtype : move_pc](c, d);
    world.cur_map->cmap[c->pos[Dim::dim_y]][c->pos[Dim::dim_x]] = nullptr;

    if (c->pc && world.cur_map->map[d[Dim::dim_y]][d[Dim::dim_x]] == TerrainType::TerrainType::ter_gate) {
      move_Pco_neighboring_map(c, d);
    } else {
      world.cur_map->cmap[d[Dim::dim_y]][d[Dim::dim_x]] = c;

      if (c->pc) {
        pathfind(world.cur_map);
      }
    }

    c->next_turn += move_cost[c->npc ? c->npc->ctype : Pc]
                             [world.cur_map->map[d[Dim::dim_y]][d[Dim::dim_x]]];
    c->pos[Dim::dim_y] = d[Dim::dim_y];
    c->pos[Dim::dim_x] = d[Dim::dim_x];
    heap_insert(&world.cur_map->turn, c);
  }
}


void usage(char *s)
{
  std::cerr << "Usage: " << s << " [-s|--seed <seed>]\n";
  std::exit(1);
}

int main(int argc, char *argv[])
{
  std::chrono::high_resolution_clock::time_point tp;
  uint32_t seed;
  int long_arg;
  int do_seed;
  int i;

  do_seed = 1;

  if (argc > 1) {
    for (i = 1, long_arg = 0; i < argc; i++, long_arg = 0) {
      if (argv[i][0] == '-') {
        if (argv[i][1] == '-') {
          argv[i]++;
          long_arg = 1;
        }
        switch (argv[i][1]) {
        case 's':
          if ((!long_arg && argv[i][2]) ||
              (long_arg && std::strcmp(argv[i], "-seed")) ||
              argc < ++i + 1 ||
              !std::sscanf(argv[i], "%u", &seed)) {
            usage(argv[0]);
          }
          do_seed = 0;
          break;
        default:
          usage(argv[0]);
        }
      } else {
        usage(argv[0]);
      }
    }
  }

  if (do_seed) {
    tp = std::chrono::high_resolution_clock::now();
    auto duration = tp.time_since_epoch();
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    seed = static_cast<uint32_t>(micros & 0xffffffff);
  }

  std::cout << "Using seed: " << seed << std::endl;
  std::srand(seed);

  IO::initTerminal();
  init_world();



  game_loop();
  delete_world();
  IO::resetTerminal();

  return 0;
}

static int Maperrain(Map *m, int8_t n, int8_t s, int8_t e, int8_t w)
{
  int32_t i, x, y;
  queue_node_t *head, *tail, *tmp;
  int num_grass, num_clearing, num_mountain, num_forest, num_water, num_total;
  TerrainType type;
  int added_current = 0;

  num_grass = std::rand() % 4 + 2;
  num_clearing = std::rand() % 4 + 2;
  num_mountain = std::rand() % 2 + 1;
  num_forest = std::rand() % 2 + 1;
  num_water = std::rand() % 2 + 1;
  num_total = num_grass + num_clearing + num_mountain + num_forest + num_water;

  std::memset(&m->map, 0, sizeof(m->map));

  for (i = 0; i < num_total; i++) {
    do {
      x = std::rand() % MAP_X;
      y = std::rand() % MAP_Y;
    } while (m->map[y][x]);
    if (i == 0) {
      type = ter_grass;
    } else if (i == num_grass) {
      type = ter_clearing;
    } else if (i == num_grass + num_clearing) {
      type = TerrainType::ter_mountain;
    } else if (i == num_grass + num_clearing + num_mountain) {
      type = ter_forest;
    } else if (i == num_grass + num_clearing + num_mountain + num_forest) {
      type = TerrainType::ter_water;
    }
    m->map[y][x] = type;
    if (i == 0) {
      head = tail = new queue_node_t;
    } else {
      tail->next = new queue_node_t;
      tail = tail->next;
    }
    tail->next = nullptr;
    tail->x = x;
    tail->y = y;
  }


  /*
  out = fopen("seeded.pgm", "w");
  fprintf(out, "P5\n%u %u\n255\n", MAP_X, MAP_Y);
  fwrite(&m->map, sizeof (m->map), 1, out);
  fclose(out);
  */

  /* Diffuse the vaules to fill the space */
  while (head) {
    x = head->x;
    y = head->y;
    i = m->map[y][x];
    
    if (x - 1 >= 0 && !m->map[y][x - 1]) {
      if ((rand() % 100) < 80) {
        m->map[y][x - 1] = i;
       tail->next = new queue_node_t;
        tail = tail->next;
        tail->next = NULL;
        tail->x = x - 1;
        tail->y = y;
      } else if (!added_current) {
        added_current = 1;
        m->map[y][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y;
      }
    }

    if (y - 1 >= 0 && !m->map[y - 1][x]) {
      if ((rand() % 100) < 20) {
        m->map[y - 1][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y - 1;
      } else if (!added_current) {
        added_current = 1;
        m->map[y][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y;
      }
    }

    if (y + 1 < MAP_Y && !m->map[y + 1][x]) {
      if ((rand() % 100) < 20) {
        m->map[y + 1][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y + 1;
      } else if (!added_current) {
        added_current = 1;
        m->map[y][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y;
      }
    }

    if (x + 1 < MAP_X && !m->map[y][x + 1]) {
      if ((rand() % 100) < 80) {
        m->map[y][x + 1] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x + 1;
        tail->y = y;
      } else if (!added_current) {
        added_current = 1;
        m->map[y][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y;
      }
    }

    added_current = 0;
    tmp = head;
    head = head->next;
    delete tmp;
  }

  /*
  out = fopen("diffused.pgm", "w");
  fprintf(out, "P5\n%u %u\n255\n", MAP_X, MAP_Y);
  fwrite(&m->map, sizeof (m->map), 1, out);
  fclose(out);
  */
  
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      if (y == 0 || y == MAP_Y - 1 ||
          x == 0 || x == MAP_X - 1) {
        mapxy(x, y) = border_type(m, x, y);
      }
    }
  }

  m->n = n;
  m->s = s;
  m->e = e;
  m->w = w;

  if (n != -1) {
    mapxy(n,         0        ) = TerrainType::ter_gate;
    mapxy(n,         1        ) = TerrainType::ter_gate;
  }
  if (s != -1) {
    mapxy(s,         MAP_Y - 1) = TerrainType::ter_gate;
    mapxy(s,         MAP_Y - 2) = TerrainType::ter_gate;
  }
  if (w != -1) {
    mapxy(0,         w        ) = TerrainType::ter_gate;
    mapxy(1,         w        ) = TerrainType::ter_gate;
  }
  if (e != -1) {
    mapxy(MAP_X - 1, e        ) = TerrainType::ter_gate;
    mapxy(MAP_X - 2, e        ) = TerrainType::ter_gate;
  }

  return 0;
}

static int place_boulders(Map *m)
{
  int i;
  int x, y;

  for (i = 0; i < MIN_BOULDERS || std::rand() % 100 < BOULDER_PROB; i++) {
    y = std::rand() % (MAP_Y - 2) + 1;
    x = std::rand() % (MAP_X - 2) + 1;
    if (m->map[y][x] != TerrainType::ter_forest &&
        m->map[y][x] != TerrainType::ter_path   &&
        m->map[y][x] != TerrainType::ter_gate) {
      m->map[y][x] = TerrainType::ter_boulder;
    }
  }

  return 0;
}

static int place_trees(Map *m)
{
  int i;
  int x, y;
  
  for (i = 0; i < MIN_TREES || std::rand() % 100 < TREE_PROB; i++) {
    y = std::rand() % (MAP_Y - 2) + 1;
    x = std::rand() % (MAP_X - 2) + 1;
    if (m->map[y][x] != TerrainType::ter_mountain &&
        m->map[y][x] != TerrainType::ter_path     &&
        m->map[y][x] != TerrainType::ter_water    &&
        m->map[y][x] != TerrainType::ter_gate) {
      m->map[y][x] = TerrainType::ter_tree;
    }
  }

  return 0;
}

void rand_pos(Pair pos)
{
  pos[Dim::dim_x] = (std::rand() % (MAP_X - 2)) + 1;
  pos[Dim::dim_y] = (std::rand() % (MAP_Y - 2)) + 1;
}

void new_hiker()
{
  Pair pos;
  CharacterType *c;

  do {
    rand_pos(pos);
  } while (world.hiker_dist[pos[Dim::dim_y]][pos[Dim::dim_x]] == DIJKSTRA_PATH_MAX ||
           world.cur_map->cmap[pos[Dim::dim_y]][pos[Dim::dim_x]]                   ||
           pos[Dim::dim_x] < 3 || pos[Dim::dim_x] > MAP_X - 4                      ||
           pos[Dim::dim_y] < 3 || pos[Dim::dim_y] > MAP_Y - 4);

  world.cur_map->cmap[pos[Dim::dim_y]][pos[Dim::dim_x]] = c = new CharacterType;
  c->npc = new Npc;
  c->pos[Dim::dim_y] = pos[Dim::dim_y];
  c->pos[Dim::dim_x] = pos[Dim::dim_x];
  c->npc->ctype = CharacterType::Hiker;
  c->npc->mtype = MovementType::Hiker;
  c->npc->dir[Dim::dim_x] = 0;
  c->npc->dir[Dim::dim_y] = 0;
  c->npc->defeated = 0;
  c->pc = nullptr;
  c->symbol = HIKER_SYMBOL;
  c->next_turn = 0;
  c->seq_num = world.char_seq_num++;
  heap_insert(&world.cur_map->turn, c);
}

void new_rival()
{
  Pair pos;
  CharacterType *c;

  do {
    rand_pos(pos);
  } while (world.rival_dist[pos[Dim::dim_y]][pos[Dim::dim_x]] == DIJKSTRA_PATH_MAX ||
           world.rival_dist[pos[Dim::dim_y]][pos[Dim::dim_x]] < 0                  ||
           world.cur_map->cmap[pos[Dim::dim_y]][pos[Dim::dim_x]]                   ||
           pos[Dim::dim_x] < 3 || pos[Dim::dim_x] > MAP_X - 4                      ||
           pos[Dim::dim_y] < 3 || pos[Dim::dim_y] > MAP_Y - 4);

  world.cur_map->cmap[pos[Dim::dim_y]][pos[Dim::dim_x]] = c = new CharacterType;
  c->npc = new Npc;
  c->pos[Dim::dim_y] = pos[Dim::dim_y];
  c->pos[Dim::dim_x] = pos[Dim::dim_x];
  c->npc->ctype = CharacterType::Rival;
  c->npc->mtype = MovementType::Rival;
  c->npc->dir[Dim::dim_x] = 0;
  c->npc->dir[Dim::dim_y] = 0;
  c->npc->defeated = 0;
  c->pc = nullptr;
  c->symbol = RIVAL_SYMBOL;
  c->next_turn = 0;
  c->seq_num = world.char_seq_num++;
  heap_insert(&world.cur_map->turn, c);
}

void new_swimmer()
{
  Pair pos;
  CharacterType *c;

  do {
    rand_pos(pos);
  } while (world.cur_map->map[pos[Dim::dim_y]][pos[Dim::dim_x]] != TerrainType::ter_water ||
           world.cur_map->cmap[pos[Dim::dim_y]][pos[Dim::dim_x]]);

  world.cur_map->cmap[pos[Dim::dim_y]][pos[Dim::dim_x]] = c = new CharacterType;
  c->npc = new Npc;
  c->pos[Dim::dim_y] = pos[Dim::dim_y];
  c->pos[Dim::dim_x] = pos[Dim::dim_x];
  c->npc->ctype = CharacterType::Swimmer;
  c->npc->mtype = MovementType::Swim;
  rand_dir(c->npc->dir);
  c->npc->defeated = 0;
  c->pc = nullptr;
  c->symbol = SWIMMER_SYMBOL;
  c->next_turn = 0;
  c->seq_num = world.char_seq_num++;
  heap_insert(&world.cur_map->turn, c);
}

void new_char_other()
{
  Pair pos;
  CharacterType *c;

  do {
    rand_pos(pos);
  } while (world.rival_dist[pos[Dim::dim_y]][pos[Dim::dim_x]] == DIJKSTRA_PATH_MAX ||
           world.rival_dist[pos[Dim::dim_y]][pos[Dim::dim_x]] < 0                  ||
           world.cur_map->cmap[pos[Dim::dim_y]][pos[Dim::dim_x]]                   ||
           pos[Dim::dim_x] < 3 || pos[Dim::dim_x] > MAP_X - 4                      ||
           pos[Dim::dim_y] < 3 || pos[Dim::dim_y] > MAP_Y - 4);

  world.cur_map->cmap[pos[Dim::dim_y]][pos[Dim::dim_x]] = c = new CharacterType;
  c->npc = new Npc;
  c->pos[Dim::dim_y] = pos[Dim::dim_y];
  c->pos[Dim::dim_x] = pos[Dim::dim_x];
  c->npc->ctype = char_other;
  switch (std::rand() % 4) {
  case 0:
    c->npc->mtype = MovementType::Pace;
    c->symbol = PACER_SYMBOL;
    break;
  case 1:
    c->npc->mtype = MovementType::Wander;
    c->symbol = WANDERER_SYMBOL;
    break;
  case 2:
    c->npc->mtype = MovementType::Sentry;
    c->symbol = SENTRY_SYMBOL;
    break;
  case 3:
    c->npc->mtype = MovementType::Explore;
    c->symbol = EXPLORER_SYMBOL;
    break;
  }
  rand_dir(c->npc->dir);
  c->npc->defeated = 0;
  c->pc = nullptr;
  c->next_turn = 0;
  c->seq_num = world.char_seq_num++;
  heap_insert(&world.cur_map->turn, c);
}

void place_characters()
{
  world.cur_map->num_trainers = 3;

  // Always place a hiker and a rival, then place a random number of others
  new_hiker();
  new_rival();
  new_swimmer();
  do {
    // higher probability of non- hikers and rivals
    switch(std::rand() % 10) {
    case 0:
      new_hiker();
      break;
    case 1:
      new_rival();
      break;
    case 2:
      new_swimmer();
      break;
    default:
      new_char_other();
      break;
    }
  } while (++world.cur_map->num_trainers < MIN_TRAINERS ||
           ((std::rand() % 100) < ADD_TRAINER_PROB));
}

void init_pc()
{
  int x, y;

  do {
    x = std::rand() % (MAP_X - 2) + 1;
    y = std::rand() % (MAP_Y - 2) + 1;
  } while (world.cur_map->map[y][x] != TerrainType::ter_path);

  world.pc.pos[Dim::dim_x] = x;
  world.pc.pos[Dim::dim_y] = y;
  world.pc.symbol = PC_SYMBOL;
  world.pc.pc = new Pc;
  world.pc.npc = nullptr;

  world.cur_map->cmap[y][x] = &world.pc;
  world.pc.next_turn = 0;

  world.pc.seq_num = world.char_seq_num++;

  heap_insert(&world.cur_map->turn, &world.pc);
}


void place_pc()
{
  CharacterType *c;

  if (world.pc.pos[Dim::dim_x] == 1) {
    world.pc.pos[Dim::dim_x] = MAP_X - 2;
  } else if (world.pc.pos[Dim::dim_x] == MAP_X - 2) {
    world.pc.pos[Dim::dim_x] = 1;
  } else if (world.pc.pos[Dim::dim_y] == 1) {
    world.pc.pos[Dim::dim_y] = MAP_Y - 2;
  } else if (world.pc.pos[Dim::dim_y] == MAP_Y - 2) {
    world.pc.pos[Dim::dim_y] = 1;
  }

  world.cur_map->cmap[world.pc.pos[Dim::dim_y]][world.pc.pos[Dim::dim_x]] = &world.pc;

  if ((c = heap_peek_min(&world.cur_map->turn))) {
    world.pc.next_turn = c->next_turn;
  } else {
    world.pc.next_turn = 0;
  }
}

// New map expects cur_idx to refer to the index to be generated.  If that
// map has already been generated then the only thing this does is set
// cur_map.
int new_map(int teleport)
{
  int d, p;
  int e, w, n, s;
  int x, y;

  if (world.world[world.cur_idx[Dim::dim_y]][world.cur_idx[Dim::dim_x]]) {
    world.cur_map = world.world[world.cur_idx[Dim::dim_y]][world.cur_idx[Dim::dim_x]];
    place_pc();

    return 0;
  }

  world.cur_map = world.world[world.cur_idx[Dim::dim_y]][world.cur_idx[Dim::dim_x]] = new Map;

  smooth_height(world.cur_map);
  
  if (!world.cur_idx[Dim::dim_y]) {
    n = -1;
  } else if (world.world[world.cur_idx[Dim::dim_y] - 1][world.cur_idx[Dim::dim_x]]) {
    n = world.world[world.cur_idx[Dim::dim_y] - 1][world.cur_idx[Dim::dim_x]]->s;
  } else {
    n = 3 + rand() % (MAP_X - 6);
  }
  if (world.cur_idx[Dim::dim_y] == WORLD_SIZE - 1) {
    s = -1;
  } else if (world.world[world.cur_idx[Dim::dim_y] + 1][world.cur_idx[Dim::dim_x]]) {
    s = world.world[world.cur_idx[Dim::dim_y] + 1][world.cur_idx[Dim::dim_x]]->n;
  } else  {
    s = 3 + rand() % (MAP_X - 6);
  }
  if (!world.cur_idx[Dim::dim_x]) {
    w = -1;
  } else if (world.world[world.cur_idx[Dim::dim_y]][world.cur_idx[Dim::dim_x] - 1]) {
    w = world.world[world.cur_idx[Dim::dim_y]][world.cur_idx[Dim::dim_x] - 1]->e;
  } else {
    w = 3 + rand() % (MAP_Y - 6);
  }
  if (world.cur_idx[Dim::dim_x] == WORLD_SIZE - 1) {
    e = -1;
  } else if (world.world[world.cur_idx[Dim::dim_y]][world.cur_idx[Dim::dim_x] + 1]) {
    e = world.world[world.cur_idx[Dim::dim_y]][world.cur_idx[Dim::dim_x] + 1]->w;
  } else {
    e = 3 + rand() % (MAP_Y - 6);
  }
  
  Maperrain(world.cur_map, n, s, e, w);
     
  place_boulders(world.cur_map);
  place_trees(world.cur_map);
  build_paths(world.cur_map);
  d = (abs(world.cur_idx[Dim::dim_x] - (WORLD_SIZE / 2)) +
       abs(world.cur_idx[Dim::dim_y] - (WORLD_SIZE / 2)));
  p = d > 200 ? 5 : (50 - ((45 * d) / 200));
  //  printf("d=%d, p=%d\n", d, p);
  if ((rand() % 100) < p || !d) {
    place_pokemart(world.cur_map);
  }
  if ((rand() % 100) < p || !d) {
    place_center(world.cur_map);
  }

  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      world.cur_map->cmap[y][x] = NULL;
    }
  }

  heap_init(&world.cur_map->turn, cmp_char_turns, delete_character);

  if ((world.cur_idx[Dim::dim_x] == WORLD_SIZE / 2) &&
      (world.cur_idx[Dim::dim_y] == WORLD_SIZE / 2)) {
    init_pc();
  } else {
    place_pc();
  }

  pathfind(world.cur_map);
  if (teleport) {
    do {
      world.cur_map->cmap[world.pc.pos[Dim::dim_y]][world.pc.pos[Dim::dim_x]] = NULL;
      world.pc.pos[Dim::dim_x] = rand_range(1, MAP_X - 2);
      world.pc.pos[Dim::dim_y] = rand_range(1, MAP_Y - 2);
    } while (world.cur_map->cmap[world.pc.pos[Dim::dim_y]][world.pc.pos[Dim::dim_x]] ||
             (move_cost[Pc][world.cur_map->map[world.pc.pos[Dim::dim_y]]
                                                   [world.pc.pos[Dim::dim_x]]] ==
              DIJKSTRA_PATH_MAX)                                           ||
             world.rival_dist[world.pc.pos[Dim::dim_y]][world.pc.pos[Dim::dim_x]] < 0);
    world.cur_map->cmap[world.pc.pos[Dim::dim_y]][world.pc.pos[Dim::dim_x]] = &world.pc;
    pathfind(world.cur_map);
  }
  
  place_characters();

  return 0;
}
