#pragma once

#include "eaglesteward/shortest_path.hpp"
#include "robotic/eagle_packet.hpp"
#include "utils/game_entities.hpp"
#include "utils/sized_array.hpp"

#include <array>
#include <utility>

class World {
  public:
    World(); // default bleachers & potential field ready

    /** Add default bleachers. */
    void reset();

    /** Replace bleachers with those found in EaglePacket (object_type==0). */
    void reset_from_eagle_packet(const EaglePacket &packet);

    /** (bleacher, distance) in metres; (undefined, 1e9f) if no bleacher left. */
    std::pair<Bleacher, float> closest_bleacher(float x, float y) const;

    /* read‑only access for planners / visualisation */
    const auto &potential() const { return potential_field_; }
    const auto &bleacher_list() const { return bleachers_; }

    void path_to_closest_bleacher(float robotX, float robotY, float opponentX, float opponentY,
                                  SizedArray<Coord, FIELD_WIDTH_SQ * FIELD_HEIGHT_SQ> &outPath);

  private:
    /* data ----------------------------------------------------------- */
    std::array<std::array<float, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> potential_field_{};
    SizedArray<Bleacher, 10> bleachers_;

    /* helpers -------------------------------------------------------- */
    void init_default_bleachers();
    void build_potential_field();
    void add_walls();
    void add_bleachers();
};
