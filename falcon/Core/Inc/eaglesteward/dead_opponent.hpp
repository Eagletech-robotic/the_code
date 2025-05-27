#pragma once

struct DeadOpponent {
    float x{0.0f}, y{0.0f};
    float last_move_timestamp{0.0f};
    float elapsed_time{0.0f}; // Recopie du temps courant pour virer une dépendance au temps dans is_dead

    void tick(float opponent_x, float opponent_y, float elapsed_time);
    bool is_dead() const;
};
