/*
 * DeadOpponent.hpp
 *
 *  Created on: May 27, 2025
 *      Author: nboulay
 */
#pragma once

struct DeadOpponent {

    float x{0.0f}, y{0.0f}, theta{0.0f};
    float last_move_timestamp{0.0f};
    float elapsed_time; // Recopie du temps courant pour virer une dépendance au temps dans isReallyDead

    void tick(float opponent_x, float opponent_y, float opponent_theta, float elasped_time);
    bool isReallyDead();
};
