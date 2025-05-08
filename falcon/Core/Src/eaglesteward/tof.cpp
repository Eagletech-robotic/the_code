#include "eaglesteward/tof.hpp"

#include "eaglesteward/state.hpp"
#include "utils/myprintf.hpp"

float tof_filter(const State &state, const float value) {
    if (value < 0.01f || value > 1.9f) {
        return state.filtered_tof_m;
    }
    constexpr float rate = 0.04f;
    return (1.0f - rate) * state.filtered_tof_m + (rate * value);
}

// --- TOF
// Avec la position physique du 20250426
//   > 0.48 il n'y a rien
//   [0.47; 0.36], il y a un gradin collé
//   [0.48; 0.26 mini ; 0.36/0.47] en approche de gradin
//   < 0.26 -> un robot approche

bool isInRange(float min_, float val, float max_) { return min_ < val && val < max_; }

// A autour de 0.5 je détecte le sol à ~ 40 cm
bool isNearSpaceFree(const State &state) { return state.filtered_tof_m > 0.5f; }

// Le robot ou une bordure ou un grand gradin sont proches (<15cm)
// La chose est détecté aussi avec un gradin au contact
bool isBigThingClose(const State &state) { return state.filtered_tof_m < 0.26f; }

// On a ces chiffres si le gradin est là mais aussi si on approche
bool isBleacherPossiblyAtContact(const State &state) { return isInRange(0.32f, state.filtered_tof_m, 0.49); }

// On passe par le mini vers 0.3 puis cela remonte.
bool isPossiblyBleacherApproch(const State &state) { return isInRange(0.28f, state.filtered_tof_m, 0.5); }

bool isPossiblyBleacherApprochMinimum(const State &state) {
    return state.filtered_tof_m < 0.32f;
} // ~15cm, ceux minimum peut descendre à 0.27 mais pas toujours

// Machine d'état qui suit l'approche d'un gradin et en déduit que l'on est au contact
// L'idée est de la faire tourner tout le temps
// si on est à BG_GET_IT, c'est qu'il faut déployer la pelle pour attraper un bleacher
// Trop sensible pour être utilisable
void updateTofStateMachine(State &state) {
    switch (state.tof_state) {
    case TofState::CLEAR_PATH: {
        myprintf("TofState::CLEAR_PATH\n");
        if (!isNearSpaceFree(state)) {
            state.tof_state = TofState::OBJECT_DETECTED;
            return;
        }
    }; break;
    case TofState::OBJECT_DETECTED:
        myprintf("TofState::OBJECT_DETECTED\n");
        if (isNearSpaceFree(state)) {
            state.tof_state = TofState::CLEAR_PATH;
            return;
        }
        if (isPossiblyBleacherApprochMinimum(state)) {
            state.tof_state = TofState::OBJECT_NEARBY;
            return;
        };
        break;
    case TofState::OBJECT_NEARBY:
        myprintf("TofState::OBJECT_NEARBY");
        if (isNearSpaceFree(state)) {
            state.tof_state = TofState::CLEAR_PATH;
            return;
        }
        if (isBleacherPossiblyAtContact(state)) {
            state.tof_state = TofState::BLEACHER_LIFTED;
            return;
        };
        break;
    case TofState::BLEACHER_LIFTED:
        myprintf("TofState::BLEACHER_LIFTED");
        if (!isBleacherPossiblyAtContact(state)) {
            state.tof_state = TofState::CLEAR_PATH;
            return;
        };
        break;
    }
}
