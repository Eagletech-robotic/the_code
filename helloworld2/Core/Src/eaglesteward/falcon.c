#include "iot01A/top_driver.h" // c'est la dépendance au interface, rien d'autre n'est authorisé à être inclus depuis iot01A

#ifdef THIBAULT_AUTOPILOT
#include "eaglesteward/thibault.hpp"
#else
#include "eaglesteward/nicolas.h"
#endif

void top_init(config_t *config) {
#ifdef THIBAULT_AUTOPILOT
    thibault_top_init(config);
#else
    nicolas_top_init(config);
#endif
}

void top_step(config_t *config, input_t *input, output_t *output) {
#ifdef THIBAULT_AUTOPILOT
    thibault_top_step(config, input, output);
#else
    nicolas_top_step(config, input, output);
#endif
}
