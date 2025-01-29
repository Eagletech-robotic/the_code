#include "iot01A/top_driver.h"  // c'est la dépendance au interface, rien d'autre n'est authorisé à être inclus depuis iot01A

// #define MODE_THIBAULT 1

#ifdef MODE_THIBAULT
void thibault_top_init(config_t* config);
void thibault_top_step(config_t* config, input_t* input, output_t* output);
#else
void nicolas_top_init(config_t* config);
void nicolas_top_step(config_t* config, input_t* input, output_t* output);
#endif

void top_init(config_t* config) {
#ifdef MODE_THIBAULT
    thibault_top_init(config);
#else
    nicolas_top_init(config);
#endif
}

void top_step(config_t* config, input_t* input, output_t* output) {
#ifdef MODE_THIBAULT
    thibault_top_step(config, input, output);
#else
    nicolas_top_step(config, input, output);
#endif
}
