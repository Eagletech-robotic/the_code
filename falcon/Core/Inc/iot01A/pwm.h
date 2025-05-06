#pragma once
#ifdef __cplusplus
extern "C" {
#endif

void PWMstart();
void PWMset(float ratio2, float ratio15);
void PWMSet_16(float ratio);
void PWMSet_17(float ratio);

#ifdef __cplusplus
}
#endif
