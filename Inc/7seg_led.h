#include "stm32f1xx_hal.h"
#include "main.h"

#define SA_SET HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
#define SA_RESET HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
#define SB_SET HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
#define SB_RESET HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
#define SC_SET HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
#define SC_RESET HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
#define SD_SET HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
#define SD_RESET HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
#define SE_SET HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
#define SE_RESET HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
#define SF_SET HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
#define SF_RESET HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
#define SG_SET HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
#define SG_RESET HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
#define DP_SET HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_RESET);
#define DP_RESET HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);

#define DIG1_SET HAL_GPIO_WritePin(HG1_GPIO_Port, HG1_Pin, GPIO_PIN_RESET), HAL_GPIO_WritePin(HG2_GPIO_Port, HG2_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(HG3_GPIO_Port, HG3_Pin, GPIO_PIN_SET);
#define DIG2_SET HAL_GPIO_WritePin(HG2_GPIO_Port, HG2_Pin, GPIO_PIN_RESET), HAL_GPIO_WritePin(HG1_GPIO_Port, HG1_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(HG3_GPIO_Port, HG3_Pin, GPIO_PIN_SET);
#define DIG3_SET HAL_GPIO_WritePin(HG3_GPIO_Port, HG3_Pin, GPIO_PIN_RESET), HAL_GPIO_WritePin(HG1_GPIO_Port, HG1_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(HG2_GPIO_Port, HG2_Pin, GPIO_PIN_SET);
#define ALL_DIG_OFF HAL_GPIO_WritePin(HG1_GPIO_Port, HG1_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(HG2_GPIO_Port, HG2_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(HG3_GPIO_Port, HG3_Pin, GPIO_PIN_SET);

void seg_print(uint8_t seg);
void led_print(float value);
