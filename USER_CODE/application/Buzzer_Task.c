#include "Buzzer/Buzzer_init.h"
#include "Define.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Buzzer_SoundEffect 已由 Buzzer_init.h 声明为 extern
 * volatile，这里不再重复声明 */

void Buzzer_Task(void *argument) {
  Buzzer_Init();
  for (;;) {
    /*===| 根据需要播放的声音来分别播放音符 |===*/
    switch ((uint8_t)Buzzer_SoundEffect) {
    case Buzzer_SoundEffect_OFF: {
      Buzzer_Set_Tone(P);
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      Buzzer_Stop();
      break;
    }
    case Buzzer_SoundEffect_SystemStart: {
      Buzzer_Start();
      Buzzer_Set_Tone(M1);
      osDelay(200);
      Buzzer_Set_Tone(P);
      osDelay(50);
      Buzzer_Set_Tone(M2);
      osDelay(200);
      Buzzer_Set_Tone(P);
      osDelay(50);
      Buzzer_Set_Tone(M3);
      osDelay(200);
      Buzzer_Set_Tone(P);
      osDelay(50);
      Buzzer_Set_Tone(M2);
      osDelay(200);
      Buzzer_Set_Tone(P);
      osDelay(50);
      Buzzer_Set_Tone(M5);
      osDelay(200);
      Buzzer_Set_Tone(P);
      osDelay(50);
      Buzzer_Set_Tone(M7);
      osDelay(200);
      Buzzer_Set_Tone(P);
      osDelay(50);
      Buzzer_Set_Tone(M6);
      osDelay(500);
      Buzzer_Set_Tone(M5);
      osDelay(250);
      Buzzer_Set_Tone(M6);
      osDelay(750);
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_SuperCap_ON: {
      Buzzer_Start();
      Buzzer_Set_Tone(M1);
      osDelay(200);
      Buzzer_Set_Tone(M3);
      osDelay(200);
      Buzzer_Set_Tone(M5);
      osDelay(200);
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_SuperCap_OFF: {
      Buzzer_Start();
      Buzzer_Set_Tone(M5);
      osDelay(200);
      Buzzer_Set_Tone(M3);
      osDelay(200);
      Buzzer_Set_Tone(M1);
      osDelay(200);
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_Aim_ON: {
      Buzzer_Start();
      Buzzer_Set_Tone(M7);
      osDelay(50);
      Buzzer_Set_Tone(P);
      osDelay(150);
      Buzzer_Set_Tone(M7);
      osDelay(50);
      Buzzer_Set_Tone(P);
      osDelay(150);
      Buzzer_Set_Tone(M7);
      osDelay(50);
      Buzzer_Set_Tone(P);
      osDelay(150);
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_Aim_OFF: {
      Buzzer_Start();
      Buzzer_Set_Tone(M7);
      osDelay(200);
      Buzzer_Set_Tone(M5);
      osDelay(200);
      Buzzer_Set_Tone(M3);
      osDelay(200);
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_Shoot1: {
      Buzzer_Start();
      for (uint8_t i = 0; i < 1; i++) {
        Buzzer_Set_Tone(M5);
        osDelay(100);
        Buzzer_Set_Tone(P);
        osDelay(100);
      }
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_Shoot2: {
      Buzzer_Start();
      for (uint8_t i = 0; i < 2; i++) {
        Buzzer_Set_Tone(M5);
        osDelay(100);
        Buzzer_Set_Tone(P);
        osDelay(100);
      }
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_Shoot3: {
      Buzzer_Start();
      for (uint8_t i = 0; i < 3; i++) {
        Buzzer_Set_Tone(M5);
        osDelay(100);
        Buzzer_Set_Tone(P);
        osDelay(100);
      }
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_Shoot4: {
      Buzzer_Start();
      for (uint8_t i = 0; i < 4; i++) {
        Buzzer_Set_Tone(M5);
        osDelay(100);
        Buzzer_Set_Tone(P);
        osDelay(100);
      }
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_Shoot5: {
      Buzzer_Start();
      for (uint8_t i = 0; i < 5; i++) {
        Buzzer_Set_Tone(M5);
        osDelay(100);
        Buzzer_Set_Tone(P);
        osDelay(100);
      }
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_Shoot6: {
      Buzzer_Start();
      for (uint8_t i = 0; i < 6; i++) {
        Buzzer_Set_Tone(M5);
        osDelay(100);
        Buzzer_Set_Tone(P);
        osDelay(100);
      }
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    case Buzzer_SoundEffect_Shoot7: {
      Buzzer_Start();
      for (uint8_t i = 0; i < 7; i++) {
        Buzzer_Set_Tone(M5);
        osDelay(100);
        Buzzer_Set_Tone(P);
        osDelay(100);
      }
      Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
      break;
    }
    }

    osDelay(BUZZER_TASK_CONTROL_TIME);
  }
}
