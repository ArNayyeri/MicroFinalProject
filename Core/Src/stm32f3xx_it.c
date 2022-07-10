/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int state_7segment = 0;
int score = 1;
bool is_10_score = true;
int difficulty = 0;
unsigned char data;
unsigned char buffer[100] = "";
int position = 0;
char name[100] = "Doodler";
unsigned char current_char[2] = "1\0";
int char_pos = 0;
int frq_counter = 0;
typedef unsigned char byte;


// char bytes:

// indx = 0
byte plat_char[] = {
        0x00,
        0x1C,
        0x0C,
        0x0C,
        0x0C,
        0x0C,
        0x1C,
        0x00
};

// indx = 1
byte broke_plat_char[] = {
        0x00,
        0x1C,
        0x0C,
        0x04,
        0x00,
        0x0C,
        0x1C,
        0x00
};

// indx = 2
byte spring_plat_char[] = {
        0x00,
        0x1C,
        0x04,
        0x14,
        0x0C,
        0x04,
        0x1C,
        0x00
};

// indx = 3
byte black_hole_char[] = {
        0x00,
        0x0E,
        0x1F,
        0x1F,
        0x1F,
        0x1F,
        0x0E,
        0x00
};

// indx = 4
byte alien_char[] = {
        0x11,
        0x0A,
        0x04,
        0x1F,
        0x1F,
        0x04,
        0x0A,
        0x11
};

// indx = 5
byte player_char[] = {
        0x00,
        0x00,
        0x17,
        0x17,
        0x1E,
        0x0E,
        0x00,
        0x00
};


void send_UART(char *string);

void show_menu();

void PWM_Start();

void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume);

void end_game();

void setup_melody(int melody[], int size_arr);

void fill_new_row();

void shift_map();

void fill_buffer(int diff);

// map[][] values:
const int blank = 0;
const int plat = 1;
const int broke_plat = 2;
const int spring_plat = 3;
const int black_hole = 4;
const int alien = 5;

bool wasFalling = true;
const int player_initial_x = 1;
const int player_initial_y = 18;
int old_x = -1;
int old_y = -1;
int curr_y = -1;
int curr_x = -1;
int jump = 0;
bool right_flag = false;
bool left_flag = false;
bool alien_collision = false;

int map[20][4];
int old_map[20][4];

const int row_buffer_size = 8;
int row_buffer[8][4];
int buffer_content = 0;
const int num_of_elements = 6;
int ps[6][2];

int bullets_pos[20][2];
int old_bullets_pos[20][2];
bool shot_fired = false;
const int in_active_bullet = -5;

const int map_m = 20;
const int map_n = 4;
const int half_board = 9;
bool god_mode = false;

void copy_map(int old_map[map_m][map_n], int map[map_m][map_n]) {
    old_x = curr_x;
    old_y = curr_y;
    for (int i = 0; i < map_m; i++) {
        for (int j = 0; j < map_n; j++) {
            old_map[i][j] = map[i][j];
        }
    }
}

void copy_bullets_pos(int old[20][2], int curr[20][2]) {
    for (int i = 0; i < 20; i++) {
        old[i][0] = curr[i][0];
        old[i][1] = curr[i][1];
    }
}

void init_lcd();

void update_lcd();

void reset_port_7segment() {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);

}

int BCDConversion(int n) {
    // 0 = 0000
    // 1 = 0001
    // 2 = 0010
    // 3 = 0011
    // 4 = 0100
    // 5 = 0101
    // 6 = 0110
    // 7 = 0111
    // 8 = 1000
    // 9 = 1001
    switch (n) {
        case 0:
            return 0;
        case 1:
            return 1;
        case 2:
            return 10;
        case 3:
            return 11;
        case 4:
            return 100;
        case 5:
            return 101;
        case 6:
            return 110;
        case 7:
            return 111;
        case 8:
            return 1000;
        case 9:
            return 1001;
    }
    return 0;
}

void set_number(int number, int pin) {
    int bcd = BCDConversion(number);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, bcd % 10);
    bcd /= 10;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, bcd % 10);
    bcd /= 10;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, bcd % 10);
    bcd /= 10;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, bcd % 10);
    bcd /= 10;

    switch (pin) {
        case 4:
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
            break;
    }
}

void show_number(int n, int pin) {
    reset_port_7segment();
    int num = (int) (n / pow(10, pin)) % 10;
    set_number(num, pin + 1);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (data == '\n') {
        strcpy(name, buffer);
        strcpy(buffer, "");
        position = 0;
    } else {
        buffer[position] = data;
        buffer[position + 1] = '\0';
        position = position + 1;
    }
}


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */
extern const int player_index, stair_index, broke_stair_index, coil_index, status_start, status_menu,
        status_info, status_game, status_end, status_select_name;
extern int status;
extern RTC_TimeTypeDef rTime;
extern RTC_DateTypeDef rDate;
extern RTC_HandleTypeDef hrtc;
extern int starwars_sound[176];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void) {
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) {
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void) {
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void) {
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void) {
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void) {
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void) {
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void) {
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void) {
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void) {
    /* USER CODE BEGIN EXTI0_IRQn 0 */
    if (status == status_end) {
        HAL_NVIC_SystemReset();
    }
    /* USER CODE END EXTI0_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    /* USER CODE BEGIN EXTI0_IRQn 1 */
    if (status == status_start) {
        show_menu();
    }
    /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void) {
    /* USER CODE BEGIN EXTI3_IRQn 0 */
    if (status == status_end) {
        HAL_NVIC_SystemReset();
    }
    /* USER CODE END EXTI3_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    /* USER CODE BEGIN EXTI3_IRQn 1 */

    /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void) {
    /* USER CODE BEGIN EXTI4_IRQn 0 */
    if (status == status_end) {
        HAL_NVIC_SystemReset();
    }
    /* USER CODE END EXTI4_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
    /* USER CODE BEGIN EXTI4_IRQn 1 */

    /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void) {
    /* USER CODE BEGIN EXTI9_5_IRQn 0 */
    if (status == status_end) {
        HAL_NVIC_SystemReset();
    }
    /* USER CODE END EXTI9_5_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    /* USER CODE BEGIN EXTI9_5_IRQn 1 */

    /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void) {
    /* USER CODE BEGIN TIM2_IRQn 0 */

    /* USER CODE END TIM2_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2);
    /* USER CODE BEGIN TIM2_IRQn 1 */

    /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void) {
    /* USER CODE BEGIN TIM3_IRQn 0 */

    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */
    PWM_Change_Tone(700, 0);
    if (!alien_collision) {
        // shot fired gets checked regardless of frq_counter
        if (shot_fired) {
            shot_fired = false;
            // find a non-active bullet:
            // (non active bullets have [-1][-1] as their position)
            for (int i = 0; i < 20; i++) {
                if (bullets_pos[i][0] == in_active_bullet && bullets_pos[i][1] == in_active_bullet) {
                    // active the non active bullet
                    bullets_pos[i][0] = curr_y; // ???????????????? curr_y -1 ???????????????
                    bullets_pos[i][1] = curr_x;
                    break;
                }
            }
        }


        // bullets gets updated regardless of value of frq_counter
        // update bullets position:
        for (int i = 0; i < 20; i++) {
            // if bullet is active, update position
            if (bullets_pos[i][0] != in_active_bullet && bullets_pos[i][1] != in_active_bullet) {
                bullets_pos[i][0]--;
            }
        }

        // check colision
        for (int curr_bullet = 0; curr_bullet < 20; curr_bullet++) {
            int by = bullets_pos[curr_bullet][0];
            int bx = bullets_pos[curr_bullet][1];

            if (by != in_active_bullet && bx != in_active_bullet) {
                if (map[by][bx] == alien) {
                    // alien on [by][bx] is dead
                    map[by][bx] = blank;
                    // bullet in active
                    bullets_pos[curr_bullet][0] = in_active_bullet;
                    bullets_pos[curr_bullet][1] = in_active_bullet;
                }
            }

        }


        if (!frq_counter) {
            if (right_flag == true) {
                right_flag = false;
                int new_x = curr_x;
                new_x--;
                if (new_x < 0) new_x = 3;
                curr_x = new_x;
            } else if (left_flag == true) {
                left_flag = false;
                int new_x = curr_x;
                new_x++;
                if (new_x > 3) new_x = 0;
                curr_x = new_x;
            }
            if (god_mode) {
                curr_y--;
                if (curr_y <= half_board) shift_map();
            } else {
                if (wasFalling) {
                    if (map[curr_y + 1][curr_x] == plat) {
                        // this is where the jump starts
                        PWM_Change_Tone(700, 100);
                        wasFalling = false;
                        curr_y--;
                        jump = 7;
                        if (curr_y <= half_board) {
                            shift_map();
                        }
                    } else if (map[curr_y + 1][curr_x] == spring_plat) {
                        // this is where the jump starts
                        PWM_Change_Tone(700, 100);
                        wasFalling = false;
                        curr_y--;
                        jump = 20;
                        if (curr_y <= half_board) {
                            shift_map();
                        }
                    } else if (map[curr_y + 1][curr_x] == broke_plat) {
                        map[curr_y + 1][curr_x] = blank;
                        curr_y++;
                    } else if (map[curr_y + 1][curr_x] == blank) {
                        curr_y++;
                    } else if (map[curr_y + 1][curr_x] == alien) {
                        // commented to test sth
                        alien_collision = true;

                    } else if (map[curr_y + 1][curr_x] == black_hole) {
                        // commented to test sth
                        end_game();
                        return;
                    }
                } else {
                    // wasJumping
                    jump--;
                    if (jump > 0) {
                        // still jumping
                        curr_y--;

                        if (map[curr_y][curr_x] == alien) {
                            // commented to test sth
                            alien_collision = true;

                        } else if (map[curr_y][curr_x] == black_hole) {
                            // TODO
                            // game over beacuse collision with black hole (add different end screen (animation?))

                            // commented to test sth
                            end_game();
                            return;
                        }

                        if (curr_y <= half_board) {
                            score += difficulty + 1;
                            shift_map();
                            is_10_score = false;

                        } else {
                            if (is_10_score && 19 - curr_y > score)
                                score = (19 - curr_y) * (difficulty + 1);
                        }
                    } else {
                        //end of jump

                        // this is where jumping ends
                        // (start of falling)
                        PWM_Change_Tone(400, 100);
                        wasFalling = true;
                        jump = 0;
                        // anythin else ? ? ?
                        // curr_y++ ? --
                    }
                }
            }

        }
    } else {
        //alien_collision = true
        curr_y++;
    }
    if (curr_y >= 19) {
        if (alien_collision) {
            // game ended cause collided with alien
        } else {
            // game ended cause ?
        }
        // end game
        end_game();
        return;
    }
    // update LCD
    update_lcd();
    copy_map(old_map, map);


    //check for in-active bullets:
    for (int i = 0; i < 20; i++) {
        if (bullets_pos[i][0] < 0) {
            bullets_pos[i][0] = in_active_bullet;
            bullets_pos[i][1] = in_active_bullet;
        }
    }

    copy_bullets_pos(old_bullets_pos, bullets_pos);

    frq_counter = (frq_counter + 1) % 3;
    /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void) {
    /* USER CODE BEGIN TIM4_IRQn 0 */

    /* USER CODE END TIM4_IRQn 0 */
    HAL_TIM_IRQHandler(&htim4);
    /* USER CODE BEGIN TIM4_IRQn 1 */
    show_number(difficulty * 1000 + (score % 1000), state_7segment);
    state_7segment = (state_7segment + 1) % 4;
    HAL_ADC_Start_IT(&hadc4);
    /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt / UART4 wake-up interrupt through EXTI line 34.
  */
void UART4_IRQHandler(void) {
    /* USER CODE BEGIN UART4_IRQn 0 */

    /* USER CODE END UART4_IRQn 0 */
    HAL_UART_IRQHandler(&huart4);
    /* USER CODE BEGIN UART4_IRQn 1 */
    HAL_UART_Receive_IT(&huart4, &data, sizeof(unsigned char));

    /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles Timer 6 interrupt and DAC underrun interrupts.
  */
void TIM6_DAC_IRQHandler(void) {
    /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

    /* USER CODE END TIM6_DAC_IRQn 0 */
    HAL_TIM_IRQHandler(&htim6);
    /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
    /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles ADC4 interrupt.
  */
void ADC4_IRQHandler(void) {
    /* USER CODE BEGIN ADC4_IRQn 0 */

    /* USER CODE END ADC4_IRQn 0 */
    HAL_ADC_IRQHandler(&hadc4);
    /* USER CODE BEGIN ADC4_IRQn 1 */
    int x = HAL_ADC_GetValue(&hadc4);
    x = x * 10 / 4000;
    difficulty = x;
    /* USER CODE END ADC4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void shift_map() {
    //update player "y"
    curr_y++;

    // update bullets "y"
    for (int i = 0; i < 20; ++i) {
        if (bullets_pos[i][0] != in_active_bullet && bullets_pos[i][1] != in_active_bullet) {
            bullets_pos[i][0]++;
        }
    }


    //shift map[rows[1->19]] one unit down
    for (int i = 19; i >= 1; i--) {
        for (int j = 0; j < 4; j++) {
            map[i][j] = map[i - 1][j];
        }
    }

    if (buffer_content < 0) {
        fill_buffer(difficulty);
    }

    for (int i = 0; i < 4; i++) {
        map[0][i] = row_buffer[buffer_content][i];
    }

    buffer_content--;

    // build new row
    // build a specific function based on difficulty
    // int new_row[4];
    // fill_new_row();
}

int get_max(int a, int b) {
    if (a > b) return a;
    else return b;
}

int get_min(int a, int b) {
    if (a > b) return b;
    else return a;
}

double randfrom(double min, double max) {
    double range = (max - min);
    double div = RAND_MAX / range;
    return min + (rand() / div);
}

void make_ps(int diff) {
    // map values:
    /*
    const int blank = 0;
    const int plat = 1;
    const int broke_plat = 2;
    const int spring_plat = 3;
    const int black_hole = 4;
    const int alien = 5;
    */

    // ps[i][0] -> value of object i in the map[][]
    // ps[i][1] -> # of object i in the buffer based on difficulty


    int c_temp[6];
    // [0] - blank_ps_count
    // [1] - plat_ps_count
    // [2] - broke_plat_ps_count
    // [3] - spring_plat_ps_count
    // [4] - black_hole_ps_count
    // [5] - alien_ps_count

    for (int i = 0; i < num_of_elements; i++) {
        ps[i][0] = i;
    }

    if (diff == 0) {
        ps[0][1] = 1;
        ps[1][1] = 2;
        ps[2][1] = 0;
        ps[3][1] = 5;
        ps[4][1] = 0;
        ps[5][1] = 0;
    } else if (diff == 1) {
        ps[0][1] = 1;
        ps[1][1] = 2;
        ps[2][1] = 1;
        ps[3][1] = 4;
        ps[4][1] = 0;
        ps[5][1] = 0;
    } else if (diff == 2) {
        ps[0][1] = 2;
        ps[1][1] = 2;
        ps[2][1] = 2;
        ps[3][1] = 2;
        ps[4][1] = 0;
        ps[5][1] = 0;
    } else if (diff == 3) {
        ps[0][1] = 3;
        ps[1][1] = 2;
        ps[2][1] = 2;
        ps[3][1] = 1;
        ps[4][1] = 0;
        ps[5][1] = 0;
    } else if (diff == 4) {
        ps[0][1] = 2;
        ps[1][1] = 2;
        ps[2][1] = 3;
        ps[3][1] = 1;
        ps[4][1] = 0;
        ps[5][1] = 0;
    } else if (diff == 5) {
        ps[0][1] = 3;
        ps[1][1] = 2;
        ps[2][1] = 1;
        ps[3][1] = 1;
        ps[4][1] = 0;
        ps[5][1] = 1;
    } else if (diff == 6) {
        ps[0][1] = 3;
        ps[1][1] = 2;
        ps[2][1] = 0;
        ps[3][1] = 1;
        ps[4][1] = 0;
        ps[5][1] = 2;
    } else if (diff == 7) {
        ps[0][1] = 3;
        ps[1][1] = 2;
        ps[2][1] = 0;
        ps[3][1] = 1;
        ps[4][1] = 1;
        ps[5][1] = 1;
    } else if (diff == 8) {
        ps[0][1] = 1;
        ps[1][1] = 2;
        ps[2][1] = 1;
        ps[3][1] = 1;
        ps[4][1] = 1;
        ps[5][1] = 2;
    } else if (diff == 9) {
        ps[0][1] = 2;
        ps[1][1] = 3;
        ps[2][1] = 0;
        ps[3][1] = 0;
        ps[4][1] = 2;
        ps[5][1] = 1;
    }
}

void fill_buffer(int diff) {
    // ps[i][0] -> value of object i in the map[][]
    // ps[i][1] -> # of object i in the buffer based on difficulty
    make_ps(diff);

    int row_select_map[row_buffer_size];
    for (int i = 0; i < row_buffer_size; i++) {
        row_select_map[i] = -1;
    }

    for (int i = 0; i < num_of_elements; i++) {
        int num_of_object = ps[i][1];
        // ps[i][0] -> value of object in the map
        for (int j = 0; j < num_of_object; j++) {
            int curr_row = -1;
            do {
                curr_row = (rand()) % row_buffer_size;
                curr_row = (curr_row + 3) % row_buffer_size;
            } while (row_select_map[curr_row] != -1);
            row_select_map[curr_row] = ps[i][0];
        }
    }

    double x;
    double res;

    double amplitude = 2.0;

    double wavelength = randfrom(0.1, 3);
    double domain = 2 * M_PI * wavelength;

    for (int i = 0; i < row_buffer_size; i++) {
        x = randfrom(0, domain);
        int temp = 0;
        if (i % 2 == 1) temp = (M_PI * wavelength / 2);
        x = amplitude * sin((x / wavelength) + temp) + amplitude;
        int alter = ((int) (x * 1000) % 10) / 2.5;

        for (int j = 0; j < 4; j++) {
            if (j == alter) {
                if (row_select_map[i] == plat || row_select_map[i] == spring_plat) {
                    if (i != 0 && row_buffer[i - 1][alter] == black_hole) {
                        alter++;
                        if (alter > 3) alter = 0;
                    }
                } else {
                    if (i != 0 && row_buffer[i - 1][alter] != blank) {
                        // two objact same col
                        int rand_chance = rand() % 10;
                        if (rand_chance <= 8) {
                            // do not allow same column
                            alter += 2;
                            alter = alter % 4;
                        } else {
                            // allow same column
                        }
                    }
                }
                row_buffer[i][alter] = row_select_map[i];
            } else row_buffer[i][j] = blank;
        }
    }
    buffer_content = row_buffer_size - 1;
}

int get_custom_char_index(int value) {
    if (value == plat) return 0;
    else if (value == broke_plat) return 1;
    else if (value == spring_plat) return 2;
    else if (value == black_hole) return 3;
    else if (value == alien) return 4;
    else {
        // should not happen
    }

}

void write_map_on_lcd(int map_value, int y, int x) {
    setCursor(y, x);
    if (map_value == blank) {
        print(" ");
    } else {
        write(get_custom_char_index(map_value));
    }
}

void update_lcd() {
    for (int i = 0; i < map_m; i++) {
        for (int j = 0; j < map_n; j++) {
            if (i == curr_y && j == curr_x) {
                if (old_x != curr_x || old_y != curr_y) {
                    // write player on lcd
                    setCursor(curr_y, curr_x);
                    write(5);

                    // ********************************************************
                    // in lcd, position [old_y][old_x] is currently displaying
                    // player character, so we need to write the character of map
                    // in that position instead, since the player is not in the
                    // position [old_y][old_x] anymore
                    setCursor(old_y, old_x);
                    write_map_on_lcd(map[old_y][old_x], old_y, old_x);
                }
            } else {
                // print bullets first
                bool is_bullet = false;
                bool was_bullet = false;
                for (int bull = 0; bull < 20; bull++) {
                    if (bullets_pos[bull][0] != in_active_bullet && bullets_pos[bull][1] != in_active_bullet) {
                        // active bullet
                        if (bullets_pos[bull][0] == i && bullets_pos[bull][1] == j) {
                            is_bullet = true;
                            setCursor(i, j);
                            print(".");

                            //break ? ? ? ??
                        }
                    }
                }

                for (int old_bull = 0; old_bull < 20; old_bull++) {
                    int old_b_y = old_bullets_pos[old_bull][0];
                    int old_b_x = old_bullets_pos[old_bull][1];
                    if (old_b_y != -1 && old_b_x != -1) {
                        if (old_b_y == i && old_b_x == j) {
                            was_bullet = true;
                        }
                    }
                }

                if (was_bullet && !is_bullet) {
                    // if it "was" bullet
                    write_map_on_lcd(map[i][j], i, j);
                } else if (!is_bullet && !was_bullet && old_map[i][j] != map[i][j]) {
                    write_map_on_lcd(map[i][j], i, j);
                }
            }
        }
    }
}

void build_initial_map() {
    for (int i = 0; i < 20; ++i) {
        int random_col = rand() % 4;

        for (int j = 0; j < 4; ++j) {
            if (j != random_col) map[i][j] = blank;
            else {
                map[i][j] = plat;
                //map[i][j] = blank;
            }
        }
    }

    map[curr_y][curr_x] = blank;
    map[curr_y + 1][curr_x] = plat;
}

void init_lcd() {
    //player on [18][1]
    curr_x = player_initial_x;
    curr_y = player_initial_y;

    for (int i = 0; i < 20; ++i) {
        old_bullets_pos[i][0] = in_active_bullet;
        old_bullets_pos[i][1] = in_active_bullet;
        bullets_pos[i][0] = in_active_bullet;
        bullets_pos[i][1] = in_active_bullet;
    }

    createChar(0, plat_char);
    createChar(1, broke_plat_char);
    createChar(2, spring_plat_char);
    createChar(3, black_hole_char);
    createChar(4, alien_char);
    createChar(5, player_char);

    srand(HAL_GetTick());

    // build initial map
    build_initial_map();

    copy_map(old_map, map);

    // init lcd:
    for (int i = 19; i >= 0; i--) {
        for (int j = 0; j < 4; ++j) {
            setCursor(i, j);
            if (i == curr_y && j == curr_x) {
                write(5);
            } else if (map[i][j] == blank) {
                print(" ");
            } else {
                write(get_custom_char_index(map[i][j]));
            }
        }
    }

    // start the timer
    HAL_TIM_Base_Start_IT(&htim3);

}

// Input pull down rising edge trigger interrupt pins:
// Row1 PD3, Row2 PD5, Row3 PD7, Row4 PB4
GPIO_TypeDef *const Row_ports[] = {GPIOD, GPIOD, GPIOD, GPIOB};
const uint16_t Row_pins[] = {GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_4};
// Output pins: Column1 PD4, Column2 PD6, Column3 PB3, Column4 PB5
GPIO_TypeDef *const Column_ports[] = {GPIOD, GPIOD, GPIOB, GPIOB};
const uint16_t Column_pins[] = {GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_3, GPIO_PIN_5};
volatile uint32_t last_gpio_exti;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (last_gpio_exti + 200 > HAL_GetTick()) // Simple button debouncing
    {
        return;
    }
    last_gpio_exti = HAL_GetTick();

    int8_t row_number = -1;
    int8_t column_number = -1;

    if (GPIO_Pin == GPIO_PIN_0) {
        // blue_button_pressed = 1;
        return;
    }

    for (uint8_t row = 0; row < 4; row++) // Loop through Rows
    {
        if (GPIO_Pin == Row_pins[row]) {
            row_number = row;
        }
    }

    HAL_GPIO_WritePin(Column_ports[0], Column_pins[0], 0);
    HAL_GPIO_WritePin(Column_ports[1], Column_pins[1], 0);
    HAL_GPIO_WritePin(Column_ports[2], Column_pins[2], 0);
    HAL_GPIO_WritePin(Column_ports[3], Column_pins[3], 0);

    for (uint8_t col = 0; col < 4; col++) // Loop through Columns
    {
        HAL_GPIO_WritePin(Column_ports[col], Column_pins[col], 1);
        if (HAL_GPIO_ReadPin(Row_ports[row_number], Row_pins[row_number])) {
            column_number = col;
        }
        HAL_GPIO_WritePin(Column_ports[col], Column_pins[col], 0);
    }

    HAL_GPIO_WritePin(Column_ports[0], Column_pins[0], 1);
    HAL_GPIO_WritePin(Column_ports[1], Column_pins[1], 1);
    HAL_GPIO_WritePin(Column_ports[2], Column_pins[2], 1);
    HAL_GPIO_WritePin(Column_ports[3], Column_pins[3], 1);

    if (row_number == -1 || column_number == -1) {
        return; // Reject invalid scan
    }
    //   C0   C1   C2   C3
    // +----+----+----+----+
    // | 1  | 2  | 3  | 4  |  R0
    // +----+----+----+----+
    // | 5  | 6  | 7  | 8  |  R1
    // +----+----+----+----+
    // | 9  | 10 | 11 | 12 |  R2
    // +----+----+----+----+
    // | 13 | 14 | 15 | 16 |  R3
    // +----+----+----+----+
    const uint8_t button_number = row_number * 4 + column_number + 1;
    switch (button_number) {
        case 1:
            break;
        case 2:
            if (status == status_select_name) {
                if (current_char[0] == 'a') {
                    current_char[0] = 'b';
                } else if (current_char[0] == 'b') {
                    current_char[0] = 'c';
                } else if (current_char[0] == 'c') {
                    current_char[0] = 'A';
                } else if (current_char[0] == 'A') {
                    current_char[0] = 'B';
                } else if (current_char[0] == 'B') {
                    current_char[0] = 'C';
                } else {
                    current_char[0] = 'a';
                }
                setCursor(char_pos, 0);
                print(current_char);
            }
            break;
        case 3:
            if (status == status_select_name) {
                if (current_char[0] == 'd') {
                    current_char[0] = 'e';
                } else if (current_char[0] == 'e') {
                    current_char[0] = 'f';
                } else if (current_char[0] == 'f') {
                    current_char[0] = 'D';
                } else if (current_char[0] == 'D') {
                    current_char[0] = 'E';
                } else if (current_char[0] == 'B') {
                    current_char[0] = 'F';
                } else {
                    current_char[0] = 'd';
                }

                setCursor(char_pos, 0);
                print(current_char);
            }
            break;
        case 4:
            if (status == status_menu) {
                status = status_game;
                HAL_TIM_Base_Start_IT(&htim4);
                // init game
                init_lcd();

            } else if (status == status_info) {
                show_menu();
            } else if (status == status_game) {
                // right shift
                right_flag = true;
                left_flag = false;
                send_UART("right shift");
            }
            break;
        case 5:
            if (status == status_select_name) {
                if (current_char[0] == 'g') {
                    current_char[0] = 'h';
                } else if (current_char[0] == 'h') {
                    current_char[0] = 'i';
                } else if (current_char[0] == 'i') {
                    current_char[0] = 'G';
                } else if (current_char[0] == 'G') {
                    current_char[0] = 'H';
                } else if (current_char[0] == 'H') {
                    current_char[0] = 'I';
                } else {
                    current_char[0] = 'g';
                }
            }
            break;
        case 6:
            if (status == status_select_name) {
                if (current_char[0] == 'j') {
                    current_char[0] = 'k';
                } else if (current_char[0] == 'k') {
                    current_char[0] = 'l';
                } else if (current_char[0] == 'l') {
                    current_char[0] = 'J';
                } else if (current_char[0] == 'J') {
                    current_char[0] = 'K';
                } else if (current_char[0] == 'K') {
                    current_char[0] = 'L';
                } else {
                    current_char[0] = 'j';
                }

                setCursor(char_pos, 0);
                print(current_char);
            }
            break;
        case 7:
            if (status == status_select_name) {
                if (current_char[0] == 'm') {
                    current_char[0] = 'n';
                } else if (current_char[0] == 'n') {
                    current_char[0] = 'o';
                } else if (current_char[0] == 'o') {
                    current_char[0] = 'M';
                } else if (current_char[0] == 'M') {
                    current_char[0] = 'N';
                } else if (current_char[0] == 'N') {
                    current_char[0] = 'O';
                } else {
                    current_char[0] = 'm';
                }

                setCursor(char_pos, 0);
                print(current_char);
            }
            break;
        case 8:
            if (status == status_menu) {
                status = status_info;
                begin(20, 4);
                setCursor(2, 0);
                print("Amirreza Nayyeri");
                setCursor(1, 1);
                print("Hamed Mohmmadzadeh");
                HAL_RTC_GetTime(&hrtc, &rTime, RTC_FORMAT_BIN);
                HAL_RTC_GetDate(&hrtc, &rDate, RTC_FORMAT_BIN);
                char time_date[20];
                sprintf(time_date, "20%d-%d-%d  %d:%d:%d", rDate.Year, rDate.Month, rDate.Date, rTime.Hours,
                        rTime.Minutes, rTime.Seconds);
                setCursor(0, 3);
                print(time_date);

            } else if (status == status_game) {
                //left shift
                left_flag = true;
                right_flag = false;
                send_UART("left shift");
            } else if (status == status_select_name) {
                strcat(name, current_char);
                show_menu();
            }
            break;
        case 9:
            if (status == status_select_name) {
                if (current_char[0] == 'p') {
                    current_char[0] = 'q';
                } else if (current_char[0] == 'q') {
                    current_char[0] = 'r';
                } else if (current_char[0] == 'r') {
                    current_char[0] = 's';
                } else if (current_char[0] == 's') {
                    current_char[0] = 'P';
                } else if (current_char[0] == 'P') {
                    current_char[0] = 'Q';
                } else if (current_char[0] == 'Q') {
                    current_char[0] = 'R';
                } else if (current_char[0] == 'R') {
                    current_char[0] = 'S';
                } else {
                    current_char[0] = 'p';
                }

                setCursor(char_pos, 0);
                print(current_char);
            }
            break;
        case 10:
            if (status == status_select_name) {
                if (current_char[0] == 't') {
                    current_char[0] = 'u';
                } else if (current_char[0] == 'u') {
                    current_char[0] = 'v';
                } else if (current_char[0] == 'v') {
                    current_char[0] = 'T';
                } else if (current_char[0] == 'T') {
                    current_char[0] = 'U';
                } else if (current_char[0] == 'U') {
                    current_char[0] = 'V';
                } else {
                    current_char[0] = 't';
                }

                setCursor(char_pos, 0);
                print(current_char);
            }
            break;
        case 11:
            if (status == status_select_name) {
                if (current_char[0] == 'w') {
                    current_char[0] = 'x';
                } else if (current_char[0] == 'x') {
                    current_char[0] = 'y';
                } else if (current_char[0] == 'y') {
                    current_char[0] = 'z';
                } else if (current_char[0] == 'z') {
                    current_char[0] = 'W';
                } else if (current_char[0] == 'W') {
                    current_char[0] = 'X';
                } else if (current_char[0] == 'X') {
                    current_char[0] = 'Y';
                } else if (current_char[0] == 'Y') {
                    current_char[0] = 'Z';
                } else {
                    current_char[0] = 'w';
                }

                setCursor(char_pos, 0);
                print(current_char);
            }
            break;
        case 12:
            if (status == status_menu) {
                status = status_select_name;
                clear();
                char_pos = 0;
                current_char[0] = ' ';
                strcpy(name, "");
            } else if (status == status_game) {
                shot_fired = true;
                send_UART("Shot Fired");
            }
            break;
        case 13:
            /* code */
            break;
        case 14:
            if (status == status_select_name) {
                strcat(name, current_char);
                current_char[0] = ' ';
                char_pos += 1;

            }
            break;
        case 15:
            /* code */
            break;
        case 16:
            /* code */
            break;
        default:
            break;
    }
}


void show_menu() {
    status = status_menu;
    begin(20, 4);
    setCursor(5, 0);
    print("Start Game");
    setCursor(8, 1);
    print("About");
    setCursor(5, 2);
    print("Select Name");
    char string[100];
    sprintf(string, "name : %s", name);
    setCursor(0, 3);
    print(string);
}

TIM_HandleTypeDef *pwm_timer = &htim2; // Point to PWM timer configured in CubeMX
uint32_t pwm_channel = TIM_CHANNEL_2;  // Specify configured PWM channel

void PWM_Start() {
    HAL_TIM_PWM_Start(pwm_timer, pwm_channel);
}

void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume) // pwm_freq (1 - 20000), volume (0 - 1000)
{
    if (pwm_freq == 0 || pwm_freq > 20000) {
        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, 0);
    } else {
        const uint32_t internal_clock_freq = HAL_RCC_GetSysClockFreq();
        // const uint16_t prescaler = 1;
        const uint16_t prescaler = 1 + internal_clock_freq / pwm_freq / 60000;
        const uint32_t timer_clock = internal_clock_freq / prescaler;
        const uint32_t period_cycles = timer_clock / pwm_freq;
        const uint32_t pulse_width = volume * period_cycles / 1000 / 2;

        pwm_timer->Instance->PSC = prescaler - 1;
        pwm_timer->Instance->ARR = period_cycles - 1;
        pwm_timer->Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, pulse_width); // pwm_timer->Instance->CCR2 = pulse_width;
    }
}

void send_UART(char *string) {
    HAL_RTC_GetTime(&hrtc, &rTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &rDate, RTC_FORMAT_BIN);
    char s[1000] = "";
    int size = sprintf(s, "%s name:%s 20%d-%d-%d  %d:%d:%d\n", string, name, rDate.Year, rDate.Month, rDate.Date,
                       rTime.Hours,
                       rTime.Minutes, rTime.Seconds);
    HAL_UART_Transmit_IT(&huart4, s, size);

}

void end_game() {
    status = status_end;
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_TIM_Base_Stop_IT(&htim4);
    reset_port_7segment();
    begin(20, 4);
    setCursor(3, 0);
    print(name);
    char string[100] = "";
    sprintf(string, "Score: %d", score);
    setCursor(3, 1);
    print(string);
    HAL_TIM_Base_Start_IT(&htim6);
    setup_melody(starwars_sound, sizeof(starwars_sound));
}

void setup_melody(int melody[], int size_arr) {
    int tempo = 180;
    int notes = size_arr / sizeof(melody[0]) / 2;
    int wholenote = (60000 * 4) / tempo;
    int divider = 0, noteDuration = 0;

    PWM_Start();
    for (int thisNote = 0;
         thisNote < notes * 2 && (status == status_start || status == status_end); thisNote = thisNote + 2) {
        divider = melody[thisNote + 1];
        if (divider > 0) {
            noteDuration = (wholenote) / divider;
        } else if (divider < 0) {
            noteDuration = (wholenote) / abs(divider);
            noteDuration *= 1.5;
        }

        PWM_Change_Tone(melody[thisNote], 100);

        HAL_Delay(noteDuration);

        PWM_Change_Tone(melody[thisNote], 0);
    }
}

/* USER CODE END 1 */
