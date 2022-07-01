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
#include <stdbool.h>

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
int score = 888;
int difficulty = 4;
typedef unsigned char byte;

byte platform_char[] = {
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00
};

byte broken_plat_char[] = {
        0x00,
        0x01,
        0x01,
        0x00,
        0x00,
        0x01,
        0x01,
        0x00
};

byte spring_plat_char[] = {
        0x00,
        0x15,
        0x0B,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00
};

byte black_hole_char[] = {
        0x00,
        0x0E,
        0x11,
        0x11,
        0x11,
        0x11,
        0x0E,
        0x00
};


byte alien_char[] = {
        0x11,
        0x0A,
        0x06,
        0x0E,
        0x0E,
        0x06,
        0x0A,
        0x11
};

byte player_char[] = {
        0x00,
        0x00,
        0x00,
        0x16,
        0x16,
        0x1E,
        0x0C,
        0x00
};


byte p_on_plat_char[] = {
        0x00,
        0x01,
        0x01,
        0x17,
        0x17,
        0x1F,
        0x0D,
        0x00
};


byte p_on_broken_plat_char[] = {
        0x00,
        0x01,
        0x01,
        0x16,
        0x16,
        0x1F,
        0x0D,
        0x00
};

byte p_on_spring_plat_char[] = {
        0x00,
        0x15,
        0x0B,
        0x01,
        0x17,
        0x17,
        0x1F,
        0x0C
};

byte bullet_char[] = {
        0x00,
        0x00,
        0x00,
        0x0C,
        0x0C,
        0x00,
        0x00,
        0x00
};

byte bullet_on_plat_char[] = {
        0x00,
        0x01,
        0x01,
        0x0D,
        0x0D,
        0x01,
        0x01,
        0x00
};

byte bullet_on_broken_plat_char[] = {
        0x00,
        0x01,
        0x01,
        0x0C,
        0x0C,
        0x01,
        0x01,
        0x00
};

byte bullet_on_spring_plat_char[] = {
        0x00,
        0x15,
        0x0B,
        0x0D,
        0x0D,
        0x01,
        0x01,
        0x00
};

void send_UART(char *string);

void show_menu();

void PWM_Start();

void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume);


const int plat = 0;                            //platform_char
const int broken_plat = 1;                    //broken_plat_char
const int spring_plat = 2;                    //spring_plat_char
const int black_hole = 3;                    //black_hole_char
const int alien = 4;                        //alien_char
const int player_blank = 5;                    //player_char
const int p_on_plat = 6;                    //player_char -> p_on_plat_char
const int p_on_broken_plat = 7;                //player_char -> p_on_broken_plat_char
const int p_on_spring_plat = 8;                //player_char -> p_on_spring_plat_char
const int p_on_alien = 9;                    //player_char ? ? ?
const int p_on_black_hole = 10;                //player_char ? ? ?
const int bullet_blank = 11;                //bullet_char
const int bullet_on_plat = 12;                //bullet_char -> bullet_on_plat_char
const int bullet_on_broken_plat = 13;        //bullet_char -> bullet_on_broken_plat_char
const int bullet_on_spring_plat = 14;        //bullet_char -> bullet_on_spring_plat
const int bullet_on_black_hole = 15;        //bullet_char -> ?
const int bullet_on_alien = 16;                //bullet_char -> ?
const int blank = 20;
int char_map[20];

bool isFalling = false;
int curr_y = 19;
int curr_x = 1;
int jump = 7;

int get_custom_char_index(int value) {
    // update customChar if needed (if player on plat and...)
    return char_map[value];

}

const int platform_char_indx = 0;
const int broken_plat_char_indx = 1;
const int spring_plat_char_indx = 2;
const int black_hole_char_indx = 3;
const int alien_char_indx = 4;
const int player_char_indx = 5;
const int bullet_char_indx = 6;
int map[20][4];

void init_lcd() {
    char_map[plat] = 0;                //platform_char
    char_map[broken_plat] = 1;            //broken_plat_char
    char_map[spring_plat] = 2;            //spring_plat_char
    char_map[black_hole] = 3;            //black_hole_char
    char_map[alien] = 4;                //alien_char
    char_map[player_blank] = 5;            //player_char
    char_map[p_on_plat] = 5;            //player_char -> p_on_plat_char
    char_map[p_on_broken_plat] = 5;        //player_char -> p_on_broken_plat_char
    char_map[p_on_spring_plat] = 5;        //player_char -> p_on_spring_plat_char
    char_map[p_on_alien] = 5;            //player_char ? ? ?
    char_map[p_on_black_hole] = 5;        //player_char ? ? ?
    char_map[bullet_blank] = 6;        //bullet_char
    char_map[bullet_on_plat] = 6;        //bullet_char -> bullet_on_plat_char
    char_map[bullet_on_broken_plat] = 6;//bullet_char -> bullet_on_broken_plat_char
    char_map[bullet_on_spring_plat] = 6;//bullet_char -> bullet_on_spring_plat
    char_map[bullet_on_black_hole] = 6;    //bullet_char ? ? ?
    char_map[bullet_on_alien] = 6;        //bullet_char ? ? ?

    createChar(0, platform_char);
    createChar(1, broken_plat_char);
    createChar(2, spring_plat_char);
    createChar(4, black_hole_char);
    createChar(5, player_char);
    createChar(6, bullet_char);


    srand(HAL_GetTick());

    // build map

    //map[][] = 20 --> blank
    for (int i = 0; i < 20; ++i) {
        int random_col = rand() % 4;

        for (int j = 0; j < 4; ++j) {
            if (j != random_col) map[i][j] = blank;
            else {
                map[i][j] = plat;
            }
        }
    }

    //player on [1][2]
    createChar(player_char_indx, p_on_plat_char);
    map[19][1] = p_on_plat;

    // init lcd:
    for (int i = 19; i >= 0; i--) {
        for (int j = 0; j < 4; ++j) {
            setCursor(i, j);
            if (map[i][j] == blank) {
                print(" ");
            } else {
                write(get_custom_char_index(map[i][j]));
            }
        }
    }
}


void reset_port_7segment() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 0);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 0);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 0);

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

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, bcd % 10);
    bcd /= 10;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, bcd % 10);
    bcd /= 10;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, bcd % 10);
    bcd /= 10;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, bcd % 10);
    bcd /= 10;

    switch (pin) {
        case 4:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
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
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */
extern const int player_index, stair_index, broke_stair_index, coil_index, status_start, status_menu,
        status_info, status_game, status_end;
extern int status;
extern RTC_TimeTypeDef rTime;
extern RTC_DateTypeDef rDate;
extern RTC_HandleTypeDef hrtc;
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
    if (isFalling) {

    } else {
        // not isFalling
        if (jump > 0) {
            curr_y++;
            jump--;

            if (map[curr_y][curr_x] == blank) {

            }
        }
    }
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
    show_number(difficulty * 1000 + score, state_7segment);
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

    /* USER CODE END UART4_IRQn 1 */
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
    x = x * 10 / 4095;
    difficulty = x;
    /* USER CODE END ADC4_IRQn 1 */
}

/* USER CODE BEGIN 1 */


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
            /* code */
            break;
        case 3:
            /* code */
            break;
        case 4:
            if (status == status_menu) {
                status = status_game;
                //init game
                init_lcd();

            } else if (status == status_info) {
                show_menu();
            } else if (status == status_game) {
                //right shift
            }
            break;
        case 5:
            /* code */
            break;
        case 6:
            /* code */
            break;
        case 7:
            /* code */
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
            }
            break;
        case 9:
            /* code */
            break;
        case 10:
            /* code */
            break;
        case 11:
            /* code */
            break;
        case 12:
            /* code */
            break;
        case 13:
            /* code */
            break;
        case 14:
            /* code */
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
    setCursor(5, 1);
    print("Start Game");
    setCursor(8, 2);
    print("About");
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
    int size = sprintf(s, "%s 20%d-%d-%d  %d:%d:%d\n", string, rDate.Year, rDate.Month, rDate.Date, rTime.Hours,
                       rTime.Minutes, rTime.Seconds);
    HAL_UART_Transmit(&huart4, s, size, 100);

}
/* USER CODE END 1 */
