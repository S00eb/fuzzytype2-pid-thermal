
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Handle */
ADC_HandleTypeDef hadc_eco;
TIM_HandleTypeDef htim_eco;
UART_HandleTypeDef huart_eco;

/* --- Variabel Sensor */
uint16_t raw_suhu = 0;
uint16_t raw_cahaya = 0;
float suhu_celsius = 0.0f; /* 0 - 100 derajat C */
float cahaya_lux = 0.0f;   /* 0 - 1000 Lux */

/* --- Variabel PID*/
float setpoint_suhu = 50.0f;            /* Target suhu optimal (derajat C) */
float Kp = 12.0f, Ki = 0.05f, Kd = 1.0f; /* Gain lebih halus */
float error = 0, prev_error = 0, integral = 0;
int output_pwm = 0;
char msg_eco[120];

/* Variabel display  */
int suhu_int10 = 0; /* suhu x10, misal 253 = 25.3C */
int cahaya_int = 0; /* cahaya dalam Lux (bulat) */

/* --- Task Scheduling --- */
uint32_t tick_sensor = 0;
uint32_t tick_fuzzy = 0;
uint32_t tick_motor = 0;
uint32_t tick_uart = 0;

/* Prototypes */
void EcoGrid_GPIO_Init(void);
void EcoGrid_UART_Init(void);
void EcoGrid_ADC_Init(void);
void EcoGrid_TIM_Init(void);
void EcoGrid_Clock_Init(void);
uint16_t EcoGrid_ReadADC(uint32_t channel);
float IT2_Fuzzy_PID(float suhu, float cahaya, float sp);

/* SOFT I2C LCD DRIVER */
#define SCL_PIN GPIO_PIN_8
#define SDA_PIN GPIO_PIN_9
#define I2C_PORT GPIOB
#define LCD_ADDR 0x40  

void I2C_Delay(void) {
    volatile int i = 50; while(i--);
}

void I2C_Start(void) {
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
}

void I2C_Stop(void) {
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET);
    I2C_Delay();
}

void I2C_Write(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET);
        else             HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET);
        I2C_Delay();
        HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);
        I2C_Delay();
        HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
        data <<= 1;
    }
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET); /* Bebaskan SDA untuk ACK */
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET); /* Clock ke-9 */
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
}

void lcd_send_cmd (char cmd) {
    char data_u = (cmd&0xf0);
    char data_l = ((cmd<<4)&0xf0);
    I2C_Start(); I2C_Write(LCD_ADDR);
    I2C_Write(data_u|0x0C); I2C_Write(data_u|0x08);
    I2C_Write(data_l|0x0C); I2C_Write(data_l|0x08);
    I2C_Stop();
}

void lcd_send_data (char data) {
    char data_u = (data&0xf0);
    char data_l = ((data<<4)&0xf0);
    I2C_Start(); I2C_Write(LCD_ADDR);
    I2C_Write(data_u|0x0D); I2C_Write(data_u|0x09);
    I2C_Write(data_l|0x0D); I2C_Write(data_l|0x09);
    I2C_Stop();
}

/* Fungsi khusus hanya untuk tahap inisialisasi awal 4-bit */
void lcd_send_nibble(char nibble) {
    I2C_Start(); I2C_Write(LCD_ADDR);
    I2C_Write(nibble | 0x0C); /* EN=1, RS=0 */
    I2C_Write(nibble | 0x08); /* EN=0, RS=0 */
    I2C_Stop();
}

void lcd_init (void) {
    /* Urutan HD44780 masuk ke mode 4-bit, Delay dikurangi ekstrem agar cepat */
    HAL_Delay(20);
    lcd_send_nibble(0x30); HAL_Delay(5);
    lcd_send_nibble(0x30); HAL_Delay(1);
    lcd_send_nibble(0x30); HAL_Delay(2);
    lcd_send_nibble(0x20); HAL_Delay(2); 
    
    lcd_send_cmd(0x28); HAL_Delay(1); 
    lcd_send_cmd(0x08); HAL_Delay(1); 
    lcd_send_cmd(0x01); HAL_Delay(2); 
    lcd_send_cmd(0x06); HAL_Delay(1); 
    lcd_send_cmd(0x0C); HAL_Delay(1); 
}

void lcd_put_cur(int row, int col) {
    switch (row) {
        case 0: col |= 0x80; break;
        case 1: col |= 0xC0; break;
    }
    lcd_send_cmd(col);
}

void lcd_send_string (char *str) {
    while (*str) lcd_send_data (*str++);
}

/* MAIN */
int main(void) {
  HAL_Init();
  EcoGrid_Clock_Init();
  EcoGrid_GPIO_Init();
  EcoGrid_UART_Init();

  /* UART Transmit awal DITUTUP karena baud rate 4800 memakan 58ms waktu simulasi yang sangat memberatkan */
  // HAL_UART_Transmit(&huart_eco, (uint8_t *)"\r\n=== Wait for system ===\r\n", 28, 100);

  EcoGrid_TIM_Init();
  HAL_TIM_PWM_Start(&htim_eco, TIM_CHANNEL_1);
  EcoGrid_ADC_Init();

  /* Inisialisasi Layar LCD I2C */
  lcd_init();
  
  /* Tidak ada lagi tampilan "Starting...", LCD lgsg dihapus dan lgsg masuk data sensor! */
  lcd_send_cmd(0x01); 

  /* Motor: PA8=PWM ke ENA, PB1=HIGH(IN1), PB2=LOW(IN2) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /* MEMANIPULASI WAKTU AWAL AGAR DATA LANGSUNG MUNCUL TANPA MENUNGGU 1 DETIK! */
  tick_sensor = HAL_GetTick() - 1000;
  tick_fuzzy = HAL_GetTick() - 500;
  tick_motor = HAL_GetTick() - 1000;
  tick_uart = HAL_GetTick() - 500;

  while (1) {
    uint32_t now = HAL_GetTick();

    /* TASK 1: Baca Sensor (500ms) */
    if (now - tick_sensor >= 1000) 
    {
      tick_sensor = now;
      raw_suhu = EcoGrid_ReadADC(ADC_CHANNEL_0);
      raw_cahaya = EcoGrid_ReadADC(ADC_CHANNEL_1);

      /* Konversi satuan BENAR UNTUK LM35 DAN LDR */
      suhu_celsius = (float)raw_suhu * 165.0f / 4095.0f;
      cahaya_lux = (float)raw_cahaya * 1000.0f / 4095.0f;

      /* save integer */
      suhu_int10 = (int)(suhu_celsius * 10.0f);
      cahaya_int = (int)(cahaya_lux);            
    }

    /* TASK 2: Hitung Interval Type-2 Fuzzy-PID (250ms) */
    if (now - tick_fuzzy >= 500) 
    {
      tick_fuzzy = now;
      output_pwm = (int)IT2_Fuzzy_PID(suhu_celsius, cahaya_lux, setpoint_suhu);
    }

    /* TASK 3: Update Motor & LED (500ms) */
    if (now - tick_motor >= 1000) 
    {
      tick_motor = now;
      __HAL_TIM_SET_COMPARE(&htim_eco, TIM_CHANNEL_1, output_pwm);

      if (output_pwm > 350) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      }
    }

    /* TASK 4: Kirim data UART & LCD */
    if (now - tick_uart >= 500) {
      tick_uart = now;
      
      int tegangan_int10 = (output_pwm * 120) / 999;

      int len = sprintf(msg_eco, "T:%d.%dC L:%dLx PWM:%d V:%d.%dV\r\n",
                        suhu_int10 / 10, suhu_int10 % 10, cahaya_int,
                        output_pwm, tegangan_int10 / 10, tegangan_int10 % 10);
      HAL_UART_Transmit(&huart_eco, (uint8_t *)msg_eco, len, 100);

      /* Update LCD I2C */
      char lcd_buf[20];
      
      lcd_put_cur(0, 0);
      sprintf(lcd_buf, "T:%d.%dC L:%dLx  ", suhu_int10/10, suhu_int10%10, cahaya_int);
      lcd_send_string(lcd_buf);
      
      lcd_put_cur(1, 0);
      sprintf(lcd_buf, "PWM:%d V:%d.%dV  ", output_pwm, tegangan_int10/10, tegangan_int10%10);
      lcd_send_string(lcd_buf);
    }
  }
}

/* ==========================================================
 * INTERVAL TYPE-2 FUZZY-PID CONTROLLER
 * ========================================================== */
float IT2_Fuzzy_PID(float suhu, float cahaya, float sp) {
  error = suhu - sp;

  /* ====== FUZZIFIKASI SUHU ====== */
  float mu_s_low_U = 0, mu_s_low_L = 0;
  float mu_s_med_U = 0, mu_s_med_L = 0;
  float mu_s_hi_U = 0, mu_s_hi_L = 0;

  if (suhu <= 30.0f) mu_s_low_U = 1.0f;
  else if (suhu <= 55.0f) mu_s_low_U = (55.0f - suhu) / 25.0f;
  else mu_s_low_U = 0.0f;
  
  if (suhu <= 20.0f) mu_s_low_L = 1.0f;
  else if (suhu <= 45.0f) mu_s_low_L = (45.0f - suhu) / 25.0f;
  else mu_s_low_L = 0.0f;

  if (suhu <= 25.0f) mu_s_med_U = 0.0f;
  else if (suhu <= 50.0f) mu_s_med_U = (suhu - 25.0f) / 25.0f;
  else if (suhu <= 75.0f) mu_s_med_U = (75.0f - suhu) / 25.0f;
  else mu_s_med_U = 0.0f;
  
  if (suhu <= 30.0f) mu_s_med_L = 0.0f;
  else if (suhu <= 50.0f) mu_s_med_L = (suhu - 30.0f) / 20.0f;
  else if (suhu <= 70.0f) mu_s_med_L = (70.0f - suhu) / 20.0f;
  else mu_s_med_L = 0.0f;

  if (suhu <= 45.0f) mu_s_hi_U = 0.0f;
  else if (suhu <= 70.0f) mu_s_hi_U = (suhu - 45.0f) / 25.0f;
  else mu_s_hi_U = 1.0f;
  
  if (suhu <= 55.0f) mu_s_hi_L = 0.0f;
  else if (suhu <= 80.0f) mu_s_hi_L = (suhu - 55.0f) / 25.0f;
  else mu_s_hi_L = 1.0f;

  /* ====== FUZZIFIKASI CAHAYA ====== */
  float mu_c_low_U = 0, mu_c_low_L = 0;
  float mu_c_med_U = 0, mu_c_med_L = 0;
  float mu_c_hi_U = 0, mu_c_hi_L = 0;

  if (cahaya <= 300.0f) mu_c_low_U = 1.0f;
  else if (cahaya <= 550.0f) mu_c_low_U = (550.0f - cahaya) / 250.0f;
  else mu_c_low_U = 0.0f;
  
  if (cahaya <= 200.0f) mu_c_low_L = 1.0f;
  else if (cahaya <= 450.0f) mu_c_low_L = (450.0f - cahaya) / 250.0f;
  else mu_c_low_L = 0.0f;

  if (cahaya <= 250.0f) mu_c_med_U = 0.0f;
  else if (cahaya <= 500.0f) mu_c_med_U = (cahaya - 250.0f) / 250.0f;
  else if (cahaya <= 750.0f) mu_c_med_U = (750.0f - cahaya) / 250.0f;
  else mu_c_med_U = 0.0f;
  
  if (cahaya <= 300.0f) mu_c_med_L = 0.0f;
  else if (cahaya <= 500.0f) mu_c_med_L = (cahaya - 300.0f) / 200.0f;
  else if (cahaya <= 700.0f) mu_c_med_L = (700.0f - cahaya) / 200.0f;
  else mu_c_med_L = 0.0f;

  if (cahaya <= 450.0f) mu_c_hi_U = 0.0f;
  else if (cahaya <= 700.0f) mu_c_hi_U = (cahaya - 450.0f) / 250.0f;
  else mu_c_hi_U = 1.0f;
  
  if (cahaya <= 550.0f) mu_c_hi_L = 0.0f;
  else if (cahaya <= 800.0f) mu_c_hi_L = (cahaya - 550.0f) / 250.0f;
  else mu_c_hi_L = 1.0f;

  /* ====== RULE BASE ====== */
  float rule_gain[9] = {0.3f, 0.5f, 1.0f, 0.7f, 1.0f, 1.5f, 1.2f, 1.5f, 2.0f};

  float w_U[9];
  w_U[0] = (mu_s_low_U < mu_c_low_U) ? mu_s_low_U : mu_c_low_U;
  w_U[1] = (mu_s_low_U < mu_c_med_U) ? mu_s_low_U : mu_c_med_U;
  w_U[2] = (mu_s_low_U < mu_c_hi_U) ? mu_s_low_U : mu_c_hi_U;
  w_U[3] = (mu_s_med_U < mu_c_low_U) ? mu_s_med_U : mu_c_low_U;
  w_U[4] = (mu_s_med_U < mu_c_med_U) ? mu_s_med_U : mu_c_med_U;
  w_U[5] = (mu_s_med_U < mu_c_hi_U) ? mu_s_med_U : mu_c_hi_U;
  w_U[6] = (mu_s_hi_U < mu_c_low_U) ? mu_s_hi_U : mu_c_low_U;
  w_U[7] = (mu_s_hi_U < mu_c_med_U) ? mu_s_hi_U : mu_c_med_U;
  w_U[8] = (mu_s_hi_U < mu_c_hi_U) ? mu_s_hi_U : mu_c_hi_U;

  float w_L[9];
  w_L[0] = (mu_s_low_L < mu_c_low_L) ? mu_s_low_L : mu_c_low_L;
  w_L[1] = (mu_s_low_L < mu_c_med_L) ? mu_s_low_L : mu_c_med_L;
  w_L[2] = (mu_s_low_L < mu_c_hi_L) ? mu_s_low_L : mu_c_hi_L;
  w_L[3] = (mu_s_med_L < mu_c_low_L) ? mu_s_med_L : mu_c_low_L;
  w_L[4] = (mu_s_med_L < mu_c_med_L) ? mu_s_med_L : mu_c_med_L;
  w_L[5] = (mu_s_med_L < mu_c_hi_L) ? mu_s_med_L : mu_c_hi_L;
  w_L[6] = (mu_s_hi_L < mu_c_low_L) ? mu_s_hi_L : mu_c_low_L;
  w_L[7] = (mu_s_hi_L < mu_c_med_L) ? mu_s_hi_L : mu_c_med_L;
  w_L[8] = (mu_s_hi_L < mu_c_hi_L) ? mu_s_hi_L : mu_c_hi_L;

  /* ====== TYPE-REDUCTION ====== */
  float sum_wU = 0, sum_wgU = 0;
  float sum_wL = 0, sum_wgL = 0;
  for (int i = 0; i < 9; i++) {
    sum_wgU += w_U[i] * rule_gain[i];
    sum_wU += w_U[i];
    sum_wgL += w_L[i] * rule_gain[i];
    sum_wL += w_L[i];
  }

  float y_upper = (sum_wU > 0.001f) ? (sum_wgU / sum_wU) : 1.0f;
  float y_lower = (sum_wL > 0.001f) ? (sum_wgL / sum_wL) : 1.0f;

  float fuzzy_gain = (y_upper + y_lower) / 2.0f;

  /* PID CALCULATION */
  float adaptive_sp = sp + (500.0f - cahaya) * 0.02f;

  error = suhu - adaptive_sp; 
  integral += error * 0.25f;

  if (integral > 300.0f) integral = 300.0f;
  if (integral < -300.0f) integral = -300.0f;

  float deriv = error - prev_error;
  prev_error = error;

  float pid = fuzzy_gain * (Kp * error + Ki * integral + Kd * deriv);

  if (pid > 999.0f) pid = 999.0f;
  if (pid < 0.0f) pid = 0.0f;

  return pid;
}

/* CLOCK */
void EcoGrid_Clock_Init(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&osc);

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}

/* GPIO */
void EcoGrid_GPIO_Init(void) {
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);

  g.Pin = GPIO_PIN_2;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &g);

  g.Pin = GPIO_PIN_3;
  g.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &g);

  g.Pin = GPIO_PIN_8;
  g.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &g);

  g.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);

  g.Pin = GPIO_PIN_13 | GPIO_PIN_15;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &g);

  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &g);
}

/* UART */
void EcoGrid_UART_Init(void) {
  __HAL_RCC_USART2_CLK_ENABLE();
  huart_eco.Instance = USART2;
  huart_eco.Init.BaudRate = 4800;
  huart_eco.Init.WordLength = UART_WORDLENGTH_8B;
  huart_eco.Init.StopBits = UART_STOPBITS_1;
  huart_eco.Init.Parity = UART_PARITY_NONE;
  huart_eco.Init.Mode = UART_MODE_TX_RX;
  huart_eco.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_eco.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart_eco);
}

/* ADC */
void EcoGrid_ADC_Init(void) {
  __HAL_RCC_ADC1_CLK_ENABLE();
  hadc_eco.Instance = ADC1;
  hadc_eco.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc_eco.Init.Resolution = ADC_RESOLUTION_12B;
  hadc_eco.Init.ScanConvMode = DISABLE;
  hadc_eco.Init.ContinuousConvMode = DISABLE;
  hadc_eco.Init.DiscontinuousConvMode = DISABLE;
  hadc_eco.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc_eco.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc_eco.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc_eco);
}

/* Tim 1 for PMW */
void EcoGrid_TIM_Init(void) {
  __HAL_RCC_TIM1_CLK_ENABLE();
  htim_eco.Instance = TIM1;
  htim_eco.Init.Prescaler = 159; 
  htim_eco.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_eco.Init.Period = 999; 
  htim_eco.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim_eco.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim_eco);

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim_eco, &oc, TIM_CHANNEL_1);

  TIM_BreakDeadTimeConfigTypeDef brk = {0};
  brk.OffStateRunMode = TIM_OSSR_DISABLE;
  brk.OffStateIDLEMode = TIM_OSSI_DISABLE;
  brk.LockLevel = TIM_LOCKLEVEL_OFF;
  brk.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_eco, &brk);
}

/* Read ADC */
uint16_t EcoGrid_ReadADC(uint32_t channel) {
  ADC_ChannelConfTypeDef cfg = {0};
  cfg.Channel = channel;
  cfg.Rank = 1;
  cfg.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  HAL_ADC_ConfigChannel(&hadc_eco, &cfg);

  HAL_ADC_Start(&hadc_eco);
  if (HAL_ADC_PollForConversion(&hadc_eco, 10) == HAL_OK) {
    uint16_t val = HAL_ADC_GetValue(&hadc_eco);
    HAL_ADC_Stop(&hadc_eco);
    return val;
  }
  HAL_ADC_Stop(&hadc_eco);
  return 0;
}

/* Clock */
void SystemClock_Config(void) {}
void Error_Handler(void) {
  while (1);
}
void vApplicationGetIdleTaskMemory(void **p1, void **p2, uint32_t *p3) {}
void vApplicationGetTimerTaskMemory(void **p1, void **p2, uint32_t *p3) {}
