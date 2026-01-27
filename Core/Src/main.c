/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "mpu6051.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define dev_num 2 //number of led matrices
#define droptime 1000 //The time that spawn a led
#define readtime 200 // the time of reading MPU data


uint8_t display_buffer[16]={0}; //this buffer is used to store the data will be flushed out
uint8_t world[16][16]; //this matrix is for the data to store and calculate

double Ax, Ay;
MPU6050_t MPU6050;


/*---------the primary send data used for Init command of the MAX7219------------*/

void MAX7219_Send(uint8_t add, uint8_t data) {
	uint16_t writeData = (add<<8)|data;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // pull down the CS for the data incoming
    for (int i=0; i<dev_num; i++){
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 100);
    }// Vì có 2 led ma trận nên khởi tạo 2 lần
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);// pull up the CS pin for the data to be locked
}

/* ----This function is for the sake of the visualization the bit to move ------*/


int max_write (int row, uint8_t data)
{
	int devTarget = (row - 1) / 8;  // find out which is the actual max, where we need to write the data
	int offset = devTarget * 8;  // The offset of the start byte for the devTarget in the buffer
	uint16_t writeData = 0;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);  // Select the slave
	for (int dev = 0; dev < dev_num; dev++)   // for loop for all the max connected
	{
		if (dev == devTarget)  // if this the target
		{
			writeData = ((row - offset)<<8)|data;
			HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 1000);
		}
		else
		{
			writeData = 0;  // because 2 led matrices are daisy chained so we have to send
			//the NO-OP data for the matrix we dont want it to operate to that data
			HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 1000);
		}
	}
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);  // disable the slave
	return writeData;
}


/*Initialize 2 matrices*/
void MAX7219_Init(uint8_t intensity) {
    HAL_Delay(100); // this command is optional
    MAX7219_Send(0x0C, 0x01); // Wake up
    MAX7219_Send(0x0F, 0x00); // turn off the test mode
    MAX7219_Send(0x09, 0x00); // No decode
    MAX7219_Send(0x0B, 0x07); // Scan 8 rows
    MAX7219_Send(0x0A, intensity);

    // clear 2 matrices when operate
    for(int i = 1; i <= 8; i++) {
          MAX7219_Send(i, 0x00);
      }
}




/* ----------This function is for the getting state from the raw data from the MPU6050 ------------ */
int get_orientation_state(double Ay, double Ax) {
    // This is for the up mode
    if (Ay<0&&Ax>=0) {
        return 0;
    }

    // turn right 90 degrees
    else if (Ay>=0 &&Ax>=0) {
        return 1;
    }

    // turn 180 degrees or upside down
    else if (Ay>=0&&Ax<0) {
        return 2;
    }

    // the turn lef 90 degrees
    else {
        return 3;
    }
}


/*------This function is for taking 1 bit from the upper matrix and then spawn 1 in the lower -------*/

void source_sink(int state){
	int sink_x, sink_y;
	int source_x, source_y;

	if(state ==0){
		sink_x= 7; sink_y=8;
		source_x=8; source_y=7;

	}
	else if(state==2){
		sink_x= 8; sink_y=7;
		source_x=7; source_y=8;
	}
	else{
		return;
	}
	if (world[sink_x][sink_y] == 1 && world[source_x][source_y] == 0) {

        world[sink_x][sink_y] = 0;
        world[source_x][source_y] = 1;
	}
}
/*------
 * Func: Making the boudary for every state------------*/


bool check_inbound(int x, int y) {
    if (x < 0 || x > 15 || y < 0 || y > 15) return false;

    if (x >= 0 && x <= 7 && y >= 8 && y <= 15) return true;

    if (x >= 8 && x <= 15 && y >= 0 && y <= 7) return true;

    return false;
}

bool check_inbound_state0upper(int x, int y) // =state2below
{
		if (x < 0 || x > 15 || y < 0 || y > 15) return false;

	    if (x>=0&& x<=7&&y>=8&&y<=15) return true;
	    return false;
}
bool check_inbound_state0below(int x, int y) // =state2upper
{
		if (x < 0 || x > 15 || y < 0 || y > 15) return false;

	    if (x>=8&& x<=15&&y>=0&&y<=7) return true;
	    return false;
}

//desired result: state0 will have the boundary of the upper matrix state2 will be lower


/*-------------Func: To check if the led can move or not-------------*/
bool can_move_to(int tx, int ty) {
    if (!check_inbound(tx, ty)) return false; // boundary
    if (world[tx][ty] == 1) return false;     // another led
    return true;
}

bool can_move_to_state0upper(int tx, int ty) //=can_move_to_state2below
{
    if (!check_inbound_state0upper(tx, ty)) return false;
    if (world[tx][ty] == 1) return false;
    return true;
}
bool can_move_to_state0below(int tx, int ty) //=can_move_to_state2upper
{
    if (!check_inbound_state0below(tx, ty)) return false;
    if (world[tx][ty] == 1) return false;
    return true;
}


/*-----------Func: Define the gravity of each state------------*/
// 1.  (Forward)
int8_t fwdX, fwdY;
// 2. (Left)
int8_t leftX, leftY;
// 3. (Right)
int8_t rightX, rightY;
void set_gravity_by_state(int state) {
    switch (state) {

        case 0:
            fwdX = 1;  fwdY = -1;

            leftX = 0; leftY = -1;

            rightX = 1; rightY = 0;
            break;

        case 1:
            fwdX = 1; fwdY = 1;

            leftX = 1; leftY = 0;

            rightX = 0; rightY = 1;
            break;

        case 2:
            fwdX = -1; fwdY = 1;

            leftX = -1; leftY = 0;

            rightX = 0; rightY = 1;
            break;

        case 3:
            fwdX = -1;  fwdY = -1;

            leftX = -1; leftY = 0;

            rightX = 0; rightY = -1;
            break;
    }
}

/*------------Func: Move the particle at the coordinate (x,y) --------------------*/
void move_particle(int x, int y){
	if (world[x][y] == 0) return;
	//go forward
	int next_x = x + fwdX;
	int next_y = y + fwdY;
	//left
	int lX= x+leftX;
	int lY= y+leftY;
	//right
	int rX= x+rightX;
	int rY= y+rightY;
	if(can_move_to(next_x,next_y))
	{
		world[x][y] = 0;
		world[next_x][next_y] = 1;
		return;
	}

	bool can_left  = can_move_to(lX, lY);
	bool can_right = can_move_to(rX, rY);

	    if (can_left && can_right) {
	        // make the random decision to make the sandclock more logical
	        if (rand() % 2 == 0) {
	            world[x][y] = 0; world[lX][lY] = 1;
	        } else {
	            world[x][y] = 0; world[rX][rY] = 1;
	        }
	    }
	    else if (can_left) {
	        world[x][y] = 0; world[lX][lY] = 1;
	    }
	    else if (can_right) {
	        world[x][y] = 0; world[rX][rY] = 1;
	    }
	    else{return;}
}

//Func: also move the led but the upper matrix when we are at state 0 will not overflow to the lower
//matrix. If we dont have this func, the led will flow out of control to the lower matrix when we finish every frame
void move_particle_state0upper(int x, int y) //=state2below
{
	if (world[x][y] == 0) return;
	int lX= x+leftX;
	int lY= y+leftY;

	int rX= x+rightX;
	int rY= y+rightY;

	bool can_left  = can_move_to_state0upper(lX, lY);
	bool can_right = can_move_to_state0upper(rX, rY);

	    if (can_left && can_right) {
	        if (rand() % 2 == 0) {
	            world[x][y] = 0; world[lX][lY] = 1;
	        } else {
	            world[x][y] = 0; world[rX][rY] = 1;
	        }
	    }
	    else if (can_left) {
	        world[x][y] = 0; world[lX][lY] = 1;
	    }
	    else if (can_right) {
	        world[x][y] = 0; world[rX][rY] = 1;
	    }
	    else{return;}
}


void move_particle_state2upper(int x, int y) //=state2below
{
	if (world[x][y] == 0) return;
	int lX= x+leftX;
	int lY= y+leftY;

	int rX= x+rightX;
	int rY= y+rightY;

	bool can_left  = can_move_to_state0below(lX, lY);
	bool can_right = can_move_to_state0below(rX, rY);

	    if (can_left && can_right) {

	    	if (rand() % 2 == 0) {
	            world[x][y] = 0; world[lX][lY] = 1;
	        } else {
	            world[x][y] = 0; world[rX][rY] = 1;
	        }
	    }
	    else if (can_left) {

	    	world[x][y] = 0; world[lX][lY] = 1;
	    }
	    else if (can_right) {

	    	world[x][y] = 0; world[rX][rY] = 1;
	    }
	    else{return;}
}

void move_particle_state0below(int x, int y) //=state2 upper
{
	if (world[x][y] == 0) return;
	int next_x = x + fwdX;
	int next_y = y + fwdY;

	int lX= x+leftX;
	int lY= y+leftY;

	int rX= x+rightX;
	int rY= y+rightY;
	if(can_move_to(next_x,next_y))
	{
		world[x][y] = 0;
		world[next_x][next_y] = 1;
		return;
	}

	bool can_left  = can_move_to_state0below(lX, lY);
	bool can_right = can_move_to_state0below(rX, rY);

	    if (can_left && can_right) {

	        if (rand() % 2 == 0) {
	            world[x][y] = 0; world[lX][lY] = 1;
	        } else {
	            world[x][y] = 0; world[rX][rY] = 1;
	        }
	    }
	    else if (can_left) {

	        world[x][y] = 0; world[lX][lY] = 1;
	    }
	    else if (can_right) {

	        world[x][y] = 0; world[rX][rY] = 1;
	    }
	    else{return;}

}
void move_particle_state2below(int x, int y) //=state2 upper
{
	if (world[x][y] == 0) return;

	int next_x = x + fwdX;
	int next_y = y + fwdY;

	int lX= x+leftX;
	int lY= y+leftY;

	int rX= x+rightX;
	int rY= y+rightY;
	if(can_move_to(next_x,next_y))

	{
		world[x][y] = 0;
		world[next_x][next_y] = 1;
		return;
	}

	bool can_left  = can_move_to_state0upper(lX, lY);
	bool can_right = can_move_to_state0upper(rX, rY);

	    if (can_left && can_right) {

	        if (rand() % 2 == 0) {
	            world[x][y] = 0; world[lX][lY] = 1;
	        } else {
	            world[x][y] = 0; world[rX][rY] = 1;
	        }
	    }
	    else if (can_left) {

	        world[x][y] = 0; world[lX][lY] = 1;
	    }
	    else if (can_right) {

	        world[x][y] = 0; world[rX][rY] = 1;
	    }
	    else{return;}
	//Đứng yên
}


/*
 * Func: Scan the matrices and move the particle with the logic
 * */

//We have the difference with the state 0 and state 2 because i already explain the reason in the boundary func

//state0: scan the area that dont have led to the area that have led
//every state has the same logic to prevent the line effect when update
void Update_State_0_Down(void) {


    for (int y = 0; y <= 7; y++) {
        for (int x = 15; x >= 8; x--) {
            move_particle_state0below(x, y);
        }
    }
    for (int y = 8; y <= 15; y++) {
        for (int x = 7; x >= 0; x--) {
           move_particle_state0upper(x, y);
         }
     }
}

void Update_State_2_Up(void) {


    for (int y = 0; y <= 7; y++) {
        for (int x = 8; x <=15; x++) {
            move_particle_state2upper(x, y);
        }
    }
    for (int y = 15; y >=8; y--	) {
            for (int x = 7; x >= 0; x--) {
            move_particle_state2below(x, y);
         }
     }
}
void Update_State_1_Right(void) {
    for (int x = 0; x <=15; x++) {
        for (int y = 0; y <= 15; y++) {
            move_particle(x, y);
        }
    }
}

void Update_State_3_Left(void) {
    for (int x = 15; x >=0; x--) {
        for (int y = 15; y >=0; y--) {
            move_particle(x, y);
        }
    }
}

//Choose the logic depends on state
void Update_World(int state) {
    switch (state) {
        case 0:
            Update_State_0_Down();
            break;
        case 1:
            Update_State_1_Right();
            break;
        case 2:
            Update_State_2_Up();
            break;
        case 3:
            Update_State_3_Left();
            break;
        default:
            Update_State_0_Down();
            break;
    }
}


/*---This function will take the data from world and then latch it into
 *  1 byte which contains the state of every led in that row-------------*/

void pack_data(void){
	for(int i=0; i<16; i++) display_buffer[i]=0;

	for(int y=0; y<16; y++){
		int col=(y<8)?8:0;
		for (int x=0; x<8; x++){
			if (world[col+x][y]==1){
				display_buffer[y]|=(1<<(x));
			}
		}
	}

}

/*----------This function will take the data we have packed in the previous func
 *  to latch it out MAX7219-------------*/

void flush_to_max7219(void){
	for(int row=0; row<16; row++){
		max_write(row+1, display_buffer[row]);
	}
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  MAX7219_Init(0x05);
  while (MPU6050_Init(&hi2c1) == 1);

  MPU6050_Read_All(&hi2c1, &MPU6050);

  Ax= MPU6050.Ax;
  Ay= MPU6050.Ay;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 //fill the upper led
  for (int x = 0; x <= 6; x++) {
      for (int y = 8; y <= 15; y++) {
          world[x][y] = 1;
      }
  }

  int current_state = 0;
  // Because we dont want the MPU6050 to shutdown when transfer data so we use GetTick func
  uint32_t last_mpu_time = 0;
  uint32_t last_physics_time = 0;
  uint32_t last_flow_time = 0;
  while (1)
  {

	  uint32_t current_time = HAL_GetTick();

	        if (current_time - last_mpu_time >= readtime) {

	            current_state = get_orientation_state(Ay, Ax);
	            set_gravity_by_state(current_state);

	            last_mpu_time = current_time;
	        }

	        // the time for the particle to move 1 step
	        if (current_time - last_physics_time >= 100) {

	            Update_World(current_state);
	            pack_data();
	            flush_to_max7219();

	            last_physics_time = current_time;
	        }

	        // spawn led time
	        if (current_time - last_flow_time >= droptime) {

	            source_sink(current_state);

	            last_flow_time = current_time;
	        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
