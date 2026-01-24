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
#define dev_num 2 //so luong led ma tran
#define droptime 1000 //Thời gian xuất hiện led
#define readtime 200 // Thời gian đọc MPU


uint8_t display_buffer[16]={0}; //Su dung cho viec latch data tu world thanh 1 cum 8bit va luu vao day

/*----------Khởi tạo ma trận world 16x16 để chứa data làm việc với 0,1-----------------*/
uint8_t world[16][16];

double Ax, Ay;
MPU6050_t MPU6050;

//ghi trạng thái hiện tại vào ma trận này !!

/*---------Hàm send data sơ cấp nhất để sử dụng cho gửi hàm Init đỡ rối------------*/

void MAX7219_Send(uint8_t add, uint8_t data) {
	uint16_t writeData = (add<<8)|data;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Kéo CS xuống
    for (int i=0; i<dev_num; i++){
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 100);
    }// Vì có 2 led ma trận nên khởi tạo 2 lần
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Kéo CS lên
}

/* ----Hàm send data theo row và binary và phân chia ra từ 1-8 là của led 1, 9-16 là của led 2------*/


int max_write (int row, uint8_t data)
{
	int devTarget = (row - 1) / 8;  // find out which is the actual max, where we need to write the data
	int offset = devTarget * 8;  // The offset of the start byte for the devTarget in the buffer
	uint16_t writeData = 0;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);  // Select the slave
	for (int dev = 0; dev < dev_num; dev++)   // for loop for all the max connected
		// vif cos 2 thiet bi nen loop toi khi nao thoa man thi thoi
	{
		if (dev == devTarget)  // if this the target
		{
			writeData = ((row - offset)<<8)|data;
			// truyền dữ liệu tới dòng thứ row nếu đếm theo logic...
			//Cụ thể thì ở đây dùng để shift bit của dòng cần truyền và data cho chung 1 thể
			HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 1000);
		}
		else
		{
			writeData = 0;  // send dữ liệu NO-OP để daisy chain
			HAL_SPI_Transmit(&hspi1, (uint8_t *)&writeData, 1, 1000);
		}
	}
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);  // disable the slave
	return writeData;
}


/*Khởi tạo 2 ma trận*/
void MAX7219_Init(uint8_t intensity) {
    HAL_Delay(100); // chờ tí cho an toàn
    MAX7219_Send(0x0C, 0x01); // Wake up
    MAX7219_Send(0x0F, 0x00); // Tắt Test mode
    MAX7219_Send(0x09, 0x00); // No decode
    MAX7219_Send(0x0B, 0x07); // Scan đủ 8 hàng
    MAX7219_Send(0x0A, intensity); // Độ sáng

    // Xóa rác màn hình khi bật nguồn
    for(int i = 1; i <= 8; i++) {
          MAX7219_Send(i, 0x00);
      }
}




/* ----------Tạo hàm nhận angle để nhận trạng thái---------------- */
int get_orientation_state(double Ay, double Ax) {
    // 1. Trạng thái THẲNG (0 độ hoặc 360 độ)
    if (Ay<0&&Ax>=0) {
        return 0;
    }

    // 2. Trạng thái NGHIÊNG PHẢI (90 độ)
    else if (Ay>=0 &&Ax>=0) {
        return 1;
    }

    // 3. Trạng thái CHỔNG NGƯỢC (180 độ)
    else if (Ay>=0&&Ax<0) {
        return 2;
    }

    // 4. Trạng thái NGHIÊNG TRÁI (270 độ) - Trường hợp còn lại
    else {
        return 3;
    }
}


/*-----------Tạo hàm tạo điểm led (nguồn cát) ------------*/
/*-----------Taoj hàm quét xem led nào ở "trên mặt" để bỏ 1 trong các led đó */
//Ta gộp chung lại vì sau đó ta có hàm update nên là không bị vấn đề gì. Ta sẽ tạo 1 source ở 8,7 khi 0 độ
//và sẽ có 1 sink ở 7,8 mất 1 led cùng lúc với tạo led tạo ra hiệu ứng rơi đồng hồ cát.

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
	//Nếu như ở 7,8 có đèn và ở 8,7 trống thì mình spawn led! và xóa led cũ đi
if (world[sink_x][sink_y] == 1 && world[source_x][source_y] == 0) {

        world[sink_x][sink_y] = 0;
        world[source_x][source_y] = 1;
	}
}


/*-------Hàm latch data từ 16x16 ra 1 byte 8bit chứa trạng thái-------------*/

void pack_data(void){
	for(int i=0; i<16; i++) display_buffer[i]=0;

	for(int y=0; y<16; y++){
		int col=(y<8)?8:0; //if y<8 thì start col =8. y từ 8 đến 15 thì start col =0
		for (int x=0; x<8; x++){
			if (world[col+x][y]==1){
				display_buffer[y]|=(1<<(x));
			}
		}
	}

}
/*---------Sử dụng cho việc lấy data ra 1 ma trận khác để tránh xung đô--------------*/
void copy_world(void) {
    for (int x = 0; x < 16; x++) {
        for (int y = 0; y < 16; y++) {
            world_next[x][y] = world[x][y];
        }
    }
}


void swap_world(void){
	for (int x = 0; x < 16; x++) {
	        for (int y = 0; y < 16; y++) {
	            world[x][y] = world_next[x][y];
	        }
	    }
}
/*----------Tạo hàm DISPLAY đẩy hết data được cập nhật mỗi lần ở world ra-------------*/

void flush_to_max7219(void){
	for(int row=0; row<16; row++){
		max_write(row+1, display_buffer[row]);
	}
}


/*------------Tạo biên giới để hạt cát biết điểm dừng------------*/

bool check_inbound(int x, int y) {
    if (x < 0 || x > 15 || y < 0 || y > 15) return false;

    // Bình trên trái
    if (x >= 0 && x <= 7 && y >= 8 && y <= 15) return true;

    // Bình dưới phải
    if (x >= 8 && x <= 15 && y >= 0 && y <= 7) return true;

    return false; // Còn lại là tường
}

bool check_inbound_state0upper(int x, int y) // =state2below
{
		if (x < 0 || x > 15 || y < 0 || y > 15) return false;

	    // Bình dưới phải
	    if (x>=0&& x<=7&&y>=8&&y<=15) return true;
	    return false;
}
bool check_inbound_state0below(int x, int y) // =state2upper
{
		if (x < 0 || x > 15 || y < 0 || y > 15) return false;

	    // Bình trên trái
	    if (x>=8&& x<=15&&y>=0&&y<=7) return true;
	    return false;
}

//desired result: state0 thì có inbound trên là mặt trên: state2 thì là mặt dưới

//--> cái state quyết định inbound



/*-------------Tạo hàm để check xem hạt có thể di chuyển tiếp hay không-------------*/
bool can_move_to(int tx, int ty) {
    if (!check_inbound(tx, ty)) return false; // Đụng tường/Biên
    if (world[tx][ty] == 1) return false;     // Đụng hạt cát khác
    return true;
}

bool can_move_to_state0upper(int tx, int ty) //=can_move_to_state2below
{
    if (!check_inbound_state0upper(tx, ty)) return false; // Đụng tường/Biên
    if (world[tx][ty] == 1) return false;     // Đụng hạt cát khác
    return true;
}
bool can_move_to_state0below(int tx, int ty) //=can_move_to_state2upper
{
    if (!check_inbound_state0below(tx, ty)) return false; // Đụng tường/Biên
    if (world[tx][ty] == 1) return false;     // Đụng hạt cát khác
    return true;
}

//--> tạo ra thêm 2 cái inbound từ đó ra được 2 cái move mới dành cho 2 trường hợp state=0 và 2

/*-----------Tạo hàm rơi 1 hạt cát ở điểm x,y theo từng trườnghợp xoay 1 góc bao nhiêu------------*/
// 1. Hướng đi thẳng (Forward)
int8_t fwdX, fwdY;
// 2. Hướng trượt sang trái (Left)
int8_t leftX, leftY;
// 3. Hướng trượt sang phải (Right)
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

            leftX = 1; leftY = 0;  // Trượt sang trái

            rightX = 0; rightY = 11; // Trượt lên trên
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

/*------------Tạo hàm để check bên trái và bên phải và phía dưới--------------------*/
void move_particle(int x, int y){
	if (world[x][y] == 0) return; //không có cát thì không di chuyển cái này
	//Đi thẳng
	int next_x = x + fwdX;
	int next_y = y + fwdY;
	//Đi trái
	int lX= x+leftX;
	int lY= y+leftY;
	//Đi phải
	int rX= x+rightX;
	int rY= y+rightY;
	if(can_move_to(next_x,next_y)) //Đi thẳng
	{
		world[x][y] = 0;
		world[next_x][next_y] = 1;
		return;
	}

	bool can_left  = can_move_to(lX, lY);
	bool can_right = can_move_to(rX, rY);

	    if (can_left && can_right) {
	        // Cả 2 bên đều thoáng -> Random chọn 1
	        if (rand() % 2 == 0) {
	            world[x][y] = 0; world[lX][lY] = 1;
	        } else {
	            world[x][y] = 0; world[rX][rY] = 1;
	        }
	    }
	    else if (can_left) {
	        // Chỉ bên trái thoáng
	        world[x][y] = 0; world[lX][lY] = 1;
	    }
	    else if (can_right) {
	        // Chỉ bên phải thoáng
	        world[x][y] = 0; world[rX][rY] = 1;
	    }
	    else{return;}
	//Đứng yên
}

//Dùng cho state0 mặt trên để nó không chảy lố
void move_particle_state0upper(int x, int y) //=state2below
{
	if (world[x][y] == 0) return; //không có cát thì không di chuyển cái này
	//Đi thẳng

	//Đi trái
	int lX= x+leftX;
	int lY= y+leftY;
	//Đi phải
	int rX= x+rightX;
	int rY= y+rightY;
	/*if(can_move_to(next_x,next_y)) //Đi thẳng
	{
		world[x][y] = 0;
		world[next_x][next_y] = 1;
		return;
	}*/

	bool can_left  = can_move_to_state0upper(lX, lY);
	bool can_right = can_move_to_state0upper(rX, rY);

	    if (can_left && can_right) {
	        // Cả 2 bên đều thoáng -> Random chọn 1
	        if (rand() % 2 == 0) {
	            world[x][y] = 0; world[lX][lY] = 1;
	        } else {
	            world[x][y] = 0; world[rX][rY] = 1;
	        }
	    }
	    else if (can_left) {
	        // Chỉ bên trái thoáng
	        world[x][y] = 0; world[lX][lY] = 1;
	    }
	    else if (can_right) {
	        // Chỉ bên phải thoáng
	        world[x][y] = 0; world[rX][rY] = 1;
	    }
	    else{return;}
	//Đứng yên
}


void move_particle_state2upper(int x, int y) //=state2below
{
	if (world[x][y] == 0) return; //không có cát thì không di chuyển cái này
	//Đi thẳng

	//Đi trái
	int lX= x+leftX;
	int lY= y+leftY;
	//Đi phải
	int rX= x+rightX;
	int rY= y+rightY;
	/*if(can_move_to(next_x,next_y)) //Đi thẳng
	{
		world[x][y] = 0;
		world[next_x][next_y] = 1;
		return;
	}*/

	bool can_left  = can_move_to_state0below(lX, lY);
	bool can_right = can_move_to_state0below(rX, rY);

	    if (can_left && can_right) {
	        // Cả 2 bên đều thoáng -> Random chọn 1
	        if (rand() % 2 == 0) {
	            world[x][y] = 0; world[lX][lY] = 1;
	        } else {
	            world[x][y] = 0; world[rX][rY] = 1;
	        }
	    }
	    else if (can_left) {
	        // Chỉ bên trái thoáng
	        world[x][y] = 0; world[lX][lY] = 1;
	    }
	    else if (can_right) {
	        // Chỉ bên phải thoáng
	        world[x][y] = 0; world[rX][rY] = 1;
	    }
	    else{return;}
	//Đứng yên
}

void move_particle_state0below(int x, int y) //=state2 upper
{
	if (world[x][y] == 0) return; //không có cát thì không di chuyển cái này
	//Đi thẳng
	int next_x = x + fwdX;
	int next_y = y + fwdY;
	//Đi trái
	int lX= x+leftX;
	int lY= y+leftY;
	//Đi phải
	int rX= x+rightX;
	int rY= y+rightY;
	if(can_move_to(next_x,next_y)) //Đi thẳng
	{
		world[x][y] = 0;
		world[next_x][next_y] = 1;
		return;
	}

	bool can_left  = can_move_to_state0below(lX, lY);
	bool can_right = can_move_to_state0below(rX, rY);

	    if (can_left && can_right) {
	        // Cả 2 bên đều thoáng -> Random chọn 1
	        if (rand() % 2 == 0) {
	            world[x][y] = 0; world[lX][lY] = 1;
	        } else {
	            world[x][y] = 0; world[rX][rY] = 1;
	        }
	    }
	    else if (can_left) {
	        // Chỉ bên trái thoáng
	        world[x][y] = 0; world[lX][lY] = 1;
	    }
	    else if (can_right) {
	        // Chỉ bên phải thoáng
	        world[x][y] = 0; world[rX][rY] = 1;
	    }
	    else{return;}
	//Đứng yên
}
void move_particle_state2below(int x, int y) //=state2 upper
{
	if (world[x][y] == 0) return; //không có cát thì không di chuyển cái này
	//Đi thẳng
	int next_x = x + fwdX;
	int next_y = y + fwdY;
	//Đi trái
	int lX= x+leftX;
	int lY= y+leftY;
	//Đi phải
	int rX= x+rightX;
	int rY= y+rightY;
	if(can_move_to(next_x,next_y)) //Đi thẳng
	{
		world[x][y] = 0;
		world[next_x][next_y] = 1;
		return;
	}

	bool can_left  = can_move_to_state0upper(lX, lY);
	bool can_right = can_move_to_state0upper(rX, rY);

	    if (can_left && can_right) {
	        // Cả 2 bên đều thoáng -> Random chọn 1
	        if (rand() % 2 == 0) {
	            world[x][y] = 0; world[lX][lY] = 1;
	        } else {
	            world[x][y] = 0; world[rX][rY] = 1;
	        }
	    }
	    else if (can_left) {
	        // Chỉ bên trái thoáng
	        world[x][y] = 0; world[lX][lY] = 1;
	    }
	    else if (can_right) {
	        // Chỉ bên phải thoáng
	        world[x][y] = 0; world[rX][rY] = 1;
	    }
	    else{return;}
	//Đứng yên
}

/*----------Taọ hàm update world -------------*/
/*----------Tạo hàm scan qua các điểm để xem là có led hay không 	-------------*/
/*	---------------------------
 * |							|. Cát rơi down --> quét từ dưới lên trên và phải sang trái
 * |							| 		   up --> quét từ trên xuống dưới và từ trái sang phải
 * |							|			rơi phải: quét từ dưới lên và trái sang phải
 * |							|			rơi trái: từ trên xuống và phải sang
 * |							|
 * |							|
 * |							|							|
 *   ---------------------------
 * */
void Update_State_0_Down(void) {


    for (int y = 0; y <= 7; y++) {
        for (int x = 15; x >= 8; x--) {
            move_particle_state0below(x, y); //quét ở dưới để tránh lấy phần tử ở trên
        }
    }
    for (int y = 8; y <= 15; y++) {
        for (int x = 7; x >= 0; x--) {
           move_particle_state0upper(x, y); //quét ở trên tránh nhầm ở dưới
         }
     }
}
//Cần tạo thêm 1 hàm update state 0 mặt trên. Có boundary khác bình thường
//--> Tạo thêm hàm move_particle có boundary khác!

//Nếu update cái này thì hàm pạck_data liệu có còn lưu như bình thường?

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

//Nên dựa vào state để scan
//Nếu state==0 thì cát rơi từ trên xuống nên là quét từ dưới lên trên
//state==1 thì scan từ phải sang trái
//state==2 từ trên xuống dưới
//state==3 từ trái sang phải
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
            Update_State_0_Down(); // Mặc định
            break;
    }
}
//Nói scan cho sang chứ bản chất là hàm for !



/*----Hàm thực hiện dịch bit của ma trận world-----*/


/*---------Luuư đồ hoạt động thì input vẫn là MPU6050--> Từ đây quyết định state
 * --> Từ state thì quyết định ra tới các hướng led di chuyển cũng scan như nào
 * --> Từ đó ta có thể thao tác từng bước lên ma trận 16x16 đã có trong world
 * --> Cuối cùng thì ta bốc hết các dữ liệu đó và latch ra ngoài là hoàn thành! */



/*Thêm hàm lấy MPU vào, thêm thư viện vào--> Thay các angle thành KalmanAngle.
 * Ngoai ra ta cần thêm 1 thời gian chờ ổn định của MPU để kết quả lấy đưuojc không bị lệch
 * Timf các khoảng của MPU để tích hợp vào
 * */


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
//đổ đầy led đầu
for (int x = 0; x <= 6; x++) {
      for (int y = 8; y <= 15; y++) {
          world[x][y] = 1;
      }
  }

int current_state = 0;
// Khởi tạo biến thời gian
  uint32_t last_mpu_time = 0;
  uint32_t last_physics_time = 0;
  uint32_t last_flow_time = 0;
  while (1)
  {

	  uint32_t current_time = HAL_GetTick(); // Lấy thời gian hiện tại (tính bằng ms)

	        // Đọc cảm biến
	        if (current_time - last_mpu_time >= readtime) {

	            MPU6050_Read_All(&hi2c1, &MPU6050); // Đọc giá trị mới nhất
	            Ax = MPU6050.Ax;
	            Ay = MPU6050.Ay;

	            // Cập nhật hướng trọng lực
	            current_state = get_orientation_state(Ay, Ax);
	            set_gravity_by_state(current_state);

	            last_mpu_time = current_time;
	        }

	        // Tốc độ rơi của hạt cát
	        if (current_time - last_physics_time >= 100) {

	            Update_World(current_state);
	            pack_data();
	            flush_to_max7219();

	            last_physics_time = current_time;
	        }

	        // Tương đương HAL_Delay(1000) của bạn, điều chỉnh tốc độ xuất hiện hạt cát
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
