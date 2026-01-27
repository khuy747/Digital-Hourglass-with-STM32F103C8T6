# Digital Hourglass using STM32F103C8T6
This is the project using the MPU6050 to control 2 8x8 LED Matrices Cascaded 

## Materials 
**MCU** :STM32F103C8T6 

**Sensor** :MPU6050 

**Display** : 2 LED Matrices with the module of IC MAX7219 

**Power** : 1 cell of Li-on battery with capacity 2000mAh with 3,7V and Boost circuit combined with charging circuit 

> [!IMPORTANT]
> My project has the their own way to display MPU6050 so make sure to change the direction in the `int get_orientation_state(double Ay, double Ax)` by measuring your own status
> 
> Another thing to take consideration is the MPU address in **mpu6051.c** file in the `if (check == 112) // 0x68 will be returned by the sensor if everything goes well
` line. If you use the authentic MPU6050, you should change 112 to 0x68. 

## How to use? 
