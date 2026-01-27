# Digital Hourglass using STM32F103C8T6
This is the project using the MPU6050 to control 2 8x8 LED Matrices Cascaded with State machine, Time-sliced task and Superloop 

## Materials in this project 
**1.MCU** :STM32F103C8T6 

**2.Sensor** :MPU6050 [Register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) 

**3.Display** : 2 LED Matrices with the module of IC MAX7219 (Use this [Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max7219-max7221.pdf) to know how the IC MAX7219 works. It needs a "No-operation data" to use the cascade mode, like when you cascade 2 matrices and you want to only communicate with the second one but not the first, you need to send the No-op data to the first matrix )

**4.Power** : 1 cell of Li-on battery with capacity 2000mAh with 3,7V and Boost circuit combined with charging circuit [Schematic](https://i.pinimg.com/736x/7d/34/64/7d3464c19563e37884a307127ac778dd.jpg) 

## Configuration 

1. `Debug -> Serial Wire`
2. `Connectivity -> I2C1 -> I2C -> Fast mode and 400kHz clock`
3. `Connectivity -> SPI1 -> Transmit only master -> Data size: 16bit -> MSB First -> Prescaler 64 -> CPOL LOW , CPHA 1 Edge `
4. `Clock config -> HCLK 64MHz`


> [!IMPORTANT]
> My project has the their own way to display MPU6050 so make sure to change the direction in the `int get_orientation_state(double Ay, double Ax)` by measuring your own status
> 
> Another thing to take consideration is the MPU address in **mpu6051.c** file in the `if (check == 112) // 0x68 will be returned by the sensor if everything goes well
` line. If you use the authentic MPU6050, you should change 112 to 0x68. (And if not, check the   

## How it works 

### First: I have to give you the logic of my core algorithm 

If you have seen the sand falling algorithm my algorithm is like that. The code scans from low to high and check if there is a particle 
1. If there is a particle at coordinate (x,y) the particle move to (x,y-1) if there is no particle below like this picture

![sand_simulation](sand_simulation.webp)

Then when it meets other particle it will have 2 option to go left or right so the coordinate will be (x-1,y-1) or (x+1,y-1). Else it will stay the 

### Second: How i arrange the matrices in the code 
This project arrange the matrices with the right picture order. So to make the calculation easier i rotate the matrices 45 degrees counter clockwise and have the left picture. And the gravity now according to the order has the vector (1,-1) so it will flow from high to low . Follow that logic the gravity in the state 90 degrees is (1,1) and so on. 

![Image](principle_matrix.jfif)


The system operates based on a non-blocking loop (using `HAL_GetTick`) to simulate sand physics in real-time:

1. **Virtual Matrix Representation**: A 16x16 matrix is defined to simulate the coordinate system where each LED represents a "sand particle".
2. **Sensor Data Acquisition**: Raw data from the **MPU6050** is read via **I2C1** to determine the tilt angle (Ax, Ay).
3. **State Determination**: Based on the tilt angle, the system defines 4 orientations (Up, Down, Left, Right) to set the "gravity vector".
4. **Sand Flow Logic (Source/Sink)**: The system checks if sand should flow from the upper chamber to the lower chamber based on the current orientation.
5. **Boundary & Collision Check**: Before moving, each particle checks if the target coordinate is within the matrix boundary and if it is already occupied.
6. **Optimized Scanning**: To avoid the "line-effect" (particles moving multiple steps in one frame), the matrix is scanned in a specific order (e.g., from bottom to top for downward gravity).
7. **Display Rendering**: The 16x16 world data is "packed" into a byte-stream and sent to the **MAX7219** drivers via **SPI1** to be displayed on the 8x8 LED matrices.


## Project Structure 
`Code_STM32` : The STM32 Code 

`Demo_video`: The video i use this code to run in my hardware 

`Schematic`: The way i connect all the material 

## Demo run 
![Video](Demo_Video.mp4)

## How to use? 
1. Clone this repo
2. Open STM32CubeIDE and click **File>Import**
3. Check the configuration in the `.ioc` file
4. Compile and run
