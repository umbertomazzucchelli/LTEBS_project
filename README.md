# Electronic Technologies and Biosensors Laboratory

## Academic Year 2021/2022 - II Semester

## Final Project - Project 4 - Respiratory/Heart Rate Monitor

## Objective

In this project, the aim was to develop a simple **Respiratory/Heart Rate Monitor** based on the PSoC and a 3-axis accelerometer LIS3DH. The device implements an algorithm for respiratory and heart rate computation, and then it allows data visualization on a custom GUI. Communication between PSoC and PC has been made possible by using a HC05 Bluetooth module, in order to make the device wearable and use less cables possible.  
The PSoC communicates with the LIS3DH thanks to I2C communication, to acquire data at the best full-scale range possible, which has been set to $\pm 2G$, and with sampling frequency of 50 Hz.  
Finally, the whole PCB is powered thanks to a $9V$ battery, which allows the device to be portable and light. 

## Repo structure

### C code

The *PSoC Code* folder contains all the files used to program the PSoC: in particular, all the functions and variables have been reordered in the proper file, allowing to increase the main.c file readibility.  
This code basically sets the proper configuration for the accelerometer registers at startup, making it ready for the data acquisition. All the different phases are managed thanks to a UART, that enables the different operations, according to the signals that come from the GUI. So, at the beginning the accelerometer is set and ready, once the PSoC receives the *start* signal from the GUI it begins to acquire data and to transmit them.  

Regarding the data acquisition, as previously mentioned the accelerometer has been set to a sampling frequency of 50 Hz, according to the Shannon's theorem and to the literature. In addition to that, the accelerometer operates in **FIFO mode**, so it accumulates all the samplings in the FIFO register (which contains up to 32 samples) and then it stops getting new data until the register has been emptied, allowing to reduce the computational load on the CPU and so the battery consumption, with respect to **bypass** mode, that sends the data in a continuous stream.  
In order to read the FIFO register, the PSoC checks regularly the value of the *overrun bit*, that becomes 1 once the FIFO register is full and returns to 0 when it has been entirely read. 
