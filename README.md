# AWR1843 Read Data (Python 3)

Python program to read and plot the data in real time from the **AWR1843** mmWave radar board (Texas Instruments, MMWAVE SDK 3). The program has been tested with Windows and Raspberry Pi and is based on the Matlab demo from Texas Instruments.

First, the program configures the Serial ports and sends the CLI commands defined in the configuration file to the radar. Next, the data comming from the radar is parsed to extract the 3D position and doppler velocity of the reflected points. Finally, the 2D position of the reflected points is shown in a scatter plot.

Although it works correctly, the program is still in development, so if you have comments or questions, please feel free to comment.

## Required Python packages
* **numpy**: For the array calculations.
* **serial**: To read the serial data from the radar.
* **time**: To wait until more data is generated.
* **pyqtgraph**: For the scatter plot showing the 2D position of the reflected points.

## Program Functions
* **serialConfig()**: Configures the serial ports and sends the CLI commands to the radar. It outputs the serial objects for the data and CLI ports.
* **parseConfigFile()**: Parses the configuration file to extract the configuration parameters. It returns the **configParameters** dictionary with the extracted parameters.
* **readAndParseData18xx()**: It reads the data from the data serial port and parses the recived buffer to extract the data from the **Detected objects** package only. Othe package types (range profile, range-azimuth heat map...) could be done similarly but have not been implemented yet. This functions returns a boolean variable (**dataOK**) that stores if the data has been correctly, the frame number and the **detObj** dictionary with the number of detected objects, 3D position (m) and doppler velocity (m/s).
* **update()**: Runs the **readAndParseData18xx()** function to read the current data and if the data has been read correctly the scatter with the 2D position is updated.

## HOW TO USE
* Download the required packages.
* Change the name of the configuration file (.cfg).
* Change the serial ports.
* If **not all the antennas** are being used, then change the value of **numRxAnt** and **numTxAnt**.
* Run the program.
* The data of each frame with the position and velocities of the reflected points is stored in the **detObj** dictionary. Each frame data is stored in the **frameData** dictionary array.
