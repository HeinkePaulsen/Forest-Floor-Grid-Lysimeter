# Forest-Floor-Grid-Lysimeter
In this git you can find all the information you need to build the Forest Floor Grid-Lysimeter.
It contains STL-files for all the 3D printed parts, information about the controller board, and code to run the lysimeter.

# Contents
## STLs
- TB_Frame: Frame to hold the tipping bucket
- TippingBucket: tipping bucket, use print feature "ironing" (*important*)
- connector_frame: Connector for steel frame corners
- cuvette_bottom & cuvette_top: measurement unit (MU), that controls water flow and allows for water quality measurements
- funnel: funnel to collect water to MU
- mounting: Connects MU to  frame and TB_Frame
  
## Code
- serial_output: Programm to test functions of the lysimeter, puts all the readings to serial monitor, adaptable timesteps
- sd_modus: Programm to run the lysimeter autonomously, readings are safd to sd-card, adaptable timesteps
- LC_calibration: Programm to calibrate LCs

## Board

