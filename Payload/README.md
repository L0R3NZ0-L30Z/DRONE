# Code
## Summary:
This is the code that drives the microprocesor that flyes the drone.

## Functions:
 The code is subdivided in functions to better organize the program.
 
#### WIFI start
 It does all the starting conections to prepare to recieve and send data.
 
 ````Inicio_Wifi();````
 
#### WIFI conection
 It does the recieving and sending data from the board to the App and viceversa.
 
 ````Coneccion_Wifi();````

#### Trim
 It clasifies and separates the info recieved by the App.
 
````Trim(int val);````
