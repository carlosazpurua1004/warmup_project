# warmup_project
* **Driving in a Square**
  * This problem requires the robot to be programmed so that it gracefully switch between traveling a fixed distance in a straight line, and turning 90 degrees. My approach was to create a driver class with two states, 'foward' and 'turn,' which the robot switches between after a certain number of iterations of the publishing while loop. Depending on the state the robot is in, it will either adjust the linear velocity and keep angular at 0, or vice versa.
  * The SquareDriver object is initialized, with some input variables that are useful for adjusting behavior, like a maximum speed, turn speed, and an initial state (foward or turn). Then the run function commences the publishing while loop. In this loop, the robot switches between going foward and turning every 5 seconds (25 cycles). We use a fixed angular velocity, but in order to avoid drifting and other issues driving foward, we gradually increase and then decrease speed while in a drive state. This speed for a given cycle is computed by the 'compute_speed' function, which can determine the speed from a counter of the number of cycles of the while loop. 
  * ![image](https://user-images.githubusercontent.com/63179479/113522888-b7a3c480-9569-11eb-8df9-ed8f9b2b82cc.png)
