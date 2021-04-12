# warmup_project
* **Driving in a Square**
  * This problem requires the robot to be programmed so that it gracefully switch between traveling a fixed distance in a straight line, and turning 90 degrees. My approach was to create a driver class with two states, 'foward' and 'turn,' which the robot switches between after a certain number of iterations of the publishing while loop. Depending on the state the robot is in, it will either adjust the linear velocity and keep angular at 0, or vice versa.
  * The SquareDriver object is initialized, with some input variables that are useful for adjusting behavior, like a maximum speed, turn speed, and an initial state (foward or turn). Then the run function commences the publishing while loop. In this loop, the robot switches between going foward and turning every 5 seconds (25 cycles). We use a fixed angular velocity, but in order to avoid drifting and other issues driving foward, we gradually increase and then decrease speed while in a drive state. This speed for a given cycle is computed by the 'compute_speed' function, which can determine the speed from a counter of the number of cycles of the while loop. 
  * ![square_drive](https://user-images.githubusercontent.com/63179479/113523140-9cd24f80-956b-11eb-9edb-7472dedde20c.gif)
* **Follower**
  * a
  * b
  * ![follower](https://user-images.githubusercontent.com/63179479/114346576-52cc0980-9b29-11eb-9bcd-fc5043e3b059.gif)

* ** Wall Follower**
  * a
  * b
  * ![wall_follower](https://user-images.githubusercontent.com/63179479/114346555-4ba4fb80-9b29-11eb-8d46-9779740cc90d.gif)

* **Challenges** \
 c
* **Future Work** \
 f
* ** Take aways **
  * a
  * b
