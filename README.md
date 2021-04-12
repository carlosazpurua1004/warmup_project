# warmup_project
* **Driving in a Square**
  * This problem requires the robot to be programmed so that it gracefully switch between traveling a fixed distance in a straight line, and turning 90 degrees. My approach was to create a driver class with two states, 'foward' and 'turn,' which the robot switches between after a certain number of iterations of the publishing while loop. Depending on the state the robot is in, it will either adjust the linear velocity and keep angular at 0, or vice versa.
  * The SquareDriver object is initialized, with some input variables that are useful for adjusting behavior, like a maximum speed, turn speed, and an initial state (foward or turn). Then the run function commences the publishing while loop. In this loop, the robot switches between going foward and turning every 5 seconds (25 cycles). We use a fixed angular velocity, but in order to avoid drifting and other issues driving foward, we gradually increase and then decrease speed while in a drive state. This speed for a given cycle is computed by the 'compute_speed' function, which can determine the speed from a counter of the number of cycles of the while loop. 
  * ![square_drive](https://user-images.githubusercontent.com/63179479/113523140-9cd24f80-956b-11eb-9edb-7472dedde20c.gif)
* **Follower**
  * The follower robot iterates through all the scan data to find the closest object, its distance, and then to turn towards that object, and move towards the object (or away if it is too close). I allow faster speeds when the robot is aligned with the object is following, and we allow faster turns when we are at low speeds. We keep speed proportional to distance from the target, but cap it at a maximum of 1, and don't actually change it immediately, rather we set a "target speed" and move the speed closer in increments of 0.1.
  * We initialize values for the object, and then to "run" we simply subscribe to the '/scan' topic. That means velocity and direction are updated as often as the '\scan' topic is, and this updating happens mostly the callback for '\scan', 'process_scan'. This iterates through all the angles to find the closest object, and then uses proportional control to set a target velocity, and to set turn_speed (which is immediately updated at the end). We call 'update_target_speed()' after setting the target speed to bring the speed slightly closer towards the target velocity.
  * ![follower](https://user-images.githubusercontent.com/63179479/114346576-52cc0980-9b29-11eb-9bcd-fc5043e3b059.gif)

* **Wall Follower**
  * For the purposes of this explanation, let the ray parallel to the foward motion vector of the robot be at 0 degrees, and the ray perpendicular and left of the robot be at 90 degrees. This robot moves foward with constant velocity 0.3, but if it detects a wall 0 degrees ahead, it beings turning CW (we assume we are following a wall to our left). This is not enough to get the robot to fully turn, but it will get it to start early. If no wall is ahead, the robot checks the difference between measurements at 45 and 135 degrees, which should be equal if the robot is aligned paralel to the wall and turns accordingly if not. A final adjustment is made to keep the robot at the appropriate distance by checking the measurement at 90 degrees, and turning away from the wall if too close, or toward the wall if too far.
  * The structure of this object is similar to wall follower, updating every time scan data is recieved, but even simpler. It initializes values, and then begins running by subscribing to the '\scan' topic and calling back 'process_scan', which checks the 4 angles previously mentioned (0, 45, 90, 135), and makes adjustments to the 'turn_speed' accordingly by publishing to '/cmd_vel'. I started with proportional control, but added a square root because I felt that for lower values (which are less than 1), the robot wasn't adjusting quickly enough, and that a square root would help. 
  * ![wall_follower](https://user-images.githubusercontent.com/63179479/114346555-4ba4fb80-9b29-11eb-8d46-9779740cc90d.gif)

* **Challenges** \
 Programming robots to peform these behaviors feels like a sandbox task-there is no straight path nor even necessarily a best path. This makes starting the process of designing the robot a little more intimidating, but I found that it also makes the task more fun. With most other CS classes, I don't feel like there are often multiple approaches, but I often found myself urged to try new things. The Square Follower robot caused much trouble because robots do not always behave in physical environemnts as we expect them to, and this was kind of a blessing in disguise. Because of this I started to debug code (with print statements), which helped me better think about how the robot receives and adjusts to commands (which can be counter untuitive). I also tried incermentally adjusting my approach until I had something I was satisfied with.
* **Future Work** \
 I would write more general code that would be able to control and adjust the velocity of the robot. Adjusting the velocity of the robot often felt very clunky, and was "dangerous" in the sense that it could cause a lot of problems if not done well. I would like to be able to more smoothly and intuitively have the robot adjust its velocity in all tasks. I would also like to try and think of a way to get the wall follower robot to follow along smoothly when inside concave shapes.
* **Take aways**
  * **Things do not work as expected**\
    As I mentioned in my challenges, because of noise things to not work as expected; but this isn't the only reason this happens. A few times, I found myself trying to implement a solution that would maybe work in some cases, but not others, or maybe work in ways that I didn't expect them too, or simply not work at all. Sometimes this was due to bugs in my implementation, but conceptual errors happened too. Expirementation helped me overcome this, but for more complex robots I think I should probably thoroughly sketch out the behavior and design before writing code.
  * **Incremental Approaches Can wWrk**\
    Sometimes I found or implemented solutions that had some functionality, but not complete. For example, the wall follower, I initially managed to get it to follow walls, it would just not turn fast enough and end up closer and closer to the wall. This inspired my idea to adjust the robots turn according to its distance to the wall its following, which worked brilliantly. I think its alright to implement some functionalities of the robot, especially when just trying to get a grasp on the problem, and then to build on the solutions you currently have.
