
## Udacity Self Driving Car Nanodegree - Term 3 - Path Planning Project


### Purpose of the project:
     For this project we were required to design a path planner which is able to create smooth, safe paths for a car to follow along a 3 lane highway with traffic.
     
### Success Conditions
1. Keep inside its lane.
2. Avoid hitting other cars. 
3. Pass slower moving traffic all by using localization, sensor fusion, and map data.
4. Code must compile with cmake and make

#### Instructions to compile and run the code
1. Navigate to the 'build' directory of the solution
2. From a terminal execute cmake .. && make
3. In the terminal execute ./path_planning

## Rubric Points
1. Car is able to drive 4.32 miles without incident.
<BR CLEAR="left"/>
<img src="./ReadMeImages/Image 001.png" height="200" width="400"  align="Left"/>
<BR CLEAR="left"/>
2. The car drives according to the speedlimit.
    * This was accomplished by added a max speed to the code (see #3).
3. Max Acceleration and jerk are not exceeded.
    * Max acceleration was handled by incrementing speed by a small amount, in this case .224 mph.  This allowed for a smooth acceleration but did cause issues later (discussed in the areas for improvement).
    ```C++
    else if(ref_vel < 49.5)
    {
      ref_vel += .224;
    }
   ```
4. Car does not have collisions.
    * collisions with vehicles in front of the car are handled by 2 'if' statements in the code.  The first determines if the car is within the same lane as us. it does this by looking at the car's 'd' position and determines if it is within +- 2 meters of ours.  This +-2 allows the vehicle to detect a vehicle which is offcenter, but still in the lane. The second determines if the vehicle's 's' position is greater than ours (in front of the car) and is less than 30 meters away and triggers the 'slow_down' and the 'change_lanes' variables. For more information, see the 'Changing Lanes' topic and explanation below.  
    ```C++
      //this will tell us if a car is in our lane. the +-2 allows us to see a car that is off center in a lane
      if(d < (2+4*lane+2) && (d>(2+4*lane-2)))
      {
        //check s values greater than mine and the s gap
        //if our car is within 30 meters, then slow down
        if((check_car_s > car_s) && ((check_car_s-car_s) < 30))
        {
          //lower the reference v so we dont hit the car ahead. Could also change lanes.
          too_close = true;
          change_lanes = true;
        }
      }
    ```
5. The car stays in it's lane, except for the time between changing lanes.
    * The only time a change lanes action is executed is when there is a vehicle in the way. 
6. The car is able to change lanes.
    * The car changes lanes when it detects a vehicle in front of it, or it is not in the center lane. The lane change is determined by using the spline file to fit a polynomial for the waypoints.
    


### Changing Lanes

Changing lanes was the part of the process which took the most thought and trial and error.  Before I started creating the logic i first had to decide how I even wanted to approach lane changes.  The first item I knew I wanted was to always favor lane changes to the left.  In America, you should always pass on the left, so naturally I wanted to make sure my car followed ALL laws.  I also knew I wanted to trigger something like a lane change event, a signal for later in the code to change lanes.  I also determined I wanted to block lanes off if there was a car in close proximity to mine.
    
The determination to block lanes off, was one of the more significant choices I had to make.  I knew I was going to set a bool if it was time to change lanes.  I also wanted to set bools to tell me if the left lane and right lane were clear of obstacles, but this involved a lot of diagramming and thought to find the best way.  Since I was using bools such as 'Left_Clear' there are 2 lines of thought when it comes to setting bools.  The 'safe' line of thought, which I have been taught by many people is, always set the bool to false, and then set it to true if needed.  This keeps you safe by ensuring the bool is always false, unless a specific set of circumstances was met.  But as I thought of the iterative nature of this programming (looping through sensor data), I thought this way would actually be less safe.  I decided to set the bools for left and right to true on reciept of a JSON message, and then set to false if a vehicle was detected. This way, whenever an object was detected in the lane which was too close, it closed the lane off for the entire iteration and there was no way it could be cleared out if a later iteration detected a vehicle which was farther away.  If I had went the opposite way, I would have had to set the bool to true at the beginning of each iteration, and then set it to false if a vehicle was detected. This seemed to be an excessive amount of switching, and there was a chance a vehicle later in the iteration would improperly open a lane.
    
When a vehicle is detected which is too close, it switched the change_lanes bool to true.  After the iterations of the JSON message, if change lanes was true, the vehicle would need to determine which lane to go to.  As I said earlier, I wanted to favor lane changes to the left, so i decided to check if left was clear first, and if not, then make a change to the right.

```C++
    //we want to favor lane changes to the left as you should always pass on the left if open.
    if(change_lanes)
    {
      // std::cout<<"Change Lanes Triggered"<<std::endl;
      if(left_clear && lane != 0)
      {
        lane = lane - 1;
        // std::cout<<"Left Clear"<<std::endl;
      }
      else if(right_clear && lane != 2)
      {
        lane = lane + 1;
        // std::cout<<"Right Clear"<<std::endl;
      }
    }  
```
The above code is simple, but accomplishes several things:
* It only executes a lane change if the change_lanes bool is true.
* it favors lane changes to the left.
* It will only execute the lane change if it does not send you off the road (checking to see if you are in lane 0 before changing lanes left).

I also decided I would want to stay in the center lane as much as possible.  No one likes someone cruising in the left lane! so I added the below snippet which would bring the vehicle back to center if it was safe to do so. Because this is an 'else' statement, it will only execute this if we are not already changing lanes.
```C++
     else
    {
      if(lane < 1 && right_clear)
      {
        lane = lane + 1;
      }
      else if(lane > 1 && left_clear)
      {
        lane = lane -1;
      }
    }

```

One of the hardest parts of this was determining if the lane to the left or right was clear.  In order to detect a vehicle which was blocking, I had to modify the code I used to determine if a vehicle was close by looking a bit behind the car as well.  
What I came up with is:
```C++
      //Check for 30m in front and 15 m behind car
      if((check_car_s > (car_s - 15)) && ((check_car_s-car_s) < 30))
      {
        //If the car is close enough to us, then determine if it is blocking us.
        //this will mark what lane the detected vehicle is in 
        int object_lane = 0;
        //Determine the lane the object is in
        if(4 < d && d < 8)
        {
          object_lane = 1;
        }
        else if(8.1 < d && d < 12)
        {
          object_lane = 2;
        }

        //now we need to determine if the car is blocking the left or right, 
        //while also excluding lanes more than 1 lane out.
        if(object_lane > lane && (object_lane < lane + 2))
        {   
          // std::cout<<"Blocking the right"<<std::endl;               
          right_clear = false;
        }
        else if (object_lane < lane && (object_lane > lane - 2))
        {
          // std::cout<<"Blocking the left"<<std::endl;
          left_clear = false;
        }                
      }
```
I decided on this because firstly, it only does this work if a car is in close proximity.  I figured there was no need to execute this code if the car was too far away to be a concern. Once it determines a vehicle is close, then it determines if the vehicle is to the the lane the vehicle is in.  Then I looked at if the lane was less than(to the left) or greater than (to the right) of the lane our car is in.  This worked good in theory, until I came to the point of 'If I am in the left lane, then ANY car on the right would block the lane change.  If a vehicle was 2 lanes over, we shouldn't block the change off'.  I was able to work around this by adding the second condition in the 'if' statement which ignored cars not in the lane next to us.

### Areas of improvement.

There are a few areas I would like to improve if I had more time to complete the project.  Firstly, the model suffers from the accordian effect.  This is due to the accelerate and decelerate logic:
```C++
if(too_close)
{
  ref_vel -=.224;
}
else if(ref_vel < 49.5)
{
  ref_vel += .224;
}
```

Basically this is just a slow down if too close, and speed up if nothing is in front.  This is happening because I am just looking at a horizon of 30 meters.  Ideally I would add some logic, where if we are approaching a car we slow down in direct relation to the distance between the vehicle and me.  As we are farther away, we slow less, as we get closer, we slow faster.  This would allow me to develop some logic that as we slow down, and the gap steadies around a specific distance, say 25 meters, then we would start to accelerate or decelerate in tiny increments to 'keep pace' with the car.  

The other area I would like to improve would be lane selection.  In the image below, the vehicle is changing lanes to the left, as it should, but in reality, the right lane would be the better choice as it has a much clearer path  
<BR CLEAR="left"/>
<img src="./ReadMeImages/Image 002.png" height="200" width="400"  align="Left"/>
<BR CLEAR="left"/>
To do this, i would look a bit further ahead in each lane and use a 'block_potential' bool, or maybe an int to represent distance to next block.  This would allow the vehicle to look at the best lane in total and make a better decision.

Finally, a lot of values are hardcoded, such as lanes and speed.  I would prefer to have speed be a variable which is passed in to the method, as this could be coming from cruise control or perhaps the speed limit from GPS data.  I would also like to tweak the lane detection guidance so it doesnt rely on assigning a lane.  I would want to be able to just determine if the lane to the left or right was clear so this would work no matter how many lanes are on the road.

