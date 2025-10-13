## Vision-Based Autonomous Maze Solving Robot Using Finite State Control and A\* Path Planning

---

### Overview

This project aspires to solve a problem which the industry generally prefers to
tackle through YOLO(you only look once) SOTA algorithm for bounding box markup and
object labelling and detection. While YOLO is popular it has its own problems
that comes complementary to it.

- Availability of Dataset (atleast 1000 images for good accuracy)
- Training time and Energy(Depending on the FOC score expected, image quality, hardware)
- Deployability in embedded applications (there are YOLO varients for this task)
- Slower

Hence, this project approaches this problem through the lens of OpenCV more particularly
Computer Vision.

---

### Results

I prefer to see the results before the explanation as that gives me the
patience to keep reading further if i find the project interesting.

So here are two videos sped up by a factor of _8_.

The first video is of maze image [5](Navigation_Bot/worlds/material/texture/5.png):
![](Demo5.gif)

Next its the maze image [12](Navigation_Bot/worlds/material/texture/12.png):
![](Demo12.gif)

You can try more examples (1000 mazes to be precise generated through "" Algorithm)

All you have to do is:

- Install as per the installation guide.
- navigate into the launch_sim.launch.py file within the launch folder.
- change the texture_name arguement to any whole number as guided.

```py
    # Include the Gazebo launch file, provided by the gazebo_ros package
    texture_name = "12"  # <--- CHANGE ME to change maze 0 to 1000

    world_path = os.path.join(
        get_package_share_directory(package_name), "worlds", "empty.world"
    )


```

- finally execute

```bash
ros2 launch navigation_bot launch_sim.launch.py
```

to launch the project.

---

### Installation

#### Prerequisites

- Installation of [CPP_GCC](https://gcc.gnu.org/install/) and [Pyhton](https://www.python.org/downloads/) on a compatible
  os is mandatory to run this project.(GCC compiler installation for linux)
- Make sure you have ROS2 humble installed.
  If not check out this guide from the official ROS2 HUMBLE distribution documentation:
  [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- This project is entirely based on OpenCV hence it is expected to download OpenCV
  version >= 4.9.0 for a hassle free experience.
  Here is a documentation to help you get up to speed on installing OpenCV:
  [OpenCV-CPP Installation Guide](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)
  [OpenCV-python Installation Guide](https://pypi.org/project/opencv-python/)

#### Project setup and build

```bash
mkdir ~/Navigation_bot/src/
cd ~/Navigation_Bot/src/
```

- Clone this repository preferably through ssh, just the main branch.

```bash
git clone -b main git@github.com:HrishikeshMRao/Navigation_Bot.git
```

- Use sudo if you are a superuser. To install workspace dependencies through rosdep
  for a seamless example run:

```bash
sudo apt update
sudo apt install python3-rosdep
sudo rosdep init  # only once
rosdep update
```

```bash
rosdep install --from-paths src --ignore-src -r -y
```

- Build it once to use it forever. ;)

```bash
cd ..
colcon build --symlink-install
```

### Methodology

#### Introduction

Before we go step by step over my contribution, Here is a quick graph to summarise
the flow of this project.
![rqt_graph output](rosgraph.png)

The image capture calls Astar function without ros interface hence not shown here.
Similarly launch file launches Convoluter.py file with image path parameters without
any ros2 topic interface.

#### Concepts

##### Maze Image Collection

Lets start with the Question i had in mind when i started this project
i.e. _How on earth do i create these Maze pictures?_. Well there is a cool algorithm
that generates mazes out of thin air. It is called _Randomized Kruskal's algorithm_.
Though, I have not included the code in this ROS project. I will provide with a
[Kaggle Repository](https://doi.org/10.34740/kaggle/dsv/4075361) i refferd for myimages. It conviniently provided with spanning
tree aswell which will be helpful as you will see while path planning.

---

#### Convoluter.py

Though the Maze folder was imported from kaggle. It could not be used directly as a texture.
The dataset was generated on a 10x10 grid hence both black and the white pixels were of same
width. This is not ideal for line following.

Hence a function was required that would:

- Decrease the width of white edges
- During the process not make the edges blurry.

The solution is a simple idea from my Computer vision class. "It has to be max pooling".
Well thats true but we have an option to erode or dialate.

- If we dialate the blacks : corners were seen to be more prominent but blurry.
- If we erode white: corners were blunt but less diffused.

```py
# 5x5 kernel for dilation (max pooling effect)
kernel = np.ones((34, 34), np.uint8)
thinned = cv2.dilate(inv, kernel, iterations=1)
```

We can strike a balance between the two by first resizing the image by "2X". And then
dialate the blacks with a larger kernel size (37,37). This gave the optimal result as
shown in the demo.

```py
scale = 2
new_size = (int(img.shape[1] * scale), int(img.shape[0] * scale))
img = cv2.resize(img, new_size, interpolation=cv2.INTER_CUBIC)
```

This function is launched passing 2 arguements:

- Absolute path to the image source folder: textures (original kaggle folder)
- Absolute path to the final image texture folder refferenced by sdf file.

---

##### Gazebo world building

Now that the images are ready we can start with developing a gazebo environment for
simulating the phisics involved in robot locomotion. Thankfully in this project all
i had to do was change the texture of a 10x10 default ground plane from material/grey
to the .png file of our choice presented in the launch file.

The easiest way to do it is through a material script. It is rewritten everytime
we launch it.

```xml
material Maze/diffuse
{
    receive_shadows off
    technique
    {
        pass
        {
           lighting off            // disables light shading
           depth_write off         // prevents z-fighting with plane
           ambient 1 1 1           // full brightness
           diffuse 1 1 1           // no color tint
           emissive 1 1 1          // self-lit (glows)
           specular 0 0 0 0        // no shininess

            texture_unit
            {
                texture 12.png
                filtering anisotropic
                max_anisotropy 16
            }
        }
    }
}
```

The parameters passed in the material script are provided in [gazebo texture tutorial](https://classic.gazebosim.org/tutorials?tut=color_model#OgreMaterialScripts)
ensure the texture is as sharp as possible before we treat it for corner detection
"Cant definitively say these are effective".

- filtering anisotropic: 16 max value possible. reduces blur when viewed at sharp
  angles.

The sdf file required by gazebo to load its environment is empty.world.
This here uses the default sun model found in .gazebo/models and a custom 10x10 ground
plane which has been passed the folder texture that has the images, and the relative path
of the folder with script material and the name of the material Maze/Diffuse in our case.
Finally we, launch this world along with gazebo gui through the launch file instead of the
terminal.

```xml
<material>
  <script>
    <uri>materials/scripts</uri>
    <uri>materials/texture</uri>
    <name>Maze/diffuse</name>
  </script>
</material>
```

---

##### XACRO for Navbot

We can focus on creating a bot which exhibits balance(Does not topple and has axle
with no z offset). Also, a known axis of rotation aligned with the camera position.
will ensure that the image captured by the camera does not sway when the robot is
in pure rotation. It is necessary as this project avoids use of AI. To compensate for
these irregularities.

Therefore it was decided to go with:

- A simple box design with 2 differential drive wheels\(continuous joints\)
- Two black wheels to ensure same frictional force on either side.
  Again to avoid sway. (Fixed joints)
- A camera mount pointing towards the ground base 0 x and y offset from baselink
  a z offset of robot body - some offset to not self collide.
- An imu aligned to the axis of rotation. To ensure perfect yaw reading.
- A gazebo GPS plugin (To point coordinates on the screen to indicate motion)

A ros2_control plugin to simulate differential drive on the two continuous joints
seperated by an axle of length 0.47. stored in a .yaml file.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true

    diff_cont:
      type: diff_drive_controller/DiffDriveController

    joint_broad:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_cont:
  ros__parameters:
    publish_rate: 50.0

    base_frame_id: base_link

    left_wheel_names: ["Back_right_wheel_joint"]
    right_wheel_names: ["Back_left_wheel_joint"]
    wheel_separation: 0.47
    wheel_radius: 0.07

    use_stamped_vel: false
```

- diff_cont: subscribes to diff_cont/cmd_vel and depending on linear x and angular z
  published it decides the angular velocity to each node. Depends on the
  parameters passed as shown above.
- joint_broad: a publisher on topic /joint_state for joint angles required by
  rviz to visualise the robot transforms.

---

#### ImageCaptureNode

A custom node launched soon after a spawner for the robot and the ros2_controllers
are run.

@returns: None
@Parameters: Map
@Type std::string
@Description: ROS PARAM passed through launch file or command line.

Subscribes to:

- "/camera/image_raw"
- "/diff_cont/odom"
- "/imu_plugin/out"

Publishes to:

- "diff_cont/cmd_vel_unstamped"
- "camera/image"

#### OpenCV Implementation

Ros2 documentation for OpenCV provides a bridge for interpreting /camera/image as
OpenCV image matrix.

This is done easily in 1 line:

```cpp
// Convert ROS image message to OpenCV image
cv_bridge::CvImagePtr cv_ptr =
    cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
image = cv_ptr->image;
```

A callback is generated for each frame received through /camera/image_raw. for efficient
processing.

#### Corner detection Algorithm

Basically this idea stems from a [post on stack overflow](https://stackoverflow.com/questions/59383119/how-to-approximate-jagged-edges-as-lines-using-python-opencv)
The key idea is to :

- Convert the image from BGR to GREY
- Binary threshold anything not 0(1,255) as white.
- Morph the edges twice. Once using parameter OPEN and then with CLOSE.
  This ensures that there are not white or black hair strand like pixels
  stemming out in solitude.
- find the contours of this morphed binary thresholded image.
- Approximate a polygon on the contour. And overlay it as a mask to create a new
  image. in this case (polygon 0.5% perimeter of the contour)
- Run Shi-Tomasi Corner Detector i.e. goodFeaturesToTrack() function.

Following is a code snippet documenting the fine tuned parameter values of
goodFeaturesToTrack for the project's use case:

```cpp
cv::goodFeaturesToTrack(
    mask, corners,
    4,   // maxCorners – increase for more detections
    0.4, // qualityLevel – smaller detects weaker corners
    170  // minDistance – minimum pixel spacing between corners
);
```

This function is preffered over harris corner detection for its speed. and the
ability to pin point the pixel coordinates of these corers. It is also possible
to adjust the sensitivity distance and neighbours making it highly customisable.

---

#### Decision Making

Finite state machines are employed for the purpose of carrying out tasks sequentially.
This allows for the program to run without loops or stalling the callback.

Two Enums are in play.

---

Enum: _State_
Has Commands representing the current state of the bot:

- SEARCH
- ALIGN_CENTRE
- TURN

  Enum: _Shape_
  Depicts the action the bot has to take at a junction/corner.

  - LEFT
  - RIGHT
  - STRAIGHT

- MOVE_FORWARD

---

The flow chart attached summarises the entire code structure:
![](Navigation_bot_Flow.png)
Tool Courtesy: draw.io

---

#### Shape detection

The flow chart provides a quick glimpse of how the shapes were identified.

##### LEFT and RIGHT

Given, the first 2 points identified are corners[0] and corners[1].

Within the ALIGN_CENTER state, after checking if the centres have aligned within
the buffer of 0.001. We check two conditions i.e.:

```cpp
if ((std::abs(midX - corners[0].x) > 5) &&
              (std::abs(midY - corners[0].y) > 5))
```

These two conditions imply a T junction if true.(A Cross is identified as a
T junction as soon as its first two points are observed)

Below Image illustrates the idea of a T junction.
![](Junction.jpeg)

Keeping in mind the x and y pixel coordinates are maximum towards the left corner
of the image. Here is an illustarion for the reason of comparing topx with midx
for decision making on the turns.
![](Turn.jpeg)

---

#### PD Controller

Lets address the elephant in the room. Why i havent chosen PID controller nearly a
gold standard for feedback control in robotics industry for this project.

- I did not face overshoot issue too often.
- I did not want to risk oscillation in my system
- Very hectic to tune Ki value compared to others.

Here is a structure of my PID controller:

```cpp
struct PIDController {
 public:
  PIDController(double kp, double kd)
      : kp_(kp), kd_(kd), previous_error_(0.0) {}

  double compute(double setpoint, double pv) {
    double error = setpoint - pv;
    double derivative = error - previous_error_;
    previous_error_ = error;
    return kp_ * error + kd_ * derivative;
  }

 private:
  double kp_;
  double kd_;
  double previous_error_;
};
```

Kp -> 0.0007
Kd -> 0.0001
was found to be just right. A balance between speed and task completion without
oscillation.

Due to floating point errors it was sought to be a good practice to provide a buffer
instead of hard equality checks in if conditions as below.

```cpp
if (std::abs(error) < 0.001)
```

---

#### IMU Normalisation and quaternion to euler conversion

This was fairly new for me to convert one form of angle representation to another.
It was much easier to tune the angular.z pd value through yaw angles rather than
position of corners.

An initial idea was planned to estimate an angle of rotation based on the diagonal
corners on a turn. By measuring their slope we can tune the pd controller to
align the corner points to their mirror about the x axis.

This worked well for the corners. But there was no assymetry on the T and cross junctions
to identify left or right turn. Hence an IMU was forced to provide accurate yaw
measurements that can directly be utilised.

Normalisation of the angles was another important step.

- The bot used to keep spinning in certain turns/junctions.
- This was a consequence of the line: (present_yaw+-90)

On the first glance the idea seemd fine but consider present_yaw to be
90.02 and by adding 90.0 the desired angle is past 180. This is a problem as the
quaternion to euler wraps the angles after 180 to -180. So we mimic the same with
present_yaw:

```cpp
auto normalize_angle = [](double angle) {
  while (angle > 180.0)
    angle -= 360.0;
  while (angle < -180.0)
    angle += 360.0;
  return angle;
};
```

---

#### Astar.cpp

The spanning tree generated by the dataset imported from kaggle was the backbone for path
planning around the maze.

- It came across to me to make the bot backtrack and correct itself.
- Blind maze where we can potentially make it a heuristic search
  by approaching those turns that lead you towards the goal rather than away.

Disadvantages of the above approach:

- The solution is time consuming
- No way to detect if the dead end is a T junction or not

So another potential candidate was RRT(Rapidly Exploring RandomTrees) path solver.

- Sample random points along the maze image
- check if the point is on a black pixel or a white pixel
- if white then add it to the nearest node if there exists a euclidean path
  without any black pixels in between.
- if not sample another point. Try sampling 10% of the time near the goal.

Disadvantages of this method:

- Requires 10000s of points to be sampled due to white area being smaller
  hence creating thousands of nodes.
- Requires GPS coordinates to determine the position of the bot wrt closest node to decide
  the route henceforth from that junction.
- why reinvent the wheel when the edge map is available.

Hence an Astar algorithm was employed. Its a replica of my submission on coursera modern robotics
ported to cpp.

Made sure not to forget including the start and goal positions after the path was returned.

```cpp
reverse(path.begin(), path.end());
path.insert(path.begin(), -1);
path.push_back(100);
```

The direction determination was basic mathematics using vectors.

- Iterate from the 0th node to N-1 node
- draw a vector from N-1 to N
- draw another vector from N-1 to N+1
- obtained the signed angle between the two vectors
- if the angle is +ve then it is a left turn
- if it is -ve a right turn
- if it is 0 then straight.

```cpp
cout << "Direction to move at junctions\n";
vector<int> direction;
for (int node = 1; node < path.size() - 1; node++) {
  if (adj[path[node]].size() > 2) {
    int ax = path[node - 1] % 10, ay = path[node - 1] / 10;
    int bx = path[node] % 10, by = path[node] / 10;
    int cx = path[node + 1] % 10, cy = path[node + 1] / 10;
    pair<int, int> vectorbase = {bx - ax, by - ay};
    pair<int, int> vectornext = {cx - ax, cy - ay};
    double signed_angle = atan2(vectornext.second, vectornext.first) -
                          atan2(vectorbase.second, vectorbase.first);
    if (signed_angle > 0)
      direction.push_back(1);
    else if (signed_angle < 0)
      direction.push_back(-1);
    else
      direction.push_back(0);
  }
}
return direction;
```

It would be more intuitive is vectors are inspired from the direction the bot is moving
return the direction vector when called by the ImageCaptureNode in its costructor.

---
