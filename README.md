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

Hence, this project faces this problem through the lens of OpenCV more particularly
Computer Vision.

---

### Results

I prefer to see the results before i am shown the explanation as that gives me the
patience to keep reading further if i find the project interesting.

So here are two videos sped up by a factor of "x3".

The first video is of maze image [5](Navigation_Bot/worlds/material/texture/5.png):
![](Demo5.gif)

Next its the maze image [12](Navigation_Bot/worlds/material/texture/12.png):
![](Demo12.gif)

You can try more examples (1000 mazes to be precise generated through "" Algorithm)

All you have to do is:

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
