[English](README.md) | [日本語](README_jp.md) 

# Hakoniwa ROS Simulator

The [TOPPERS Project Hokoniwa WG](https://toppers.github.io/hakoniwa) develops "Hakoniwa," a virtual simulation environment for the age of IoT/Cloud Robotics.

This repository provides an environment where you can quickly try simulations of ROS 2 programs on Hakoniwa.

## Requirements

* Windows: Windows 10/11
  * Ubuntu 20.04 LTS on WSL2/WSLg
* Linux: Ubuntu 20.04 LTS
* MacOS: macOS Catalina ver.10.15.7

In the Windows environment, we assume that you will perform all operations on the WSL2/Linux shell.
Please execute under the Windows file system (under `/mnt/c/`), not under the WSL2 file system (under `/home/${USER}/`).

## Preparations

### Clone this repository

The latest version is **v1.1.4**.
See also "[version information/update history](/appendix/version.md)" (the version number corresponds to [tag/release in Git/GitHub](https://github.com/toppers/hakoniwa-ros2sim/releases) and [Docker Hub tag number](https://hub.docker.com/r/toppersjp/hakoniwa-ros2sim/tags).

Clone this repository with the following command in the terminal.

```
git clone --recursive -b v1.1.4 https://github.com/toppers/hakoniwa-ros2sim.git
```

### Install Docker Engine

This simulator uses Docker.

Note: We experimentally try to run the simulator in native Linux environments (including WSL2).
Please refer to [appendix/native.md](/appendix/native.md) for the procedure to run in the native environment.

#### For Mac

We recommend using [Docker Desktop for Mac](https://docs.docker.com/desktop/mac/install/).

#### For Windows/WSL2 and Linux

The simulator requires the Docker Engine to be installed on WSL2.
If the following command outputs the same result in the terminal of WSL2, it has already been installed (the line starting with `$` indicates the command to be executed).

```
$ which docker
/usr/bin/docker
$ service --status-all |& grep docker
 [ + ]  docker   # or " [ - ]  docker "
$ service docker status
 * Docker is running   # or " * Docker is not running "
```

Since the Docker Engine installation procedure is a bit complicated, we summarized it in [`docker/install-docker.bash` in "toppersjp/hakoniwa-single_robot" repository](https://github.com/toppers/hakoniwa-single_robot/blob/main/docker/install-docker.bash) by referring the official manual. If you have any trouble to execute this script, please follow the official manual to install Docker.  
[Install Docker Engine on Ubuntu | Docker Documentation](https://docs.docker.com/engine/install/ubuntu/)

Execute as follows to install Docker with our script.

```
wget https://raw.githubusercontent.com/toppers/hakoniwa-single_robot/main/docker/install-docker.bash
bash install-docker.bash
```

If the `service docker status` is ` * Docker is not running `, start Docker.


```
sudo service docker start
```

If the output is as follows, Docker is running.

```
 * Starting Docker: docker                           [ OK ] 
```

This procedure requires the user to be a member of the `docker` group. Otherwise, execute the following command.

```
sudo gpasswd -a $USER docker
sudo chgrp docker /var/run/docker.sock
sudo service docker restart
```

The results of the above command execution will take effect after you re-login to the terminal.


### Install Unity 

* Unity Hub 3.1.1 or later
* Unity Editor 2021.3.0f1
  * If you do not see this version on the "Installs > Install Editor" screen of Unity Hub, you can install it by clicking on "Unity Hub" in the [Unity Dowonload Archive](https://unity3d.com/get-unity/download/archive).  

## Simulator Installation Procedure

### Deploy Docker images

The simulator execution environment is available as a pre-built Docker image on the Docker Hub.

https://hub.docker.com/r/toppersjp/hakoniwa-ros2sim

Execute the following command to launch Docker, pull and deploy the image.

```
bash docker/pull-image.bash
```

\[Tips for Developer\] There is a `docker/create-image.bash` for creating Docker images.

### run Docker

We recommend using two terminals to run this simulator.
Please change to the directory of this repository (`hakoniwa-ros2sim/`) on all terminals.

Start the docker container with the following command in Terminal A.

```
bash docker/run.bash
```

In a Mac environment, you need to specify the network port name (e.g., "en0") as an argument.
You can check the port name with the `ifconfig` command.

```
bash docker/run.bash <port>
```

### Install the Hakoniwa ROS environment on the launched docker container

```
bash hako-install.bash opt all
```

### Open Unity project

Start Unity Hub, click "Open" in the upper right corner, and specify the following directory.

Path：ros2/unity/tb3

![](https://camo.qiitausercontent.com/34b3dee89c42787380e888af0bb4a321c866ebe3/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f39353336626137632d383932372d383037302d613934372d3361643839396436363731622e706e67)

The following screen will be displayed when started.

![](https://camo.qiitausercontent.com/118f0af1ef4fdf3cdb4c2e7017d7ac92e2079943/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f33303965666239392d643663612d383937612d356233342d3334386233333763353339322e706e67)


And then, select "Assets/Scenes" in the "Project View" at the bottom left of the screen, and double-click the scene "Toppers_Course" at the bottom of the screen.

![](https://camo.qiitausercontent.com/34335041b2e8814a02a877d7d9de420ed0368d29/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f30303730336135642d373031342d656331322d396334612d3161616230366230323864612e706e67)


Then you will see the course as follows.

![](https://camo.qiitausercontent.com/5c2d040c568dadccec8e3347349d949716f5ce11/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f64666365393531362d663036372d303364652d366131622d3432376462653263396232332e706e67)


## Simulator Running Procedure

I will illustrate the procedure with the following example.

* ROS control program: `src/tb3/src/tb3ctrl.cpp`
* Unity application: `TB3RoboModel`

Execute docker containers in both terminals A and B.

If you have terminated the Docker container on terminal A, please start it again.

```
bash docker/run.bash
```

And, on terminal B,  you exec the following command.

```
bash docker/attach.bash
```

### Operations on Terminal A

Start ROS-TCP-ENDPOINT with the following command.

```
# bash launch.bash
```

### Operations on Terminal B

Start the ROS2 program with the following command.

```
# bash run.bash tb3 TB3RoboModel
```

### Start Unity simulation.
Click the Start Simulation button in Unity and see the following startup screen.

![](https://camo.qiitausercontent.com/8aa80400f8a6b9527febde6edc5778187dc5f1cd/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f64396265303530612d393662352d353032322d653230642d3964616332633166613430322e706e67)

Clicking the "Start" button on the Unity starts the Hakoniwa simulation, and TurtleBot3 begins to move.

### Running

![動作例](https://camo.qiitausercontent.com/6aae22e5ac3d57f9faaf43c75d9eee84eb0d0dc8/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f32316433333837622d336663652d303430322d623734362d3666353333353763643239612e676966)

Each window in this video corresponds to the following.

- Upper right: Terminal A
- Lower right: Terminal B
- Left: Unity

## Contributing

Please let us know your comments and suggestions for improvement of the "Hakoniwa ROS Simulator" in the [GitHub Discussions](https://github.com/toppers/hakoniwa/discussions/categories/idea-request). We also welcome [Pull Requests](https://github.com/toppers/hakoniwa-ros2sim/pulls) for modification proposals.

## TODO

- [ ] Provide examples of how SLAM and Nav2 work ([#19](https://github.com/toppers/hakoniwa-ros2sim/issues/19))


## Acknowledgments
* We thank Associate Professor Akio YOSHIOKA and undergraduate students Ryoji SUGISAKI and Akemi KIMURA of Takarazuka University school of media art in Tokyo for their cooperation in designing the Unity package for TurtleBot3
* TurtleBot3 Unity assets are based on data provided by Robotiz, Inc. We deeply appreciate their cooperation.

## Licenses

It is released under [TOPPERS license](https://www.toppers.jp/license.html).  
The copyright holder is TOPPERS Project Hakoniwa Working Group. 
Please refer to [LICENSE.md](./LICENSE.md) for details.
