# botanbot_sim

Documentation is here ; https://botanbot-sim.readthedocs.io/en/latest/

Simulation of a Ackermann robot for agricultural cases. 
It is also a use-case of [vox_nav](https://github.com/jediofgever/vox_nav) 2.5D navigation framework. 
Refer to [vox_nav](https://github.com/jediofgever/vox_nav) to find out more about vox_nav.

# running botanbot
Note that the main branch is targeted to ROS2 Foxy version. You can see steps under `botanbot_sim/.github/workflows/main.yml` on how to build botanbot locally. 

After sucessul build; 

Follow the steps at [here](https://vox-nav.readthedocs.io/en/latest/running_project/index.html). 
There is a simple GUI provided to start different gazebo worlds as well as to teleoperate the botanbot.

After you select a Gazebo world, then click on Gazebo World StandAlone, to spawn the robot and start the simulation.
You can also try with [navigation2](https://github.com/ros-planning/navigation2).


# Alternatively run it in docker

Build the image with:
```console
docker build -t lcas.lincoln.ac.uk/lcas/botanbot_lcas:docker .
```
Run with:
```console
docker-compose up
```
Open a browser and type: [localhost:6080](https://localhost:6080)

Kill with:
```console
docker-compose down
```

---
See the video below to get a insight of what botanbot looks like in action.  

* [Botanbot Climbing Hill like A Champ with help of vox_nav](https://www.youtube.com/watch?v=ZQdy22LmeP0)
* [Botanbot freely Roaming out of Boredom](https://www.youtube.com/watch?v=bW7AHrf01Qg)

The botanbot is able to navigate through an uneven terrain(use `container_office_world` or `uneven_world`).
