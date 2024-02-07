# Crowd Simulation 23/24 course project
#### Developed by [Gianmarco Picarella](https://github.com/gianmarcopicarella), [Joel Brieger](https://github.com/Joel-BB) and [Nikos Giakoumoglou](https://github.com/NGiakou)


<img src="https://github.com/gianmarcopicarella/crowd-simulation/blob/main/Assignment%20Deliverables/repository_wallpaper.png?raw=true" width="100%">

## About
In the project "Zombie Horde Shooter", we simulated a zombie horde with up to 20+ thousand autonomous agents trying to conquer the main city square. The player has the goal is to protect the square from the zombies by means of an automatic rocket launcher. Zombies are spawned within a torus-shaped area centered around the square center and move towards it with a constant speed using a navigation mesh and avoidance system to avoid environmental obstacles and the other zombies. The player can jump from one building roof to another in order to shoot the zombies. If the number of zombies within the square is above a specific threshold then the game is lost. If the number of zombies is zero and the previous condition has not yet occurred then the game is won. The final result was a short and interactive game created with Unreal Engine 5 achieving satisfactory results in the amount of agents simulated while also preserving a playable frame-rate. A more detailed description of the project can be found [here](https://github.com/gianmarcopicarella/crowd-simulation/blob/main/Assignment%20Deliverables/Crowd_Simulation_Game_Report.pdf).

## Tech Stack
Unreal Engine 5.3, UMass plugin, Visual Studio, C++
