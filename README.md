# ClothoidPlanner
Clothoid Planner in Unity (C#)

Clothoid planner generates multiple clothoid paths from multiple orientations (yaws/headings) at start points, to multiple orientations (yaws/headings) at goal point.
It can be used for creating smooth trajectories during local planning.

The user has access to set the orientation intervals, number of trajectories, orientation steps, as well as selecting the start and goal positions.

https://user-images.githubusercontent.com/19212519/206912310-5a79e4ed-bcb1-41b4-9830-7d1733a0a84f.mov

![image](https://user-images.githubusercontent.com/19212519/206912459-f91c99a8-3248-4482-a302-6b77389de4d8.png)

Based on: The python implementation: https://github.com/AtsushiSakai/PythonRobotics 
Paper: Fast and accurate G1 fitting of clothoid curves
https://www.researchgate.net/publication/237062806
