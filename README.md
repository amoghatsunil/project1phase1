Clone into your workspace's src folder. Then, in that folder, run:
```bash
mv ENPM662_Project1 project1phase1 && cd ../.. && colcon build --packages-select project1phase1
```
Then, to run the code, use the following command:
```bash
ros2 launch project1phase1 gazebo.launch.py
```
