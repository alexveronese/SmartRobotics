# Camera-guided tic-tac-toe robot

This ROS 2 Jazzy demo uses a seven-axis Franka Emika Panda to play tic-tac-toe on a Gazebo table. The human is red **X**, the robot is blue **O**, and cells are numbered from 1 to 9 in reading order as seen by the overhead camera.

The camera is the source of truth for the board. Each requested X move is applied to the simulated piece, then OpenCV must observe and confirm it before the minimax player selects the robot response. The robot performs an approach, grasp, transfer, release, retreat sequence through `ros2_control` and returns to `HOME` after every O turn. Minimax makes the robot unbeatable.

## Native Ubuntu 24.04 / ROS 2 Jazzy setup

Docker is not required. Install ROS 2 Jazzy Desktop first, then install this package's dependencies with `rosdep`:

The robot meshes and inertial model come from the Jazzy Panda description package. `rosdep` installs it automatically, or it can be installed explicitly with:

```bash
sudo apt install ros-jazzy-moveit-resources-panda-description
```

```bash
source /opt/ros/jazzy/setup.bash
cd ~/jazzy_ws/src
git clone <repository-url> SmartRobotics
cd ~/jazzy_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

If this repository is already your current directory, it can also be built in place:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --base-paths .
source install/setup.bash
```

## Run a match

Start simulation and controllers in terminal 1:

```bash
source ~/jazzy_ws/install/setup.bash
ros2 launch nuovo_progetto gazebo.launch.py
```

Once all three controllers report `active`, start the interactive node in terminal 2:

```bash
source ~/jazzy_ws/install/setup.bash
ros2 run nuovo_progetto tic_tac_toe_game.py --ros-args \
  --params-file $(ros2 pkg prefix nuovo_progetto)/share/nuovo_progetto/config/game.yaml
```

Enter an available number when prompted. The interactive node resets all pieces to their supply locations when it starts, so restart it to begin a clean match. Stop both terminals with `Ctrl-C`.

Useful inspection commands:

```bash
ros2 topic echo /tic_tac_toe/board_state
ros2 topic echo /tic_tac_toe/status
ros2 run rqt_image_view rqt_image_view /tic_tac_toe/annotated
ros2 control list_controllers
```

`/tic_tac_toe/board_state` is a nine-character camera result (`x`, `o`, or `-`). The annotated image shows the detected board, cell ROIs, numbers, and classifications.

## Scene dimensions

- Panda base flange: `Z = 0.000 m`, fixed to Gazebo world on a dedicated 160 mm-radius pedestal.
- Tabletop: `0.65 x 0.68 x 0.05 m`, with its top at `Z = 0.000 m`.
- Table X range: `0.205–0.855 m`; this leaves 45 mm between the tabletop and the pedestal.
- Board: `0.32 x 0.32 x 0.016 m`, top at `Z = 0.016 m`.
- Pieces: `68 mm` outside diameter, `36 mm` hole, and `14 mm` height.
- Piece centre: `Z = 0.007 m` on the table and `Z = 0.023 m` on the board.
- Panda TCP grasp target: `Z = 0.016 m` at the supply and `Z = 0.032 m` at the board.
- Safe approach height: `Z = 0.240 m`.

The O model follows the Panda TCP continuously while the gripper is closed and
is released directly at the piece-centre height. Motion speed is controlled by
`motion_duration` and `gripper_duration` in `config/game.yaml`; smaller values
are faster. The supplied `1.25 s` / `0.45 s` settings are a moderate Gazebo
speed-up. Keep `motion_duration` at or above roughly `0.8 s` unless controller
tracking has been verified on the local machine.

The Panda opens to an `80 mm` aperture and closes to a commanded `66 mm`
aperture to grip the O around its external diameter. Supply rows are separated
by `80 mm`, so adjacent pieces no longer overlap before pickup. O models are
static pose-driven bodies because their attachment is controlled by the TCP
pose service; this prevents simulated velocity from tipping a released ring.

These heights match the base coordinate convention used by the Panda description. They intentionally replace the old custom-arm scene, whose table top was at `Z = 0.20 m`.

## Tests

```bash
colcon test --packages-select nuovo_progetto
colcon test-result --verbose
```

The tests cover wins/draws, optimal replies, exhaustive proof that X cannot beat the robot, IK reachability for every supply/cell pose, and synthetic camera classification.

## Main files

- `worlds/tic_tac_toe.sdf`: table, 3x3 board, overhead camera, and ten pieces.
- `urdf/panda.urdf.xacro`: Franka Panda description extension and Jazzy `gz_ros2_control` interfaces.
- `scripts/tic_tac_toe_game.py`: terminal game loop and manipulation state machine.
- `tic_tac_toe/vision.py`: board localisation and colour classification.
- `tic_tac_toe/game.py`: rules and minimax AI.
- `tic_tac_toe/kinematics.py`: seven-axis URDF kinematics and damped-least-squares IK.
