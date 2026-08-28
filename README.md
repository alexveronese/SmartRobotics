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
- Ground plane: `2 x 2 m`, with a muted-green surface.
- Tabletop: `0.65 x 0.68 x 0.05 m`, with its top at `Z = 0.000 m`.
- Table X range: `0.245–0.895 m`; this leaves 85 mm between the tabletop and the pedestal.
- Board: `0.32 x 0.32 x 0.016 m`, top at `Z = 0.016 m`.
- O pieces: solid blue cylinders, `68 mm` diameter and `30 mm` height.
- O centre: `Z = 0.015 m` on the table and `Z = 0.031 m` on the board.
- Panda TCP grasp target: the O middle height, `Z = 0.015 m` at the supply and `Z = 0.031 m` at the board.
- Safe approach height: `Z = 0.240 m`.

The O model is attached to the Panda with Gazebo's native detachable fixed joint
while the gripper is closed and is released directly at the piece-centre height.
Motion speed is controlled by
`motion_duration` and `gripper_duration` in `config/game.yaml`; smaller values
are faster. The supplied `1.25 s` / `0.45 s` settings are a moderate Gazebo
speed-up. Keep `motion_duration` at or above roughly `0.8 s` unless controller
tracking has been verified on the local machine.

The Panda opens to an `80 mm` aperture and closes to a commanded `66 mm`
aperture to grip the O around its external diameter. X and O use matching
five-piece rows, mirrored at `Y = +/-0.26 m`, with `100 mm` between adjacent
centres so the gripper has room to approach each piece. The robot consumes its
row from the far end (`O5` through `O1`). O models are
dynamic bodies transported by the native fixed joint, avoiding delayed pose
updates and the previous magnetic-following appearance.

Gazebo's DART backend may print `NameManager::issueNewName` messages for
`fixed(1)` through `fixed(4)` when the Panda model is created. They are
informational auto-renames: Gazebo 8.11's stock detachable-joint plugin
hardcodes its internal joint name to `fixed` and exposes no custom-name option.
The joints remain functional. Piece links and collisions use unique names, so
the former `link:c` duplicate messages no longer occur.

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
