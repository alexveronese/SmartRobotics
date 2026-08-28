# Camera-guided tic-tac-toe robot

This ROS 2 Jazzy demo uses a seven-axis Franka Emika Panda to play a best-of-three tic-tac-toe match on a Gazebo table. The human is red **X**, the robot is blue **O**, and cells are numbered from 1 to 9 in reading order as seen by the overhead camera.

The camera is the source of truth for the board. For every terminal move, the
robot picks an X from the human supply and places it in the requested cell;
OpenCV confirms it before the minimax player selects and places O. The robot
returns to `HOME` after every manipulation. After every round, including the
last one, it physically returns every played piece to its original supply slot.
Minimax makes the robot unbeatable.

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

## Run a best-of-three match

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

Enter an available number when prompted. X opens rounds 1 and 3; the robot opens
round 2. After each round the robot clears the board and returns HOME. If neither
side has already reached two wins, press Enter to start the next round. Draws
complete a round but award no win; after three rounds, the side with more wins
takes the match, or the match is declared drawn when the scores are equal. Stop
both terminals with `Ctrl-C`.

Useful inspection commands:

```bash
ros2 topic echo /tic_tac_toe/board_state
ros2 topic echo /tic_tac_toe/status
ros2 run rqt_image_view rqt_image_view /tic_tac_toe/annotated
ros2 control list_controllers
```

`/tic_tac_toe/board_state` is a nine-character camera result (`x`, `o`, or `-`). The annotated image shows the detected board, cell ROIs, numbers, and classifications.

## Scene dimensions

- Panda base flange: `Z = 0.000 m`, fixed to Gazebo world on a dedicated 120 mm-radius pedestal.
- Ground plane: `2 x 2 m`, with a muted-green surface.
- Tabletop: `0.65 x 0.68 x 0.05 m`, with its top at `Z = 0.000 m`.
- Table X range: `0.245–0.895 m`; this leaves 165 mm between the tabletop and the pedestal.
- Board: `0.32 x 0.32 x 0.016 m`, top at `Z = 0.016 m`.
- X and O pieces are `30 mm` tall and `68 mm` wide. O pieces are solid blue
  cylinders; X pieces are board-coloured squares with a thin red X on top.
- Piece centre: `Z = 0.015 m` on the table and `Z = 0.031 m` on the board.
- Panda TCP grasp target: the piece middle height, `Z = 0.015 m` at the supply and `Z = 0.031 m` at the board.
- Safe approach height: `Z = 0.240 m`.

Each X or O model is attached to the Panda with Gazebo's native detachable fixed
joint while the gripper is closed and is released directly at piece-centre height.
Motion speed is controlled by
`motion_duration` and `gripper_duration` in `config/game.yaml`; smaller values
are faster. The supplied `1.25 s` / `0.45 s` settings are a moderate Gazebo
speed-up. Keep `motion_duration` at or above roughly `0.8 s` unless controller
tracking has been verified on the local machine.

The Panda opens to an `80 mm` aperture and closes to a commanded `66 mm`
aperture to grip either 68 mm piece around its external width. X and O use matching
five-piece rows centered on the board at `X = 0.57 m`, mirrored at
`Y = +/-0.26 m`, with `90 mm` between adjacent
centres so the gripper has room to approach each piece. The robot consumes both
rows from the far end (`X5` / `O5` through `X1` / `O1`). All pieces are
dynamic bodies transported by native fixed joints, avoiding delayed pose
updates and the previous magnetic-following appearance.
During cleanup, the Panda restores each row from piece 5 toward piece 1, moves
between returned pieces through a central `Z = 0.340 m` clearance waypoint,
and returns to `HOME` only after the entire board is clear. Once HOME, the final
supply poses are normalized to remove any small physics drift caused while the
gripper placed adjacent pieces. IK first uses the current posture and automatically retries
from a HOME numerical seed if the local solve cannot converge; this retry does
not command a physical HOME move.

Gazebo's DART backend may print `NameManager::issueNewName` messages for
temporary `fixed(n)` joints when the Panda model is created. They are
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
