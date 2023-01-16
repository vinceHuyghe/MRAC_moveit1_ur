
# pilz_robot_program

Based on the following [pkg](https://github.com/rivelinrobotics/move_group_sequence) which is in turn based on the [pilz_robot_programming](https://github.com/PilzDE/pilz_industrial_motion/tree/melodic-devel/pilz_robot_programming) but adapted to work with any manipulator with a moveit configuration. Requires the [Pilz Industrial Motion Planner](https://ros-planning.github.io/moveit_tutorials/doc/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html) to be installed.

## Examples

### Plan and execute

```python
sequencer.plan(Ptp(goal=[0, 0.5, 0.5, 0, 0, 0]))
sequencer.execute()
```

```python
sequencer.plan(Lin(goal=Pose(position=Point(0.2, 0, 0.8))))
sequencer.execute()
```

```python
sequencer.plan(Ptp(goal=Pose(position=Point(0.2, 0, 0.8))))
sequencer.execute()
```

```python
sequencer.plan(Circ(goal=Pose(position=Point(0.2, 0.2, 0.8)), center=Point(0.3, 0.1, 0.8)))
sequencer.execute()
```

### relative commands

```python
sequencer.plan(Ptp(goal=[0.1, 0, 0, 0, 0, 0], relative=True))
```

```python
sequencer.plan(Ptp(goal=[0.1, 0, 0, 0, 0, 0], relative=True))
```
### Sequence

```python
sequence = Sequence()
sequence.append(Lin(goal=Pose(position=Point(0.2, 0, 0.8)), vel_scale=0.1, acc_scale=0.1))
sequence.append(Circ(goal=Pose(position=Point(0.2, -0.2, 0.8)), center=Point(0.1, -0.1, 0.8), acc_scale=0.4))
sequence.append(Ptp(goal=pose_after_relative, vel_scale=0.2))

sequencer.plan(sequence)
```

see [pilz_robot_programming](https://github.com/PilzDE/pilz_industrial_motion/tree/melodic-devel/pilz_robot_programming) for more that can be adapted examples.
