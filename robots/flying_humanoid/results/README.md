# Static wrench optimization results

`static_wrench_optimizer.launch` writes one YAML file per wall-normal EE offset
and numeric torque axis, for example
`static_wrench/offset_0.050m/axis_1_0_0.yaml`.

```bash
roslaunch flying_humanoid static_wrench_optimizer.launch \
  ee_offset:=0.05 ee_axis:="[1, 0, 0]"
```

RViz, Gazebo, and MuJoCo use the same `ee_offset` and `ee_axis` arguments to
select that result.
Generated files are runtime artifacts and are not tracked by Git.

Existing results can be summarized without rerunning any optimization:

```bash
rosrun flying_humanoid summarize_static_wrench_results.py \
  $(rospack find flying_humanoid)/results/static_wrench
```

This updates `summary.csv` and `summary.md` in that directory.

Results from the earlier 0.45 m wall-target height are retained under
`static_wrench/archive_z_0.450m/` and are excluded from the current summary.
