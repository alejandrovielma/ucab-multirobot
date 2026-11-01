# simulation/

| Folder | Contents |
|---|---|
| `webots/` | `pista_ucab.wbt` world, `RecolectorUCAB` and `CuboMadera` PROTOs, `recolector_teleop` keyboard controller |

The world has two collectors: one is ours and the other is used to rehearse with the other team's collector as an obstacle. Perry is not modelled.

With ROS 2: the `pista_ucab_ros.wbt` world is the same, but Ferb (our robot) has an `<extern>` controller and is driven by the `webots_bridge` node, while Phineas (the other collector) stays still (`src/recolector_sim`). Launch it with `ros2 launch bringup simulation.launch.py` or with `docker compose up sim` (see `docker/README.md`). If you change `pista_ucab.wbt`, regenerate the variant with `python3 tools/generate_ros_world.py`.
