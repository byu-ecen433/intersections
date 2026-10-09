# ECEN 433 - Lab 6: Intersections

Starter code for Lab 6. Your robot will stop at the red line, read the
intersection sign, pick a direction, drive through on wheel odometry, and hand
control back to your lane controller. You develop it against a simulated
intersection on this machine, then run the same nodes on your robot.

The full lab instructions live on the course site. This README covers the
repository itself: how it is laid out, and how to build and run it.


## Building and running

Build the Docker image. Run this from the root of this repository:

```bash
dts devel build -f
```

Then, on this machine, against the simulator:

```bash
dts devel run -X -L intersection_sim
```

- `-X` allows the container to open GUI windows. Without it the plot never
  appears.
- `-L intersection_sim` runs `launchers/intersection_sim.sh`.

On your robot, start your Lab 4 stack first, then:

```bash
dts devel build -H DUCKIEBOT_NAME -f
dts devel run   -H DUCKIEBOT_NAME -L intersections
```

To poke around inside the container instead of launching straight away:

```bash
dts devel run -X --cmd bash
```

and to attach a second terminal to a container that is already running:

```bash
dts devel run attach
```

New Python nodes must be executable or ROS will not find them:

```bash
chmod +x packages/intersection_control/src/<node_name>.py
```
