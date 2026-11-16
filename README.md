<img src="assets/icon.svg" width="64" height="64" alt="">

# Lantern

A small RViz stand-in in Rust: draws point clouds, TF, poses, a robot model and camera images
live from any Foxglove WebSocket source, including foxglove_bridge on ROS 2.

![Lantern drawing a KUKA arm beside a table scan coloured by intensity, with TF axes, labelled objects and the side panel.](docs/screenshots/robot-home-sdk.png)

```sh
git clone https://github.com/matthewyjiang/lantern.git && cd lantern
cargo run --release      # needs Rust 1.88+ and a Vulkan driver; connects to ws://localhost:8765
```

**Docs:** [matthewyjiang.github.io/lantern](https://matthewyjiang.github.io/lantern/)
