<img src="assets/icon.svg" width="64" height="64" alt="">

# Lantern

A small RViz stand-in in Rust: draws point clouds, TF, poses, a robot model and camera images
live from any Foxglove WebSocket source, including foxglove_bridge on ROS 2. Its panels tile into
a layout you can save to a file.

![Lantern drawing a KUKA arm beside a table scan coloured by intensity, with TF axes, labelled objects and the side panel.](docs/screenshots/robot-home-sdk.png)

```sh
curl -fsSL https://raw.githubusercontent.com/matthewyjiang/lantern/main/scripts/install.sh | sh
lantern      # Linux with a Vulkan driver; connects to ws://localhost:8765
```

Or build it yourself (Rust 1.88+): `git clone https://github.com/matthewyjiang/lantern.git && cd lantern && cargo run --release`.

**Docs:** [matthewyjiang.github.io/lantern](https://matthewyjiang.github.io/lantern/)
