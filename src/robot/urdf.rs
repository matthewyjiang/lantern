//! URDF: links, joints, visuals and materials, and forward kinematics. Plain URDF only (xacro
//! must be expanded first); collision and inertial elements are ignored.

use std::collections::HashMap;

use glam::{DQuat, DVec3};
use roxmltree::{Document, Node};

use crate::scene::Pose;

/// A robot's kinematic tree. Link 0 is the root; `joints` is in topological order (every
/// joint's parent link is placed before it), so one pass over it poses every link.
#[derive(Debug)]
pub struct Robot {
    pub name: String,
    pub links: Vec<String>,
    pub joints: Vec<Joint>,
    pub visuals: Vec<Visual>,
}

#[derive(Debug)]
pub struct Joint {
    pub name: String,
    pub parent: usize,
    pub child: usize,
    /// `parent_T_joint` at zero position.
    pub origin: Pose,
    pub motion: Motion,
    /// Follows another joint: position = multiplier × theirs + offset.
    pub mimic: Option<Mimic>,
}

#[derive(Debug, PartialEq)]
pub enum Motion {
    Fixed,
    /// Revolute and continuous: radians about a unit axis in the joint frame.
    Rotate(DVec3),
    /// Prismatic: metres along a unit axis.
    Slide(DVec3),
}

#[derive(Debug)]
pub struct Mimic {
    pub joint: String,
    pub multiplier: f64,
    pub offset: f64,
}

#[derive(Debug, Clone)]
pub struct Visual {
    pub link: usize,
    /// `link_T_visual`.
    pub origin: Pose,
    pub geometry: Geometry,
    /// sRGB and alpha; `None` when the URDF gives no colour.
    pub color: Option<[f32; 4]>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Geometry {
    /// The filename as written (relative, absolute, `file://` or `package://`) and its scale.
    Mesh { filename: String, scale: DVec3 },
    Box { size: DVec3 },
    /// Along z, centred on the origin.
    Cylinder { radius: f64, length: f64 },
    Sphere { radius: f64 },
}

impl Robot {
    pub fn parse(xml: &str) -> Result<Robot, String> {
        let doc = Document::parse(xml).map_err(|e| format!("not valid XML: {e}"))?;
        let robot = doc.root_element();
        if robot.tag_name().name() != "robot" {
            return Err(format!("the root element is <{}>, not <robot>", robot.tag_name().name()));
        }
        if robot.descendants().any(|n| n.tag_name().namespace().is_some_and(|ns| ns.contains("xacro"))) {
            return Err("this is xacro, not URDF; expand it first (xacro robot.urdf.xacro > robot.urdf)".into());
        }
        let materials: HashMap<&str, [f32; 4]> = robot.children()
            .filter(|n| n.has_tag_name("material"))
            .filter_map(|m| Some((m.attribute("name")?, color_of(m)?)))
            .collect();

        let link_nodes: Vec<Node> = robot.children().filter(|n| n.has_tag_name("link")).collect();
        let joint_nodes: Vec<Node> = robot.children().filter(|n| n.has_tag_name("joint")).collect();
        let mut names: Vec<String> = Vec::new();
        for l in &link_nodes {
            names.push(attr(l, "name")?.to_string());
        }
        let index = |name: &str| names.iter().position(|n| n == name).ok_or_else(|| format!("joint refers to link {name}, which isn't defined"));

        let mut joints = Vec::new();
        for j in &joint_nodes {
            let name = attr(j, "name")?.to_string();
            let child_of = |tag: &str| j.children().find(|n| n.has_tag_name(tag)).ok_or_else(|| format!("joint {name} has no <{tag}>"));
            let parent = index(attr(&child_of("parent")?, "link")?)?;
            let child = index(attr(&child_of("child")?, "link")?)?;
            let axis = match j.children().find(|n| n.has_tag_name("axis")) {
                Some(a) => vec3(a.attribute("xyz").unwrap_or("1 0 0"))?.try_normalize().ok_or_else(|| format!("joint {name} has a zero axis"))?,
                None => DVec3::X,
            };
            let motion = match attr(j, "type")? {
                "revolute" | "continuous" => Motion::Rotate(axis),
                "prismatic" => Motion::Slide(axis),
                "fixed" => Motion::Fixed,
                other => return Err(format!("joint {name} is {other}; Lantern handles revolute, continuous, prismatic and fixed")),
            };
            let mimic = j.children().find(|n| n.has_tag_name("mimic")).map(|m| -> Result<Mimic, String> {
                Ok(Mimic { joint: attr(&m, "joint")?.to_string(), multiplier: num(m.attribute("multiplier").unwrap_or("1"))?, offset: num(m.attribute("offset").unwrap_or("0"))? })
            }).transpose()?;
            joints.push(Joint { name, parent, child, origin: origin_of(*j)?, motion, mimic });
        }

        // The root is the one link no joint has as its child; then order joints from it.
        let mut parent_of = vec![None; names.len()];
        for (k, j) in joints.iter().enumerate() {
            if let Some(other) = parent_of[j.child].replace(k) {
                return Err(format!("link {} is the child of both {} and {}", names[j.child], joints[other].name, j.name));
            }
        }
        let roots: Vec<usize> = (0..names.len()).filter(|&l| parent_of[l].is_none()).collect();
        let root = match roots[..] {
            [r] => r,
            [] => return Err(if names.is_empty() { "no links".into() } else { "the joints form a loop: no root link".into() }),
            _ => return Err(format!("{} root links ({}); a URDF has one", roots.len(), roots.iter().map(|&r| names[r].as_str()).collect::<Vec<_>>().join(", "))),
        };
        let mut order = Vec::with_capacity(joints.len());
        let mut frontier = vec![root];
        while let Some(link) = frontier.pop() {
            for (k, j) in joints.iter().enumerate().filter(|(_, j)| j.parent == link) {
                order.push(k);
                frontier.push(j.child);
            }
        }
        if order.len() < joints.len() {
            return Err("some joints form a loop, unreachable from the root link".into());
        }
        // Renumber links so the root is 0, in the order they're placed.
        let mut renumber = vec![0; names.len()];
        let mut links = vec![names[root].clone()];
        for &k in &order {
            renumber[joints[k].child] = links.len();
            links.push(names[joints[k].child].clone());
        }
        let mut slots: Vec<Option<Joint>> = joints.into_iter().map(Some).collect();
        let joints: Vec<Joint> = order.iter().map(|&k| {
            let mut j = slots[k].take().unwrap();
            j.parent = renumber[j.parent];
            j.child = renumber[j.child];
            j
        }).collect();

        let mut visuals = Vec::new();
        for (old, l) in link_nodes.iter().enumerate() {
            for v in l.children().filter(|n| n.has_tag_name("visual")) {
                let g = v.children().find(|n| n.has_tag_name("geometry")).and_then(|g| g.children().find(Node::is_element))
                    .ok_or_else(|| format!("a visual of link {} has no geometry", names[old]))?;
                let geometry = match g.tag_name().name() {
                    "mesh" => Geometry::Mesh { filename: attr(&g, "filename")?.to_string(), scale: vec3(g.attribute("scale").unwrap_or("1 1 1"))? },
                    "box" => Geometry::Box { size: vec3(attr(&g, "size")?)? },
                    "cylinder" => Geometry::Cylinder { radius: num(attr(&g, "radius")?)?, length: num(attr(&g, "length")?)? },
                    "sphere" => Geometry::Sphere { radius: num(attr(&g, "radius")?)? },
                    other => return Err(format!("link {} has unknown geometry <{other}>", names[old])),
                };
                // A material colours inline, or names one defined at the top level.
                let color = v.children().find(|n| n.has_tag_name("material"))
                    .and_then(|m| color_of(m).or_else(|| materials.get(m.attribute("name")?).copied()));
                visuals.push(Visual { link: renumber[old], origin: origin_of(v)?, geometry, color });
            }
        }
        Ok(Robot { name: robot.attribute("name").unwrap_or("robot").to_string(), links, joints, visuals })
    }

    /// Joints that take a position (not fixed, not mimicking another), in tree order.
    pub fn movable(&self) -> impl Iterator<Item = &str> {
        self.joints.iter().filter(|j| j.motion != Motion::Fixed && j.mimic.is_none()).map(|j| j.name.as_str())
    }

    /// `root_T_link` for every link, given joint positions by name; a joint with no position sits
    /// at zero.
    pub fn link_poses(&self, positions: &HashMap<String, f64>) -> Vec<Pose> {
        let mut poses = vec![Pose::IDENTITY; self.links.len()];
        for j in &self.joints {
            let q = match &j.mimic {
                Some(m) => positions.get(&m.joint).map_or(0.0, |q| m.multiplier * q + m.offset),
                None => positions.get(&j.name).copied().unwrap_or(0.0),
            };
            let motion = match j.motion {
                Motion::Fixed => Pose::IDENTITY,
                Motion::Rotate(axis) => Pose { translation: DVec3::ZERO, rotation: DQuat::from_axis_angle(axis, q) },
                Motion::Slide(axis) => Pose { translation: axis * q, rotation: DQuat::IDENTITY },
            };
            poses[j.child] = poses[j.parent].then(&j.origin).then(&motion);
        }
        poses
    }
}

fn attr<'a>(n: &Node<'a, '_>, name: &str) -> Result<&'a str, String> {
    n.attribute(name).ok_or_else(|| format!("<{}> has no {name}", n.tag_name().name()))
}

fn num(s: &str) -> Result<f64, String> {
    s.trim().parse().map_err(|_| format!("{s:?} is not a number"))
}

fn vec3(s: &str) -> Result<DVec3, String> {
    let v: Vec<f64> = s.split_whitespace().map(num).collect::<Result<_, _>>()?;
    match v[..] {
        [x, y, z] => Ok(DVec3::new(x, y, z)),
        _ => Err(format!("{s:?} is not three numbers")),
    }
}

/// An element's `<origin xyz rpy>`; identity when absent. URDF's rpy is fixed-axis roll about x,
/// then pitch about y, then yaw about z.
fn origin_of(n: Node) -> Result<Pose, String> {
    let Some(o) = n.children().find(|c| c.has_tag_name("origin")) else { return Ok(Pose::IDENTITY) };
    let t = vec3(o.attribute("xyz").unwrap_or("0 0 0"))?;
    let r = vec3(o.attribute("rpy").unwrap_or("0 0 0"))?;
    let rotation = DQuat::from_rotation_z(r.z) * DQuat::from_rotation_y(r.y) * DQuat::from_rotation_x(r.x);
    Ok(Pose { translation: t, rotation })
}

/// A `<material>`'s `<color rgba>`, if it has one.
fn color_of(m: Node) -> Option<[f32; 4]> {
    let rgba = m.children().find(|c| c.has_tag_name("color"))?.attribute("rgba")?;
    let v: Vec<f32> = rgba.split_whitespace().filter_map(|x| x.parse().ok()).collect();
    v.try_into().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    const MED7: &str = include_str!("testdata/med7.urdf");

    fn deg(q: [f64; 7]) -> HashMap<String, f64> {
        q.iter().enumerate().map(|(k, d)| (format!("lbr_joint_{k}"), d.to_radians())).collect()
    }

    fn assert_pose(robot: &Robot, poses: &[Pose], link: &str, t: [f64; 3], r: [f64; 9]) {
        let p = poses[robot.links.iter().position(|l| l == link).unwrap()];
        assert!((p.translation - DVec3::from(t)).length() < 1e-9, "{link}: {:?} vs {t:?}", p.translation);
        let m = glam::DMat3::from_quat(p.rotation);
        for row in 0..3 {
            for col in 0..3 {
                assert!((m.col(col)[row] - r[3 * row + col]).abs() < 1e-9, "{link}: R[{row}][{col}] = {} vs {}", m.col(col)[row], r[3 * row + col]);
            }
        }
    }

    // Reference values from pinocchio 4.1.0 on the same URDF (buildModelFromXML, then
    // updateFramePlacements; R row-major). Home is kpd's HOME_Q; reach turns every joint.
    #[test]
    fn med7_forward_kinematics_matches_pinocchio() {
        let robot = Robot::parse(MED7).unwrap();
        assert_eq!(robot.links[0], "world");
        assert_eq!(robot.movable().collect::<Vec<_>>(), (0..7).map(|k| format!("lbr_joint_{k}")).collect::<Vec<_>>());
        assert_eq!(robot.visuals.len(), 8);

        let home = robot.link_poses(&deg([0.0, 30.0, 0.0, -60.0, 0.0, 90.0, 0.0]));
        assert_pose(&robot, &home, "lbr_link_ee", [0.6, 0.0, 0.560410162], [-1.0, 0.0, 2.83e-16, 0.0, 1.0, 0.0, -2.83e-16, 0.0, -1.0]);
        assert_pose(&robot, &home, "lbr_link_4", [0.2, 0.0105, 0.686410162], [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0]);

        let reach = robot.link_poses(&deg([50.0, 55.0, -20.0, -70.0, 30.0, 60.0, -40.0]));
        assert_pose(&robot, &reach, "lbr_link_ee", [0.499236303, 0.426029864, 0.234258961],
            [-0.469207122, -0.877834063, -0.096187502, -0.880154088, 0.455994247, 0.131901583, -0.071926755, 0.146548985, -0.986584987]);
        assert_pose(&robot, &reach, "lbr_link_4", [0.204381964, 0.258922912, 0.566488826],
            [-0.286682329, -0.593747647, 0.751849036, -0.523639813, 0.754301309, 0.39601879, -0.802255937, -0.2801665, -0.527154763]);
    }

    // A gripper whose second finger mimics the first, mirrored: and the link order puts parents
    // before children even when the file lists the child's joint first.
    #[test]
    fn mimic_joints_follow_and_out_of_order_joints_still_pose() {
        let urdf = r#"<robot name="g">
            <joint name="right" type="prismatic"><parent link="palm"/><child link="r"/><axis xyz="0 -1 0"/>
              <mimic joint="left" multiplier="1" offset="0"/></joint>
            <link name="r"/><link name="l"/>
            <joint name="left" type="prismatic"><parent link="palm"/><child link="l"/><axis xyz="0 1 0"/></joint>
            <joint name="mount" type="fixed"><parent link="base"/><child link="palm"/><origin xyz="0 0 0.1"/></joint>
            <link name="palm"/><link name="base"/>
          </robot>"#;
        let robot = Robot::parse(urdf).unwrap();
        assert_eq!(robot.links[0], "base");
        assert_eq!(robot.movable().collect::<Vec<_>>(), ["left"]);
        let poses = robot.link_poses(&HashMap::from([("left".to_string(), 0.02)]));
        let at = |name: &str| poses[robot.links.iter().position(|l| l == name).unwrap()].translation;
        assert!((at("l") - DVec3::new(0.0, 0.02, 0.1)).length() < 1e-12);
        assert!((at("r") - DVec3::new(0.0, -0.02, 0.1)).length() < 1e-12);
    }

    #[test]
    fn xacro_and_two_roots_are_refused_with_the_reason() {
        let xacro = r#"<robot name="x" xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:property name="a" value="1"/></robot>"#;
        assert!(Robot::parse(xacro).unwrap_err().contains("expand it first"));
        let two = r#"<robot name="t"><link name="a"/><link name="b"/></robot>"#;
        assert!(Robot::parse(two).unwrap_err().contains("2 root links"));
    }
}
