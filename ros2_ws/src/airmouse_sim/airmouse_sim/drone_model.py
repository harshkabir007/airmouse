"""Builds model://airmouse = ArduPilot's iris_with_ardupilot + AirMouse sensors.

The iris model is copied from ardupilot_gazebo at launch time and the sensors are
injected, so the ArduPilot plugin config always matches the installed plugin version.

Sensors mimic the real payload:
  * Livox Mid-360-like LiDAR, mounted UPSIDE-DOWN under the frame
    (360 deg x [-7, +52] deg -> after inversion it sees the floor and walls), 10 Hz
  * IMU on the same link (the Mid-360 has a built-in IMU), 200 Hz
  * forward camera tilted 15 deg down, 640x480 @ 15 Hz
"""
import os
import xml.etree.ElementTree as ET

LIDAR_POSE = '0 0 -0.06 3.14159265 0 0'     # keep in sync with config/sim.yaml (lidar_mount_*)
CAMERA_POSE = '0.15 0 0.02 0 0.26 0'        # keep in sync with config/sim.yaml (camera_mount_*)

SENSORS = """<root>
<link name="lidar_link">
  <pose relative_to="{parent}">{lidar_pose}</pose>
  <inertial><mass>0.02</mass>
    <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
  </inertial>
  <visual name="lidar_visual">
    <geometry><cylinder><radius>0.033</radius><length>0.06</length></cylinder></geometry>
    <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>
  </visual>
  <sensor name="lidar" type="gpu_lidar">
    <gz_frame_id>lidar_link</gz_frame_id>
    <topic>lidar</topic>
    <update_rate>10</update_rate>
    <always_on>1</always_on>
    <visualize>false</visualize>
    <lidar>
      <scan>
        <horizontal><samples>720</samples><resolution>1</resolution>
          <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
        <vertical><samples>32</samples><resolution>1</resolution>
          <min_angle>-0.1222</min_angle><max_angle>0.9076</max_angle></vertical>
      </scan>
      <range><min>0.35</min><max>40</max><resolution>0.01</resolution></range>
      <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
    </lidar>
  </sensor>
  <sensor name="lidar_imu" type="imu">
    <gz_frame_id>lidar_link</gz_frame_id>
    <topic>imu</topic>
    <update_rate>200</update_rate>
    <always_on>1</always_on>
  </sensor>
</link>
<joint name="lidar_joint" type="fixed"><parent>{parent}</parent><child>lidar_link</child></joint>
<link name="camera_link">
  <pose relative_to="{parent}">{camera_pose}</pose>
  <inertial><mass>0.01</mass>
    <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
  </inertial>
  <sensor name="camera" type="camera">
    <gz_frame_id>camera_link</gz_frame_id>
    <topic>camera/image</topic>
    <update_rate>15</update_rate>
    <always_on>1</always_on>
    <camera>
      <horizontal_fov>1.57</horizontal_fov>
      <image><width>640</width><height>480</height><format>R8G8B8</format></image>
      <clip><near>0.05</near><far>30</far></clip>
    </camera>
  </sensor>
</link>
<joint name="camera_joint" type="fixed"><parent>{parent}</parent><child>camera_link</child></joint>
</root>"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>airmouse</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Iris (ArduPilot SITL) with Mid-360-like LiDAR, IMU and camera</description>
</model>
"""


def find_iris(resource_paths):
    for p in resource_paths:
        f = os.path.join(p, 'iris_with_ardupilot', 'model.sdf')
        if os.path.isfile(f):
            return f
    raise FileNotFoundError('iris_with_ardupilot not found in GZ_SIM_RESOURCE_PATH - is ardupilot_gazebo installed?')


def _base_link_name(model):
    """'base_link' if the iris links are merged into this model, else '<nested>::base_link'."""
    if model.find("link[@name='base_link']") is not None:
        return 'base_link'
    for inc in model.findall('include'):
        uri = inc.findtext('uri', '')
        if 'iris_with_standoffs' in uri:
            if inc.get('merge') == 'true':
                return 'base_link'
            return (inc.findtext('name') or 'iris_with_standoffs') + '::base_link'
    return 'base_link'


def build(out_models_dir, resource_paths):
    """Writes <out_models_dir>/airmouse/{model.sdf, model.config}."""
    tree = ET.parse(find_iris(resource_paths))
    model = tree.getroot().find('model')
    model.set('name', 'airmouse')
    extra = ET.fromstring(SENSORS.format(parent=_base_link_name(model),
                                         lidar_pose=LIDAR_POSE, camera_pose=CAMERA_POSE))
    for el in list(extra):
        model.append(el)
    out = os.path.join(out_models_dir, 'airmouse')
    os.makedirs(out, exist_ok=True)
    tree.write(os.path.join(out, 'model.sdf'), xml_declaration=True, encoding='utf-8')
    with open(os.path.join(out, 'model.config'), 'w') as f:
        f.write(MODEL_CONFIG)
    return out
