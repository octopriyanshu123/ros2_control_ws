# ros2_control_gauge_rover

   The example shows how to implement robot hardware with separate communication to each actuator.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_6/doc/userdoc.html).



 <link
    name="right_front_link">
    <inertial>
      <origin
        xyz="2.5713776339753E-05 -0.112526124429538 7.18011637226429E-05"
        rpy="0 0 0" />
      <mass
        value="140.357056498504" />
      <inertia
        ixx="0.156349262310722"
        ixy="0.000120004558843799"
        ixz="-0.000132718117351556"
        iyy="0.269360886360984"
        iyz="-1.4344844220023E-05"
        izz="0.155458190090457" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_1.dae" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_1.stl" />
      </geometry>
    </collision>
  </link>
  <joint
    name="right_front_wheel_to_base_joint"
    type="fixed">
    <origin
      xyz="-0.25584 0.030598 -0.31908"
      rpy="0 -0.42439 3.1416" />
    <parent
      link="body_link" />
    <child
      link="right_front_link" />
    <axis
      xyz="0 1 0" />
    <limit
      lower="0"
      upper="6.24"
      effort="10"
      velocity="10" />
  </joint>
  <link
    name="right_back_link">
    <inertial>
      <origin
        xyz="2.57137762453112E-05 -0.112526124428818 7.18011636879901E-05"
        rpy="0 0 0" />
      <mass
        value="140.357056499415" />
      <inertia
        ixx="0.15634926231084"
        ixy="0.000120004558878933"
        ixz="-0.000132718117466613"
        iyy="0.269360886361404"
        iyz="-1.43448442214138E-05"
        izz="0.155458190090766" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_2.dae" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_2.stl" />
      </geometry>
    </collision>
  </link>
  <joint
    name="right_back_wheel_to_base_joint"
    type="fixed">
    <origin
      xyz="-0.070756 0.030598 -0.31908"
      rpy="0 0.21622 3.1416" />
    <parent
      link="body_link" />
    <child
      link="right_back_link" />
    <axis
      xyz="0 1 0" />
    <limit
      lower="0"
      upper="6.24"
      effort="10"
      velocity="10" />
  </joint>
  <link
    name="left_front_link">
    <inertial>
      <origin
        xyz="2.5713776181796E-05 -0.112526124428283 7.18011637134697E-05"
        rpy="0 0 0" />
      <mass
        value="140.357056500094" />
      <inertia
        ixx="0.156349262310969"
        ixy="0.00012000455890453"
        ixz="-0.000132718117407678"
        iyy="0.26936088636172"
        iyz="-1.43448442265719E-05"
        izz="0.155458190090957" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_3.dae" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_3.stl" />
      </geometry>
    </collision>
  </link>
  <joint
    name="left_front_wheel_to_base_joint"
    type="fixed">
    <origin
      xyz="-0.25584 -0.024997 -0.31908"
      rpy="0 -0.76081 0" />
    <parent
      link="body_link" />
    <child
      link="left_front_link" />
    <axis
      xyz="0 1 0" />
    <limit
      lower="0"
      upper="6.24"
      effort="10"
      velocity="10" />
  </joint>
  <link
    name="left_back_link">
    <inertial>
      <origin
        xyz="2.57137761885406E-05 -0.112526124428341 7.18011637099447E-05"
        rpy="0 0 0" />
      <mass
        value="140.357056500021" />
      <inertia
        ixx="0.156349262310955"
        ixy="0.000120004558901742"
        ixz="-0.000132718117415487"
        iyy="0.269360886361686"
        iyz="-1.43448442256035E-05"
        izz="0.155458190090937" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_4.dae" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_4.stl" />
      </geometry>
    </collision>
  </link>
  <joint
    name="left_back_wheel_to_base_joint"
    type="fixed">
    <origin
      xyz="-0.070756 -0.024997 -0.31908"
      rpy="0 0.30744 0" />
    <parent
      link="body_link" />
    <child
      link="left_back_link" />
    <axis
      xyz="0 1 0" />
    <limit
      lower="0"
      upper="6.24"
      effort="10"
      velocity="10" />
  </joint>













  <link
    name="motor_1_link">
    <inertial>
      <origin
        xyz="-0.0200799823614145 -3.20495466103743E-05 0.00567272606006936"
        rpy="0 0 0" />
      <mass
        value="2.59097198613766" />
      <inertia
        ixx="0.00163707671968947"
        ixy="4.05834569748E-06"
        ixz="-0.000187935606363205"
        iyy="0.00178436896126707"
        iyz="3.66965892711528E-06"
        izz="0.00152277803547436" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_5.dae" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_5.stl" />
      </geometry>
    </collision>
  </link>
  
  
  <joint
    name="arm_base_joint"
    type="revolute">
    <origin
      xyz="-0.2458 -0.1472 0"
      rpy="0 0 1.5708" />
    <parent
      link="body_link" />
    <child
      link="motor_1_link" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="0"
      upper="6.24"
      effort="10"
      velocity="10" />
  </joint>
  <link
    name="motor_2_link">
    <inertial>
      <origin
        xyz="0.0419734207030258 0.240056643422739 0.103988160889993"
        rpy="0 0 0" />
      <mass
        value="3.11397900589646" />
      <inertia
        ixx="0.0105893542585918"
        ixy="5.6511164525869E-08"
        ixz="-3.18900925698064E-08"
        iyy="0.00153774441920444"
        iyz="-9.98026406358191E-07"
        izz="0.0112219847238024" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_6.dae" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_6.stl" />
      </geometry>
    </collision>
  </link>
  <joint
    name="arm_shoulder_joint"
    type="revolute">
    <origin
      xyz="0 0 0"
      rpy="-1.5708 3.14 1.5708" />
    <parent
      link="motor_1_link" />
    <child
      link="motor_2_link" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="0"
      upper="6.24"
      effort="10"
      velocity="10" />
  </joint>
  <link
    name="motor_3_link">
    <inertial>
      <origin
        xyz="1.65885532732446E-06 0.16725738983806 0.0588578977856613"
        rpy="0 0 0" />
      <mass
        value="3.01078180932307" />
      <inertia
        ixx="0.00717136557548414"
        ixy="1.01381336321339E-07"
        ixz="2.0734451611804E-08"
        iyy="0.00137417115863493"
        iyz="-1.82688382332719E-05"
        izz="0.00774289023463593" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_7.dae" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/octobotics/arm_gzebo_ws/src/arm_gazebo/meshes/link_7.stl" />
      </geometry>
    </collision>
  </link>
  <joint
    name="arm_elbow_joint"
    type="revolute">
    <origin
      xyz="0.041975 0.31418 0.10675"
      rpy="0 0 0" />
    <parent
      link="motor_2_link" />
    <child
      link="motor_3_link" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="0"
      upper="6.24"
      effort="10"
      velocity="10" />
  </joint>








  <joint name="${prefix}joint1" type="continuous">
    <parent link="${prefix}base_link"/>
    <child link="${prefix}link1"/>
    <origin xyz="0 ${width} ${height1 - axel_offset}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.7"/>
    <limit effort="100" velocity="1.0"/>
  </joint>

  <!-- Middle Link -->
  <link name="${prefix}link1">
    <collision>
      <origin xyz="0 0 ${height2/2 - axel_offset}" rpy="0 0 0"/>
      <geometry>
        <box size="${width} ${width} ${height2}"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 ${height2/2 - axel_offset}" rpy="0 0 0"/>
      <geometry>
        <box size="${width} ${width} ${height2}"/>
      </geometry>
      <material name="yellow"/>
    </visual>

    <inertial>
      <origin xyz="0 0 ${height2/2 - axel_offset}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia
        ixx="${mass / 12.0 * (width*width + height2*height2)}" ixy="0.0" ixz="0.0"
        iyy="${mass / 12.0 * (height2*height2 + width*width)}" iyz="0.0"
        izz="${mass / 12.0 * (width*width + width*width)}"/>
    </inertial>
  </link>

  <joint name="${prefix}joint2" type="continuous">
    <parent link="${prefix}link1"/>
    <child link="${prefix}link2"/>
    <origin xyz="0 ${width} ${height2 - axel_offset*2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.7"/>
    <limit effort="100" velocity="1.0"/>
  </joint>

  <!-- Top Link -->
  <link name="${prefix}link2">
    <collision>
      <origin xyz="0 0 ${height3/2 - axel_offset}" rpy="0 0 0"/>
      <geometry>
        <box size="${width} ${width} ${height3}"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 ${height3/2 - axel_offset}" rpy="0 0 0"/>
      <geometry>
        <box size="${width} ${width} ${height3}"/>
      </geometry>
      <material name="orange"/>
    </visual>

    <inertial>
      <origin xyz="0 0 ${height3/2 - axel_offset}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia
        ixx="${mass / 12.0 * (width*width + height3*height3)}" ixy="0.0" ixz="0.0"
        iyy="${mass / 12.0 * (height3*height3 + width*width)}" iyz="0.0"
        izz="${mass / 12.0 * (width*width + width*width)}"/>
    </inertial>
  </link>

  <joint name="${prefix}tool_joint" type="fixed">
    <origin xyz="0 0 1" rpy="0 0 0" />
    <parent link="${prefix}link2"/>
    <child link="${prefix}tool_link" />
  </joint>

  <!-- Tool Link -->
  <link name="${prefix}tool_link">
  </link>