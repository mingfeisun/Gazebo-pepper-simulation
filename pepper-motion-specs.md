## Pepper Joints

### Movable parts

* Joints

![Alt pepper joints](http://doc.aldebaran.com/2-0/_images/juliet_joints.png)

* Head motion

![Alt head motion](http://doc.aldebaran.com/2-0/_images/joint_head.png)

* Should, elbow and wrist (Arm)

![Alt head motion](http://doc.aldebaran.com/2-0/_images/joint_left_arm.png)

![Alt head motion](http://doc.aldebaran.com/2-0/_images/joint_right_arm.png)

* Hip 

![Alt head motion](http://doc.aldebaran.com/2-0/_images/joint_leg.png)

### Rotation definition
RPY --> XYZ

![Alt head motion](http://doc.aldebaran.com/2-0/_images/rollPitchYaw.png)

### ROS Pepper limitation

* LShoulderPitch

Topic: /pepper_dcm/LShoulderPitch_position_controller/command
```xml
<joint name="LShoulderPitch" type="revolute">
	<parent link="torso"/>
	<child link="LShoulder"/>
	<origin rpy="0 0 0" xyz="-0.057 0.14974 0.08682"/>
	<axis xyz="0 1.0 0"/>
	<limit effort="5.428" lower="-2.08567" upper="2.08567" velocity="7.33998"/>
</joint>/pepper_dcm/HeadPitch_position_controller/command

```

* LShoudlerRoll

Topic: /pepper_dcm/LShoulderRoll_position_controller/command
```xml
<joint name="LShoulderRoll" type="revolute">
	<parent link="LShoulder"/>
	<child link="LBicep"/>
	<origin rpy="0 0 0" xyz="0 0 0"/>
	<axis xyz="0 0 1.0"/>
	<limit effort="2.666" lower="0.00872665" upper="1.56207" velocity="9.22756"/>
</joint>
```

* LElbowYaw

Topic: /pepper_dcm/LElbowYaw_position_controller/command
```xml
<joint name="LElbowYaw" type="revolute">
	<parent link="LBicep"/>
	<child link="LElbow"/>
	<origin rpy="0 -0.157079 0" xyz="0.1812 0.015 0.00013"/>
	<axis xyz="1.0 0 0"/>
	<limit effort="5.428" lower="-2.08567" upper="2.08567" velocity="7.33998"/>
</joint>
```

* LElbowRoll

Topic: /pepper_dcm/LElbowRoll_position_controller/command
```xml
<joint name="LElbowRoll" type="revolute">
	<parent link="LElbow"/>
	<child link="LForeArm"/>
	<origin rpy="0 0 0" xyz="0 0 0"/>
	<axis xyz="0 0 1.0"/>
	<limit effort="2.666" lower="-1.56207" upper="-0.00872665" velocity="9.22756"/>
</joint>
```

* LWristYaw

Topic: /pepper_dcm/LWristYaw_position_controller/command
```xml
<joint name="LWristYaw" type="revolute">
	<parent link="LForeArm"/>
	<child link="l_wrist"/>
	<origin rpy="0 0 0" xyz="0.15 0 0"/>
	<axis xyz="1.0 0 0"/>
	<limit effort="0.2014" lower="-1.82387" upper="1.82387" velocity="17.3835"/>
</joint>
```

* LHand

Topic: /pepper_dcm/LHand_position_controller/command
```xml
<joint name="LHand" type="revolute">
	<parent link="l_wrist"/>
	<child link="l_gripper"/>
	<origin rpy="0 0 0" xyz="0.025 0 0"/>
	<axis xyz="1.0 0 0"/>
	<limit effort="0.144" lower="0.02" upper="0.98" velocity="12.56"/>
</joint>
```

* RShoulderPitch

Topic: /pepper_dcm/RShoulderPitch_position_controller/command
```xml
<joint name="RShoulderPitch" type="revolute">
	<parent link="torso"/>
	<child link="RShoulder"/>
	<origin rpy="0 0 0" xyz="-0.057 -0.14974 0.08682"/>
	<axis xyz="0 1.0 0"/>
	<limit effort="5.428" lower="-2.08567" upper="2.08567" velocity="7.33998"/>
</joint>
```

* RShoulderRoll

Topic: /pepper_dcm/RShoulderRoll_position_controller/command
```xml
<joint name="RShoulderRoll" type="revolute">
	<parent link="RShoulder"/>
	<child link="RBicep"/>
	<origin rpy="0 0 0" xyz="0 0 0"/>
	<axis xyz="0 0 1.0"/>
	<limit effort="2.666" lower="-1.56207" upper="-0.00872665" velocity="9.22756"/>
</joint>
```

* RElbowYaw

Topic: /pepper_dcm/RElbowYaw_position_controller/command
```xml
<joint name="RElbowYaw" type="revolute">
	<parent link="RBicep"/>
	<child link="RElbow"/>
	<origin rpy="0 -0.157079 0" xyz="0.1812 -0.015 0.00013"/>
	<axis xyz="1.0 0 0"/>
	<limit effort="5.428" lower="-2.08567" upper="2.08567" velocity="7.33998"/>
</joint>
```

* RElbowRoll

Topic: /pepper_dcm/RElbowRoll_position_controller/command
```xml
<joint name="RElbowRoll" type="revolute">
	<parent link="RElbow"/>
	<child link="RForeArm"/>
	<origin rpy="0 0 0" xyz="0 0 0"/>
	<axis xyz="0 0 1.0"/>
	<limit effort="2.666" lower="0.00872665" upper="1.56207" velocity="9.22756"/>
</joint>
```

* RWristYaw

Topic: /pepper_dcm/RWristYaw_position_controller/command
```xml
<joint name="RWristYaw" type="revolute">
	<parent link="RForeArm"/>
	<child link="r_wrist"/>
	<origin rpy="0 0 0" xyz="0.15 0 0"/>
	<axis xyz="1.0 0 0"/>
	<limit effort="0.2014" lower="-1.82387" upper="1.82387" velocity="17.3835"/>
</joint>
```

* RHand

Topic: /pepper_dcm/RHand_position_controller/command
```xml
<joint name="RHand" type="revolute">
	<parent link="r_wrist"/>
	<child link="r_gripper"/>
	<origin rpy="0 0 0" xyz="0.025 0 0"/>
	<axis xyz="1.0 0 0"/>
	<limit effort="0.144" lower="0.02" upper="0.98" velocity="12.56"/>
</joint>
```

* HeadYaw

Topic: /pepper_dcm/HeadYaw_position_controller/command
```xml
<joint name="HeadYaw" type="revolute">
	<parent link="torso"/>
	<child link="Neck"/>
	<origin rpy="0 0 0" xyz="-0.038 0 0.1699"/>
	<axis xyz="0 0 1.0"/>
	<limit effort="5.428" lower="-2.08567" upper="2.08567" velocity="7.33998"/>
</joint>
```

* HeadPitch

Topic: /pepper_dcm/HeadPitch_position_controller/command
```xml
<joint name="HeadPitch" type="revolute">
	<parent link="Neck"/>
	<child link="Head"/>
	<origin rpy="0 0 0" xyz="0 0 0"/>
	<axis xyz="0 1.0 0"/>
	<limit effort="2.666" lower="-0.706858" upper="0.637045" velocity="9.22756"/>
</joint>
```

* HipRoll

Topic: /pepper_dcm/HipRoll_position_controller/command
```xml
<joint name="HipRoll" type="revolute">
	<parent link="torso"/>
	<child link="Hip"/>
	<origin rpy="0 0 0" xyz="2e-05 0 -0.139"/>
	<axis xyz="1.0 0 0"/>
	<limit effort="10.1884" lower="-0.514872" upper="0.514872" velocity="2.27032"/>
</joint>
```

* HipPitch

Topic: /pepper_dcm/HipPitch_position_controller/command
```xml
<joint name="HipPitch" type="revolute">
	<parent link="Hip"/>
	<child link="Pelvis"/>
	<origin rpy="0 0 0" xyz="0 0 -0.079"/>
	<axis xyz="0 1.0 0"/>
	<limit effort="19.787" lower="-1.03847" upper="1.03847" velocity="2.93276"/>
</joint>
```

* KneePitch

Topic: /pepper_dcm/KneePitch_position_controller/command
```xml
<joint name="KneePitch" type="revolute">
	<parent link="Pelvis"/>
	<child link="Tibia"/>
	<origin rpy="0 0 0" xyz="0 0 -0.268"/>
	<axis xyz="0 1.0 0"/>
	<limit effort="27.702" lower="-0.514872" upper="0.514872" velocity="2.93276"/>
</joint>
```

