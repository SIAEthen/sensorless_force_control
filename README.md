# branch exp introduction
## files description
cfg -> ROS params dynamic configuration files
config/control -> robot kinematic model and calibrated parameters, i.e, gravity, wrench sensor
functionlib -> code of kinematics and dynamics calculation of a UVMS
src/girona_control.cpp -> source file of all the algorithms
src/controller_define.h -> header file that defines which algorithms in girona_controller.cpp to be used.
sc -> girona_controller_collect_data.cpp -> collect data for gravity and wrench sensor calibration.

## experiments description
Files in scripts whose name has a prefix of "girona_" are experimetns scripts.
### Observer test
girona_observer_test_push_hand_control_20N  -> pub a triangular force trajectory with a maximum value of 20 N while controlling the ee pushing the panel
girona_observer_test_push_hand_control_40N  -> pub a triangular force trajectory with a maximum value of 40 N while controlling the ee pushing the panel
girona_observer_test_free_floating_move_joints -> control the robot with 0 zelocity, moving the joints
girona_observer_test_free_floating_move_depth -> control the robot depth, see the effect of water pressure to wrench sensor force measurements along z axis.
### force control test
girona_force_control_sin_tra -> tracking a sinosoidal force trajectory along z axis with Patryk 2018 IROS
girona_force_control_sliding_rect -> tracking a constant force while ee sliding on the panel with a rectangular trajectory
girona_force_control_sliding_cirl -> tracking a constant force while ee sliding on the panel with a circle trajectory
### admittance control test
girona_admittance_control_sliding_cirl -> admittance control, ee tip sliding on the panel following a circle trajectory
girona_admittance_control_sliding_rect -> same experiment with a rectangular trajectory
### HQP test
girona_hqp_continuous_secnario1 -> deactivate and activate ee configuration task
girona_hqp_continuous_secnario2_circle -> ee tracking of a circle trajectory






# Girona1000 w/ Bravo Simulation #

This package contains a simulation setup for the Girona1000 w/ Bravo.

# Debug
## look up transforms
`rosrun tf tf_echo girona1000/base_link girona1000/bravo/payload_interface_link`
`rosrun tf tf_echo girona1000/bravo/base_link girona1000/bravo/cp_probe_tip_link`
rosrun tf tf_echo girona1000/bravo/link6 girona1000/bravo/cp_probe_tip_link
`rosrun tf tf_echo FrameA FrameB`
`rosrun tf tf_echo girona1000/bravo/cp_probe_tip_link girona1000/bravo/ft_link`
`rosrun tf tf_echo girona1000/base_link girona1000/bravo/ft_link`
`rosrun tf tf_echo girona1000/base_link girona1000/bravo/payload_interface_link`
`rosrun tf tf_echo girona1000/bravo/cp_probe_tip_link girona1000/bravo/payload_interface_link`
`rosrun tf tf_echo girona1000/bravo/cp_probe_tip_link girona1000/bravo/ft_link`
girona1000/bravo/payload_interface_link
return pos and rotation of FrameB expressed in FrameA
## view frames 
`rosrun tf view_frames`
this will generate a frames.pdf

## experiment
rosrun tf tf_echo girona500/base_link girona500/bravo_right/base_link



rosrun tf tf_echo girona500/bravo_right/link6 girona500/bravo_right/ft_sensor
At time 0.000
- Translation: [0.000, 0.000, 0.218]
- Rotation: in Quaternion [0.000, 0.000, -0.707, 0.707]
            in RPY (radian) [0.000, 0.000, -1.571]
            in RPY (degree) [0.000, 0.000, -90.000]
At time 0.000
- Translation: [0.000, 0.000, 0.218]
- Rotation: in Quaternion [0.000, 0.000, -0.707, 0.707]
            in RPY (radian) [0.000, 0.000, -1.571]
            in RPY (degree) [0.000, 0.000, -90.000]
^Csia@sia-Latitude-5410:~$ 
