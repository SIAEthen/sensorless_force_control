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
