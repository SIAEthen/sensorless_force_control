# Girona1000 w/ Bravo Simulation #

This package contains a simulation setup for the Girona1000 w/ Bravo.

# Debug
## look up transforms
`rosrun tf tf_echo girona1000/base_link girona1000/bravo/payload_interface_link`
`rosrun tf tf_echo FrameA FrameB`
return pos and rotation of FrameB expressed in FrameA
## view frames 
`rosrun tf view_frames`
this will generate a frames.pdf
