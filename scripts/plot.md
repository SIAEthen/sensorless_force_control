/bin/python /home/sia/girona_ws/src/sensorless_force_control/scripts/plot_pushing_scenario.py --csv src/sensorless_force_control/log/controller_data_20260220121336.csv --t-start 25 --t-end 30

/bin/python /home/sia/girona_ws/src/sensorless_force_control/scripts/plot_pushing_scenario.py --csv src/sensorless_force_control/log/controller_data_20260220154357_simulated_data.csv

/bin/python /home/sia/girona_ws/src/sensorless_force_control/scripts/plot_pushing_scenario.py --csv src/sensorless_force_control/log/controller_data_20260220160357_base_line.csv --t-start 900 --t-end 1100


/bin/python /home/sia/girona_ws/src/sensorless_force_control/scripts/plot_pushing_scenario.py --csv src/sensorless_force_control/log/controller_data_20260223104216.csv

python3 src/sensorless_force_control/scripts/plot_mu_bandit_training.py  src/sensorless_force_control/scripts/mu_bandit_20260324_164751.csv --save src/sensorless_force_control/scripts/mu_bandit_20260324_164751.png


# floating base without control -> end-effector position step forward (large velocity command with task priority control)
/bin/python /home/sia/girona_ws/src/sensorless_force_control/scripts/plot_pushing_scenario.py --csv src/sensorless_force_control/log/noised_thruster_controller_data_20260330122159_floatingbase.csv --t-start 0 --t-end 40

# floating base with control -> end-effector position controlled by joystick (small velocity command with task priority control, and also contact)
/bin/python /home/sia/girona_ws/src/sensorless_force_control/scripts/plot_pushing_scenario.py --csv src/sensorless_force_control/log/noised_thruster_controller_data_20260330123129_movingUnitilcontact.csv --t-start 90 --t-end 150