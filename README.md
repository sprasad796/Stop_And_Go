# Stop-and-Go Simulator
## Comamnds to run under directory car_simulator
python stop_and_go_main.py
  It should generate the following files. Images will be generated under IP_IMAGES directory.
  a) stop_simno_ref_expno.jpg   : is the original 256 X 256 image
  b) stop_simno_traffic_expno.jpg : is the grey image of all AVs of size 256 X 256
  c) stop_simno_LANES_expno.jpg : is the only path and stop sign image of size 256 X 256
  d) test.json                  : Description about AV and SDV
  ### To capture command output
  python stop_and_go_main.py > /tmp/file 2>&1

## Run the command in debugging mode:
  Update the stop_and_go_global.py
  Set following True
  DEBUG = True
  It should print lots of logging statements.
  To change the parameters :
  There are 2 files to be changed. All the global variables applied to all the actors in the simulation in
  stop_and_go_global.py

  All the configuration related changes in the stop_and_go_config.yml
  This file contains the gaussian parameters of the AV

  Changing subimage size will require regeneration of dataset and it's properties. Below is the reference document.


https://github.com/user-attachments/assets/7e6042ab-e8a0-4d60-b21e-7ab1de60faec

