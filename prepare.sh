python src/astra_controller/astra_controller/examples/01_lift_up.py
sleep 5
python src/astra_controller/astra_controller/examples/02.1_record_zero_left.py
sleep 5
python src/astra_controller/astra_controller/examples/02.2_record_zero_right.py
sleep 5
python src/astra_controller/astra_controller/examples/03_record_zero_head.py
sleep 5
python src/astra_controller/astra_controller/examples/04_check_arm_motors.py --device /dev/tty_puppet_left
sleep 5
python src/astra_controller/astra_controller/examples/04_check_arm_motors.py --device /dev/tty_puppet_right
sleep 5
