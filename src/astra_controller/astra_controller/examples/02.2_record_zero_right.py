from astra_controller.arm_controller import ArmController
import time

def main():
	arm_controller_right = ArmController("/dev/tty_puppet_right", do_init=True)
	try:
		time.sleep(10)

		arm_controller_right.set_torque(0)
		time.sleep(10)
	finally:
		arm_controller_right.stop()


if __name__ == "__main__":
	main()