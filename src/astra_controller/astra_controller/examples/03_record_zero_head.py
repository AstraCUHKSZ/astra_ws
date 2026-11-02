
from astra_controller.head_controller import HeadController
import time


def main():
	head = HeadController("/dev/tty_head")
	head.set_torque(128)

	start = time.monotonic()
	try:
		while time.monotonic() - start < 10.0:
			pos, vel, eff, ts = head.get_pos()
			print(
				f"t={ts:.3f} pos(rad)={pos.tolist()} vel(rad/s)={vel.tolist()} eff(est)={eff.tolist()}"
			)
			time.sleep(0.2)
	finally:
		head.stop()


if __name__ == "__main__":
	main()