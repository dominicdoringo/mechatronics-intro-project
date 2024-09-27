import time
from shared_memory_wrapper import SharedMemoryWrapper
from read_can import CANReader
from kill_button_interface import KillButton
# from MotorWrapper import MotorWrapper


# alternates between ascending and descending based on the shared memory flag.
def ascend_descend_process(shared_memory):
    can_reader = CANReader()
    kill_button = KillButton()
    #  motor_controller = MotorWrapper()  # Create an instance of MotorWrapper to control the motors

    interval = 2.5  # Timer interval for ascend/descend (half of move forward process)
    #   move_value = 5  # The value (speed or power) to move up or down, you can adjust this

    while shared_memory.running.value:
        start = time.time()

        # Check kill button and CAN reader status
        if kill_button.is_pressed() or not can_reader.read():
            shared_memory.running.value = False
            print("Kill button pressed or CAN error. Stopping ascend/descend.")
            break

        if shared_memory.ascend.value:
            # Ascend (replace with motor call)
            print("Ascending.")
            # motor_controller.move_up(move_value)  # This sends the ascend command to the motors
        else:
            # Descend (replace with motor call)
            print("Descending.")
            # motor_controller.move_down(move_value)  # This sends the descend command to the motors

        # Simulate motor action time
        time.sleep(0.5)

        # Sleep until the next interval
        end = time.time()
        if interval - (end - start) > 0:
            time.sleep(interval - (end - start))

    print("Ascend/Descend process ended.")