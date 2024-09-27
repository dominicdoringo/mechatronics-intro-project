import time
from shared_memory_wrapper import SharedMemoryWrapper
from read_can import CANReader
from kill_button_interface import KillButton
# from MotorWrapper import MotorWrapper

def move_forward_process(shared_memory):
    can_reader = CANReader()
    kill_button = KillButton()
    # motor_controller = MotorWrapper()  # Create an instance of MotorWrapper to control the motors


    interval = 5  # Timer interval for moving forward (seconds)
    # move_value = 10  # The value (speed or power) to move forward, you can adjust this
    
    while shared_memory.running.value:
        start = time.time()

        # Check kill button and CAN reader status
        if kill_button.is_pressed() or not can_reader.read():
            shared_memory.running.value = False
            print("Kill button pressed or CAN error. Stopping movement.")
            break

        # Move forward (replace with motor call)
        print("Moving forward.")
        # motor_controller.move_forward(move_value)  # This sends the forward command to the motors
        
        # Toggle ascend/descend in the shared memory for the other process
        shared_memory.ascend.value = not shared_memory.ascend.value

        # Simulate motor action time
        time.sleep(0.5)

        # Sleep until the next interval
        end = time.time()
        if interval - (end - start) > 0:
            time.sleep(interval - (end - start))

    print("Move forward process ended.")