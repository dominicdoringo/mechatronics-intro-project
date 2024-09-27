from multiprocessing import Process
from shared_memory_wrapper import SharedMemoryWrapper
from kill_button_interface import Kill_Button_Interface
from example_process import Example_Process

from move_forward_process import move_forward_process
from ascend_descend_process import ascend_descend_process

def main():
    # Create shared memory
    shared_memory_object = SharedMemoryWrapper()

    # Create objects
    kill_button_listener = Kill_Button_Interface(running=shared_memory_object.running)
    example_object = Example_Process(shared_memory_object)
    
    forward_process_object = Process(target=move_forward_process, args=(shared_memory_object,))
    ascend_process_object = Process(target=ascend_descend_process, args=(shared_memory_object,))

    # Create processes
    kill_button_listener_process = Process(target=kill_button_listener.run_loop)
    example_process = Process(target=example_object.run_loop)

    # Start processes
    kill_button_listener_process.start()
    example_process.start()
    
    forward_process_object.start()
    ascend_process_object.start()

    # Wait for processes to finish
    kill_button_listener_process.join()
    example_process.join()
    forward_process_object.join()
    ascend_process_object.join()

    print("Program has finished")

if __name__ == '__main__':
    print("RUN FROM LAUNCH")
    main()