import rclpy
import threading
import time
from canineStruct import Command, CanineFSM
from rosCommunication import ROSCommunication
from sharedMemory import SharedMemoryManager
from colorama import Fore, Style

shm = SharedMemoryManager()

def rosCommunicationThread(args=None):
    rclpy.init(args=args)
    node = ROSCommunication(shm)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def userCommandThread(args=None):
    time.sleep(1)
    while shm.middleware_connected == False:
        print(Fore.RED + "[PY_MAIN] Failed to communicate with middleware." + Style.RESET_ALL)
        time.sleep(1)
    
    print(Fore.LIGHTBLUE_EX,time.strftime("%H:%M:%S", time.gmtime(shm.local_time)),"[PY_MAIN] CANINE FSM:" ,CanineFSM(shm.fsm_state),Style.RESET_ALL)
    shm.cmd.command = Command.DWA.value
    print(Fore.LIGHTBLUE_EX,time.strftime("%H:%M:%S", time.gmtime(shm.local_time)),"[PY_MAIN] CANINE CMD:" ,Command(shm.cmd.command),Style.RESET_ALL)
    time.sleep(3)

    shm.cmd.command = Command.NO_INPUT.value
    print(Fore.LIGHTBLUE_EX,time.strftime("%H:%M:%S", time.gmtime(shm.local_time)),"[PY_MAIN] CANINE CMD:" ,Command(shm.cmd.command),Style.RESET_ALL)
    time.sleep(3)

    shm.cmd.command = Command.DWA.value
    print(Fore.LIGHTBLUE_EX,time.strftime("%H:%M:%S", time.gmtime(shm.local_time)),"[PY_MAIN] CANINE CMD:" ,Command(shm.cmd.command),Style.RESET_ALL)
    time.sleep(3)

thread_ros_communication = threading.Thread(target=rosCommunicationThread)
thread_ros_communication.start()
thread_user = threading.Thread(target=userCommandThread)
thread_user.start()