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

    shm.cmd.command = Command.START.value
    time.sleep(1)

    shm.cmd.command = Command.STAND.value
    time.sleep(5)

    shm.cmd.command = Command.WALK.value
    time.sleep(3)

    shm.cmd.reference_base_velocity = [0.1, 0.0]  
    time.sleep(3)
    print(Fore.LIGHTBLUE_EX,time.strftime("%H:%M:%S", time.gmtime(shm.local_time)),"[PY_MAIN] FSM:\t",CanineFSM(shm.fsm_state),Style.RESET_ALL)
    print(Fore.LIGHTBLUE_EX,time.strftime("%H:%M:%S", time.gmtime(shm.local_time)),"[PY_MAIN] body vel:\t",shm.body_base_velocity,Style.RESET_ALL)
    print(Fore.LIGHTBLUE_EX,time.strftime("%H:%M:%S", time.gmtime(shm.local_time)),"[PY_MAIN] base quat:\t",shm.global_base_quaternion,Style.RESET_ALL)

    time.sleep(1)
    shm.cmd.reference_base_velocity = [-0.1, 0.0]  
    time.sleep(1)
    print(Fore.LIGHTBLUE_EX,time.strftime("%H:%M:%S", time.gmtime(shm.local_time)),"[PY_MAIN] body vel:\t",shm.body_base_velocity,Style.RESET_ALL)

    time.sleep(1)
    shm.cmd.reference_base_velocity = [0.0, 0.1]  
    time.sleep(1)
    print(Fore.LIGHTBLUE_EX,time.strftime("%H:%M:%S", time.gmtime(shm.local_time)),"[PY_MAIN] body vel:\t",shm.body_base_velocity,Style.RESET_ALL)

    time.sleep(1)
    shm.cmd.reference_base_velocity = [0.0, 0.0]  
    time.sleep(1)
    print(Fore.LIGHTBLUE_EX,time.strftime("%H:%M:%S", time.gmtime(shm.local_time)),"[PY_MAIN] body vel:\t",shm.body_base_velocity,Style.RESET_ALL)

    shm.cmd.command = Command.READY.value
    time.sleep(4)

    shm.cmd.command = Command.READY.value
    time.sleep(5)

thread_ros_communication = threading.Thread(target=rosCommunicationThread)
thread_ros_communication.start()
thread_user = threading.Thread(target=userCommandThread)
thread_user.start()