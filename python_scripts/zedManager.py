import os
import pyzed.sl as sl



class zedManager:
    def __init__(self,SharedMemoryManager):
        self.shm = SharedMemoryManager
        self.camera_fps = 30
        self.zed = sl.Camera()
        self.image_left = sl.Mat()
        self.image_right = sl.Mat()
        self.open_camera()

    def __del__(self):
        self.close_camera()
        self.zed.close()
        print("[ZED] Closed ZED Camera.")

    def open_camera(self):
        # Open the camera
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.AUTO  # Use HD720 opr HD1200 video mode, depending on camera type.
        init_params.camera_fps = self.camera_fps                 # Set camera fps
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("[ZED] Camera Open : "+repr(err)+". Exit program.")
            exit()
        else:
            print("[ZED] Initialized ZED Camera.")

    def close_camera(self):
        self.zed.close()

    def get_image(self):
        runtime_parameters = sl.RuntimeParameters()
        if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
            self.zed.retrieve_image(self.image_right, sl.VIEW.LEFT)
            self.time_stamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)

    def save_image(self, save_path):
        filename = os.path.join(save_path, f"zed_image_{self.time_stamp:05d}_left.png")
        self.image_left.write(filename)
        filename = os.path.join(save_path, f"zed_image_{self.time_stamp:05d}_right.png")
        self.image_right.write(filename)
        print("Saved images")