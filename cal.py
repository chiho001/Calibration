import cv2
import numpy as np
from tkinter import *
from tkinter import filedialog
from PIL import Image, ImageTk
from scipy.spatial.transform import Rotation as R  # For converting roll, pitch, yaw to rotation matrix

class FisheyeUndistortApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Fisheye Undistortion with Intrinsic and Extrinsic Parameters GUI")

        # Default intrinsic parameters
        self.default_fx = 300
        self.default_fy = 300
        self.default_cx = 320
        self.default_cy = 240

        # Current intrinsic parameters
        self.fx = self.default_fx
        self.fy = self.default_fy
        self.cx = self.default_cx
        self.cy = self.default_cy

        # Default extrinsic parameters
        self.default_roll = 0
        self.default_pitch = 0
        self.default_yaw = 0
        self.default_tx = 0
        self.default_ty = 0
        self.default_tz = 0

        # Current extrinsic parameters
        self.roll = self.default_roll
        self.pitch = self.default_pitch
        self.yaw = self.default_yaw
        self.create_widgets()
        self.load_default_image()

    def create_widgets(self):
        # Canvas for displaying images
        self.canvas_original = Label(self.root)
        self.canvas_original.grid(row=0, column=0, columnspan=4)
        
        self.canvas_undistorted = Label(self.root)
        self.canvas_undistorted.grid(row=0, column=4, columnspan=4)

        # Sliders for intrinsic parameters
        self.fx_scale = Scale(self.root, from_=100, to=1000, orient=HORIZONTAL, label="Fx", command=self.update_fx)
        self.fx_scale.set(self.fx)
        self.fx_scale.grid(row=1, column=0)

        self.fy_scale = Scale(self.root, from_=100, to=1000, orient=HORIZONTAL, label="Fy", command=self.update_fy)
        self.fy_scale.set(self.fy)
        self.fy_scale.grid(row=1, column=1)

        self.cx_scale = Scale(self.root, from_=0, to=640, orient=HORIZONTAL, label="Cx", command=self.update_cx)
        self.cx_scale.set(self.cx)
        self.cx_scale.grid(row=1, column=2)

        self.cy_scale = Scale(self.root, from_=0, to=480, orient=HORIZONTAL, label="Cy", command=self.update_cy)
        self.cy_scale.set(self.cy)
        self.cy_scale.grid(row=1, column=3)

        # Sliders for extrinsic parameters
        self.roll_scale = Scale(self.root, from_=-180, to=180, orient=HORIZONTAL, label="Roll (degrees)", command=self.update_roll)
        self.roll_scale.set(self.roll)
        self.roll_scale.grid(row=2, column=0)

        self.pitch_scale = Scale(self.root, from_=-180, to=180, orient=HORIZONTAL, label="Pitch (degrees)", command=self.update_pitch)
        self.pitch_scale.set(self.pitch)
        self.pitch_scale.grid(row=2, column=1)

        self.yaw_scale = Scale(self.root, from_=-180, to=180, orient=HORIZONTAL, label="Yaw (degrees)", command=self.update_yaw)
        self.yaw_scale.set(self.yaw)
        self.yaw_scale.grid(row=2, column=2)

        # Load image button
        Button(self.root, text="Load Image", command=self.load_image).grid(row=4, column=0, columnspan=2)

        # Default button to revert to default values
        Button(self.root, text="Default", command=self.set_defaults).grid(row=4, column=2, columnspan=2)

    def load_default_image(self):
        # Load "Fisheye.jpg" as the default image
        self.image_path = "./calibration_images/Fisheye.jpg"  # Ensure this file exists in the working directory
        self.load_image()
        self.display_original_undistortion()

    def load_image(self):
        if not hasattr(self, 'image_path') or not self.image_path:
            self.image_path = filedialog.askopenfilename()

        if self.image_path:
            self.img = cv2.imread(self.image_path)
            self.img = cv2.resize(self.img, (640, 480))
            self.show_image(self.img, self.canvas_original)

    def show_image(self, image, canvas):
        # Convert OpenCV image to ImageTk format for displaying
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(image_rgb)
        imgtk = ImageTk.PhotoImage(image=im)
        canvas.imgtk = imgtk
        canvas.configure(image=imgtk)

    def update_fx(self, value):
        self.fx = int(value)
        self.undistort_image()

    def update_fy(self, value):
        self.fy = int(value)
        self.undistort_image()

    def update_cx(self, value):
        self.cx = int(value)
        self.undistort_image()

    def update_cy(self, value):
        self.cy = int(value)
        self.undistort_image()

    def update_roll(self, value):
        self.roll = float(value)
        self.undistort_image()

    def update_pitch(self, value):
        self.pitch = float(value)
        self.undistort_image()

    def update_yaw(self, value):
        self.yaw = float(value)
        self.undistort_image()

    def set_defaults(self):
        # Revert to default intrinsic and extrinsic parameters
        self.fx = self.default_fx
        self.fy = self.default_fy
        self.cx = self.default_cx
        self.cy = self.default_cy
        self.roll = self.default_roll
        self.pitch = self.default_pitch
        self.yaw = self.default_yaw


        # Update sliders
        self.fx_scale.set(self.default_fx)
        self.fy_scale.set(self.default_fy)
        self.cx_scale.set(self.default_cx)
        self.cy_scale.set(self.default_cy)
        self.roll_scale.set(self.default_roll)
        self.pitch_scale.set(self.default_pitch)
        self.yaw_scale.set(self.default_yaw)

        # Update the image
        self.undistort_image()

    def undistort_image(self):
        h, w = self.img.shape[:2]

        # Create new camera matrix using updated intrinsic parameters
        K = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0, 0, 1]], dtype=np.float64)

        # Assume a fixed distortion coefficient for demonstration
        D = np.array([-0.2, 0.1, 0, 0], dtype=np.float64)

        # Compute the rotation matrix from roll, pitch, yaw (ensure correct axis order)
        rotation = R.from_euler('xyz', [self.pitch, self.yaw, self.roll], degrees=True)
        R_mat = rotation.as_matrix()

        # Use rotation matrix in the rectification step
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, R_mat, K, (w, h), cv2.CV_16SC2)
        undistorted_img = cv2.remap(self.img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        # Display the undistorted image
        self.show_image(undistorted_img, self.canvas_undistorted)

    def display_original_undistortion(self):
        # Undistort image using default intrinsic parameters
        h, w = self.img.shape[:2]

        K_default = np.array([[self.default_fx, 0, self.default_cx],
                              [0, self.default_fy, self.default_cy],
                              [0, 0, 1]], dtype=np.float64)

        D_default = np.array([-0.2, 0.1, 0, 0], dtype=np.float64)

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K_default, D_default, np.eye(3), K_default, (w, h), cv2.CV_16SC2)
        undistorted_img_default = cv2.remap(self.img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        # Display the undistorted image with original intrinsic parameters
        self.show_image(undistorted_img_default, self.canvas_original)

if __name__ == "__main__":
    root = Tk()
    app = FisheyeUndistortApp(root)
    root.mainloop()
