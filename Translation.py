import cv2
import numpy as np
from tkinter import *
from tkinter import filedialog
from PIL import Image, ImageTk

class TranslationZoomApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Image Translation and Zoom (Tz) GUI")

        # Default translation parameters
        self.default_tx = 0
        self.default_ty = 0
        self.default_tz = 0

        # Current translation parameters
        self.tx = self.default_tx
        self.ty = self.default_ty
        self.tz = self.default_tz  # Controls zoom

        self.create_widgets()
        self.load_default_image()

    def create_widgets(self):
        # Canvas for displaying images
        self.canvas = Label(self.root)
        self.canvas.grid(row=0, column=0, columnspan=3)

        # Sliders for translation parameters
        self.tx_scale = Scale(self.root, from_=-100, to=100, orient=HORIZONTAL, label="Tx (Horizontal) [pixels]", command=self.update_tx)
        self.tx_scale.set(self.tx)
        self.tx_scale.grid(row=1, column=0)

        self.ty_scale = Scale(self.root, from_=-100, to=100, orient=HORIZONTAL, label="Ty (Vertical) [pixels]", command=self.update_ty)
        self.ty_scale.set(self.ty)
        self.ty_scale.grid(row=1, column=1)

        self.tz_scale = Scale(self.root, from_=-50, to=50, orient=HORIZONTAL, label="Tz (Zoom) [%]", command=self.update_tz)
        self.tz_scale.set(self.tz)
        self.tz_scale.grid(row=1, column=2)

        # Load image button
        Button(self.root, text="Load Image", command=self.load_image).grid(row=2, column=0, columnspan=2)

        # Default button to reset translations
        Button(self.root, text="Reset", command=self.reset_translation).grid(row=2, column=2)

    def load_default_image(self):
        # Load a default image (use your own image if needed)
        self.image_path = "./calibration_images/Fisheye.jpg"  # Ensure this file exists in the working directory
        self.load_image()

    def load_image(self):
        if not hasattr(self, 'image_path') or not self.image_path:
            self.image_path = filedialog.askopenfilename()

        if self.image_path:
            self.img = cv2.imread(self.image_path)
            self.img = cv2.resize(self.img, (640, 480))
            self.show_image(self.img)

    def show_image(self, image):
        # Convert OpenCV image to ImageTk format for displaying
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(image_rgb)
        imgtk = ImageTk.PhotoImage(image=im)
        self.canvas.imgtk = imgtk
        self.canvas.configure(image=imgtk)

    def update_tx(self, value):
        self.tx = int(value)
        self.apply_transform()

    def update_ty(self, value):
        self.ty = int(value)
        self.apply_transform()

    def update_tz(self, value):
        self.tz = int(value)
        self.apply_transform()

    def reset_translation(self):
        # Reset translation and zoom parameters
        self.tx = self.default_tx
        self.ty = self.default_ty
        self.tz = self.default_tz
        self.tx_scale.set(self.default_tx)
        self.ty_scale.set(self.default_ty)
        self.tz_scale.set(self.default_tz)
        self.apply_transform()

    def apply_transform(self):
        # Create a translation matrix
        translation_matrix = np.float32([[1, 0, self.tx], [0, 1, self.ty]])
        translated_img = cv2.warpAffine(self.img, translation_matrix, (self.img.shape[1], self.img.shape[0]))

        # Apply zoom based on tz (positive tz = zoom in, negative tz = zoom out)
        scale_factor = 1 + self.tz / 100.0  # Scale factor for zooming
        h, w = translated_img.shape[:2]
        center_x, center_y = w // 2, h // 2

        # Calculate new dimensions
        new_w, new_h = int(w * scale_factor), int(h * scale_factor)

        # Resize the image
        zoomed_img = cv2.resize(translated_img, (new_w, new_h))

        # Crop or pad to retain original size
        if scale_factor > 1:  # Crop
            start_x = (new_w - w) // 2
            start_y = (new_h - h) // 2
            zoomed_img = zoomed_img[start_y:start_y + h, start_x:start_x + w]
        else:  # Pad
            pad_x = (w - new_w) // 2
            pad_y = (h - new_h) // 2
            zoomed_img = cv2.copyMakeBorder(zoomed_img, pad_y, pad_y, pad_x, pad_x, cv2.BORDER_CONSTANT)

        # Display the transformed image
        self.show_image(zoomed_img)

if __name__ == "__main__":
    root = Tk()
    app = TranslationZoomApp(root)
    root.mainloop()
