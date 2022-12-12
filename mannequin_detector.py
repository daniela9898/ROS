# "In this script, we implement the use of computer vision 
# for recognizing our mannequins. Specifically, the code is able to 
# identify the mannequin only if it has a QR code in front of it. 
# We have modified certain parameters to enable our turtlebot3 to 
# effectively recognize the mannequins under varying illumination conditions."

import sys
import time
import tkinter
from collections import deque
from typing import Union, Tuple, Any, Deque

import PIL.Image, PIL.ImageTk
import cv2
import matplotlib
import numpy as np
from cv2 import KeyPoint
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

import rospy
import rospy as ros
from rospy import Subscriber, Publisher

# define hex codes for some colors we use in our gui
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import CompressedImage, LaserScan
from pyzbar.pyzbar import decode

WHITE = "#FFFFFF"
BLACK = "#000000"
BLUE = "#1E64C8"
SKY = "#8BBEE8"


def bgr(color: str) -> Tuple:
    """
    Convert color hex to bgr tuple
    :param color:
    :return:
    """
    return tuple(int(channel * 255) for channel in matplotlib.colors.to_rgb(color))[::-1]

class MannequinDetector:
    """
    This class listens to the LiDAR /scan and raspicam /raspicam_node/image/compressed topics, detects
    green objects, and publishes their estimated location to a new ROS topic /mannequins

    In addition, this class opens a simple GUI with which you can manually calibrate the camera and LiDAR
    alignments and visualize the LiDAR and raspicam readings.
    """

    LIDAR_MAX_RANGE = 3.5  # meter
    DEFAULT_CAMERA_OFFSET_ANGLE: int = 1  # degrees
    DEFAULT_CAMERA_FIELD_OF_VIEW: int = 65  # degrees
    CAMERA_LIDAR_MAPPING_FUZZYNESS: int = 5  # pixels (to get a bit more reliable distance readings)
    PUBLISH_RATE: int = 1  # Hz
    QUEUE_SIZE: int = 2
    HUE_RANGE: Tuple[int, int] = (30, 90)  # Hue range of the color green in the hsv spectrum
    VAL_MINIMUM: int = 15  # Minimum value of the color green in the hsv spectrum
    SAT_MINUMUM: int = 15  # Minimum saturation of the color green in the hsv spectrum
    LDS_BUFFER_LENGTH: float = 3.  # Seconds

    # !!! For some reason, the timestamps of the lidar and camera fromes are offset. We need to compensate for this.
    LDS_TIMESTAMP_OFFSET: float = 2.

    def __init__(self, name: str = "mannequin_detector") -> None:
        self.node_name: str = name

        # Declare ROS subscriber names
        self.lds_sub_name: str = "/scan"  # LiDAR Distance Sensor
        self.cam_sub_name: str = "/raspicam_node/image/compressed"  # RASPBERRY Pi Camera Module

        # Declare ROS publisher names
        self.mannequin_pub_name: str = "/mannequins"

        # Placeholders for subscriber and publisher objects
        self.lds_sub: Union[None, Subscriber] = None
        self.cam_sub: Union[None, Subscriber] = None
        self.mannequin_pub: Union[None, Publisher] = None

        # Placeholders for sensor readings

        # A deque is a type of collection that provides O(1) time complexity for appending and popping. However, we can
        # not insert in or remove from the middle of the deque.
        self.lds_buffer: Deque[Tuple[float, np.ndarray]] = deque([])  # Buffer of lidar scans
        self.cam_image: Union[None, np.ndarray] = None
        self.cam_stamp: Union[None, float] = None  # Timestamp when image came in

        # Placeholders for figures to display in GUI
        self.__lidar_fig: Union[None, np.ndarray] = None
        self.__range_fig: Union[None, np.ndarray] = None
        self.__camera_fig: Union[None, np.ndarray] = None

        # OpenCV blob detector to detect mannequins as clusters of green in our camera frames
        self.blob_detector: cv2.SimpleBlobDetector = self.__init_blob_detector()

        # Initialize Tkinter - the back-end we use for our GUI
        self.gui = self.__init_tkinter()

        self.last_processed_timestamp: float = None

        # Set matplotlib interactive mode off
        plt.ioff()


    @property
    def lidar_fig(self) -> Union[None, Figure]:
        """
        This property contains a plot of the LiDAR scan ranges, used for overlaying on the camera frame for
        visualization in the GUI. This is a factory method pattern. Ideally, we would like to initialize this plot
        in the constructor of this class, however we need to wait until the first camera frame and LiDAR scan have come
        in. Therefore, we leave the value of lidar_fig None initially and every time we need lidar_fig, we check if
        we have the required data already. Once we have received the first frame and scan, we initialize the plot and
        save it in a private variable, __lidar_fig. Any time we use lidar_fig after that, this method simply points
        towards the object in __lidar_fig.
        :return:
        """
        if self.__lidar_fig is None:

            if self.cam_image is None or len(self.lds_buffer) == 0:
                # If the first camera frame or LiDAR scan has not come in yet, return None and wait until all required
                # data is available
                return None
            else:  # If all information is available, initialize the plot
                h, w, _ = self.cam_image.shape
                self.__lidar_fig = plt.figure(figsize=(w / 100 + 0.01, h / 100), dpi=100)
                ax = plt.Axes(self.__lidar_fig, [0., 0., 1., 1.])
                ax.set_axis_off()
                self.__lidar_fig.add_axes(ax)
        return self.__lidar_fig

    @property
    def range_fig(self) -> Union[None, Figure]:
        """
        Another factory method for the polar plot of the LiDAR scan
        :return:
        """
        if self.__range_fig is None:
            if len(self.lds_buffer) == 0:
                return None
            else:
                if self.cam_image is None or len(self.lds_buffer) == 0:
                    return None
                else:
                    h, w, _ = self.cam_image.shape
                    self.__range_fig = plt.figure(figsize=(w / 100 + 0.01, h / 100), dpi=100)
                    ax = self.__range_fig.add_subplot(111, polar=True)
                    ax.xaxis.set_ticks([])
                    ax.yaxis.set_ticklabels([])
        return self.__range_fig

    def __init_blob_detector(self) -> cv2.SimpleBlobDetector:
        """
        This method initializes the OpenCV blob detector to find mannequins by color in our
        camera frames
        :return:
        """
        params: cv2.SimpleBlobDetector_Params = cv2.SimpleBlobDetector_Params()

        # Setting some default parameters for the blob detector. These parameters should work for our
        # purposes, but you might want to play with them a bit yourself.
        params.filterByArea = True
        params.minArea = 100.
        params.maxArea = 1000000.
        params.filterByColor = True
        params.filterByInertia = False
        params.filterByConvexity = False
        params.filterByCircularity = False

        # Initialize and return the blob detector
        return cv2.SimpleBlobDetector_create(params)

    def __init_tkinter(self) -> dict:
        """
        This method initializes Tkinter and defines the canvasses in our GUI. This code looks a bit messy,
        but as far as I know, there is no better way to do this with Tkinter.

        Tip: You can minimize this method in the editor by pressing the - symbol next to the function definition
        :return:
        """

        # We need to keep a reference to all the Tkinter components we make, even the ones we will never
        # need to reference. If we don't keep these references, the Python interpreter forgets
        # about them and garbage collection will get rid of them.

        gui: dict = {}
        gui["root"]: tkinter.Tk = tkinter.Tk()
        gui["root"].title("LiDAR & Camera Alignment")
        gui["root"].geometry("820x616")  # Size of the GUI window
        gui["root"].protocol("WM_DELETE_WINDOW", sys.exit)
        gui["row_top"]: tkinter.Frame = tkinter.Frame()
        gui["row_top"].pack(side=tkinter.TOP, expand=True)  # Add the row to root
        gui["row_bottom"]: tkinter.Frame = tkinter.Frame()
        gui["row_bottom"].pack(side=tkinter.BOTTOM, expand=True)  # Add the row to root

        # A canvas that will display the color filter over our camera frames
        gui["blob_canvas"]: tkinter.Canvas = tkinter.Canvas(gui["row_top"], height=308, width=410)
        gui["blob_canvas"].pack(side=tkinter.LEFT, anchor=tkinter.NW)
        gui["blob_photo"]: Union[None,] = None  # Placeholder

        # A canvas that will display the LiDAR scan overlayed on the camera frame
        gui["range_canvas"]: tkinter.Canvas = tkinter.Canvas(gui["row_top"], height=308, width=410)
        gui["range_canvas"].pack(side=tkinter.RIGHT, anchor=tkinter.NE)
        gui["range_photo"]: Union[None,] = None  # Placeholder

        # A frame that contains some interactive components to manually configure camera parameters
        gui["configuration_frame"]: tkinter.Frame = tkinter.Frame(gui["row_bottom"], bg=WHITE, height=308, width=410)
        gui["configuration_frame"].pack(side=tkinter.LEFT)
        # Make sure tkinter does not resize this frame if there is empty space
        gui["configuration_frame"].pack_propagate(False)

        # Description for the camera offset slider bar
        gui["configuration_label_camera_offset"]: tkinter.Label = tkinter.Label(
            gui["configuration_frame"], bg=WHITE, text="Camera Offset Angle (degrees)", font="Arial 12 bold")
        gui["configuration_label_camera_offset"].pack(expand=True)

        # Camera offset slider bar frame
        gui["configuration_frame_camera_offset"]: tkinter.Frame = tkinter.Frame(gui["configuration_frame"], bg=WHITE)
        gui["configuration_frame_camera_offset"].pack(expand=True)

        # Add the slider bar
        gui["configuration_slider_camera_offset"]: tkinter.Scale = tkinter.Scale(
            gui["configuration_frame_camera_offset"], length=380, bg=WHITE, from_=-180, to=180,
            orient=tkinter.HORIZONTAL)
        # Add a button to decrease the slider by one
        gui["configuration_button_minus_camera_offset"]: tkinter.Button = tkinter.Button(
            gui["configuration_frame_camera_offset"], text="-", relief=tkinter.FLAT,
            command=lambda: gui["configuration_slider_camera_offset"].set(
                min(180, gui["configuration_slider_camera_offset"].get() - 1)),
            repeatdelay=300, repeatinterval=100)
        gui["configuration_button_minus_camera_offset"].pack(side=tkinter.LEFT, expand=True)
        # Add a button to increase the slider by one
        gui["configuration_button_plus_camera_offset"]: tkinter.Button = tkinter.Button(
            gui["configuration_frame_camera_offset"], text="+", relief=tkinter.FLAT,
            command=lambda: gui["configuration_slider_camera_offset"].set(
                max(-180, gui["configuration_slider_camera_offset"].get() + 1)),
            repeatdelay=300, repeatinterval=100)
        gui["configuration_button_plus_camera_offset"].pack(side=tkinter.RIGHT, expand=True)
        # Set the initial value for the camera offset
        gui["configuration_slider_camera_offset"].set(self.DEFAULT_CAMERA_OFFSET_ANGLE)
        gui["configuration_slider_camera_offset"].pack(side=tkinter.LEFT, expand=True)

        # Do the same thing again for the other slider, governing camera Field Of View (FOV)
        gui["configuration_label_fov"]: tkinter.Label = tkinter.Label(
            gui["configuration_frame"], bg=WHITE, text="Horizontal Field Of View", font="Arial 12 bold")
        gui["configuration_label_fov"].pack(expand=True)

        # FOV slider bar frame
        gui["configuration_frame_fov"]: tkinter.Frame = tkinter.Frame(gui["configuration_frame"], bg=WHITE)
        gui["configuration_frame_fov"].pack(expand=True)

        # Add the slider bar
        gui["configuration_slider_fov"]: tkinter.Scale = tkinter.Scale(
            gui["configuration_frame_fov"], length=380, bg=WHITE, from_=1, to=360, orient=tkinter.HORIZONTAL)
        # Add a button to decrease the slider by one
        gui["configuration_button_minus_fov"]: tkinter.Button = tkinter.Button(
            gui["configuration_frame_fov"], text="-", relief=tkinter.FLAT,
            command=lambda: gui["configuration_slider_fov"].set(
                max(1, gui["configuration_slider_fov"].get() - 1)),
            repeatdelay=300, repeatinterval=100)
        gui["configuration_button_minus_fov"].pack(side=tkinter.LEFT, expand=True)
        # Add a button to increase the slider by one
        gui["configuration_button_plus_fov"]: tkinter.Button = tkinter.Button(
            gui["configuration_frame_fov"], text="+", relief=tkinter.FLAT,
            command=lambda: gui["configuration_slider_fov"].set(
                min(360, gui["configuration_slider_fov"].get() + 1)),
            repeatdelay=300, repeatinterval=100)
        gui["configuration_button_plus_fov"].pack(side=tkinter.RIGHT, expand=True)
        gui["configuration_slider_fov"].set(self.DEFAULT_CAMERA_FIELD_OF_VIEW)
        gui["configuration_slider_fov"].pack(side=tkinter.LEFT, expand=True)

        # A canvas that will display a plot of the LiDAR scan
        gui["lidar_canvas"]: tkinter.Canvas = tkinter.Canvas(gui["row_bottom"], height=308, width=410)
        gui["lidar_canvas"].pack(side=tkinter.RIGHT, anchor=tkinter.NE)
        gui["lidar_photo"]: Union[None,] = None

        return gui

    def __lds_ros_sub(self, msg: LaserScan) -> None:
        """
        Callback function that is executed every time a new LiDAR scan comes in
        :param msg: LiDAR ros message
        :return:
        """
        # Store msg data in placeholder
        ranges = np.array(list(msg.ranges))[::-1]

        # Set all distances outside range to max range.
        ranges[(ranges == 0.) | np.isinf(ranges) | (ranges > self.LIDAR_MAX_RANGE)] = self.LIDAR_MAX_RANGE

        # Store timestamp and values in lds buffer
        timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

        self.lds_buffer.append((timestamp, ranges))
        while timestamp - self.LDS_BUFFER_LENGTH > self.lds_buffer[0][0]:
            self.lds_buffer.popleft()

    def __cam_ros_sub(self, msg: CompressedImage) -> None:
        """
        Callback function that is executed every time a new camera frame comes in
        :param msg: Camera frame
        :return:
        """
        # Convert msg data (bitstring) to OpenCV image (np.ndarray) and store in placeholder
        self.cam_image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)[::-1, ::-1]
        self.cam_stamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

    def start_ros(self) -> None:
        """
        Initialize ros and set all Subscribers and Publishers
        :return:
        """
        # init ros node
        ros.init_node(self.node_name, log_level=ros.INFO)

        # Create the Subscribers
        self.lds_sub: Subscriber = ros.Subscriber(self.lds_sub_name, LaserScan, callback=self.__lds_ros_sub,
                                                  queue_size=self.QUEUE_SIZE)
        self.cam_sub: Subscriber = ros.Subscriber(self.cam_sub_name, CompressedImage, callback=self.__cam_ros_sub,
                                                  queue_size=self.QUEUE_SIZE)

        # Create the Publishers
        self.mannequin_pub: Publisher = ros.Publisher(self.mannequin_pub_name, numpy_msg(Floats),
                                                      queue_size=self.QUEUE_SIZE)

    def run(self):
        """
        The main loop of this function runs within the Tkinter() framework
        :return:
        """
        # We call update() once to let Tkinter know we will be using this method in the main loop (specifically the
        # recursive call to update() within the update() method lets Tkinter know that this method needs to be run
        # repeatdly)
        self.update()

        # After that, we start the main loop, and Tkinter will take care of the rest
        self.gui["root"].mainloop()

    def update(self) -> None:
        """
        Inner loop of the Tkinter gui
        :return:
        """
        if rospy.is_shutdown():
            sys.exit(0)

        if self.cam_image is None or len(self.lds_buffer) == 0:  # if no camera frames have come in yet
            pass
        elif self.cam_stamp == self.last_processed_timestamp:
            pass
        else:
            # get camera frame and Lidar scan
            img, scan = self.__temporally_aligned_frame_and_scan()
            self.last_processed_timestamp = self.cam_stamp

            if img is not None and scan is not None:
                # Process camera frame with the blob detector
                mannequin_keypoints, blob_img = self.__apply_blob_detection(img)

                # For each column in the camera frame, find the accompanying range from the lidar scan
                aligned_ranges, range_img = self.__align_lidar_scan_with_camera_frame(img, scan)

                # Make the polar plot of the LiDAR scan for the GUI
                lidar_img = self.__make_lidar_polar_plot(scan)

                self.__display_plots_in_gui(blob_img, range_img, lidar_img)

                self.__publish_measured_mannequin_locations(mannequin_keypoints, aligned_ranges)

        # Normally recursion is a bad idea in Python, but this is what Tkinter wants
        self.gui["root"].after(1000 // self.PUBLISH_RATE, self.update)

    def __temporally_aligned_frame_and_scan(self) -> Tuple[Union[None, np.ndarray], ...]:
        """
        Compare the timestamp of the last frame with timestamps of lidar scans in buffer. Return lidar scan at the time
        the frame was captured
        :return:
        """
        try:
            img, timestamp = self.cam_image, self.cam_stamp

            scan = np.stack([s[1] for s in self.lds_buffer if abs(timestamp - s[0]) < 0.5])
            scan = np.min(scan, axis=0)
            # scan = min(self.lds_buffer, key=lambda s_: abs(timestamp - s_[0] + self.LDS_TIMESTAMP_OFFSET))

            # if len(self.lds_buffer) > 10:
            #     print()
            return img, scan
        except ValueError:
            return None, None

    def __apply_blob_detection(self, img: np.ndarray) -> Tuple[Tuple[cv2.KeyPoint, ...], np.ndarray]:
        """
        Do some preprocessing on camera frame and use OpenCV blob detection to find green blobs in the image
        :param img:
        :return:
        """
        # Convert image from bgr (blue-green-red) to hsv (hue-saturation-value). In the hsv spectrum, we will be able
        # to accurately and reliably define the bounds of the color green. Values in the hsv spectrum are a better
        # description of the way light actually works, while values in the rgb spectrum - which you might be more
        # familiar with - more closely resemble the "hack" that our eyes, cameras, and screens use. If you are
        # interested in color theory, read https://en.wikipedia.org/wiki/HSL_and_HSV, or watch
        # https://youtu.be/qFXYPHE7fIo if you're a nerd like me.


        # start qr detector
        (rows, cols, channels) = img.shape

        img1: np.ndarray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh = 40
        img_bw = cv2.threshold(img1, thresh, 255, cv2.THRESH_BINARY)[1]

        qr_result = decode(img_bw)
        qr_code = 0
        if not qr_result:
            print("not qr code")
        else:
            qr_data = qr_result[0].data
            qr_data = qr_data.decode("utf-8")
            print(qr_data)

            f="FRONT"
            if f in qr_data:
                qr_code = 1

        # end qr detector


        # start line detector
        gray: np.ndarray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 75, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70, maxLineGap=250)
        # end line detector
        img2: np.ndarray = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Filter for green over hue channel
        mask_hue = (self.HUE_RANGE[0] < img2[..., 0]) & (img2[..., 0] < self.HUE_RANGE[1])
        mask_saturation = img2[..., 1] > self.SAT_MINUMUM  # Ditch pixels that are too pastel (grey-ish)
        mask_value = img2[..., 2] > self.VAL_MINIMUM  # Ditch pixels that are too dark
        # Combine masks
        if qr_code :
            print("here")
            mask = mask_hue * mask_saturation * mask_value
        else:
            mask = np.zeros([rows, cols])
        # Make the mask blurry to help the blob detector
        mask = cv2.blur(mask.astype(np.uint8) * 255, (10, 10))

        # Add some black padding around the mask; this helps the blob detector detect mannequins at the edge of the
        # frame
        padding = 10  # pixels
        padded_mask = np.zeros((mask.shape[0] + 2 * padding, mask.shape[1] + 2 * padding), dtype=np.uint8)
        padded_mask[padding:-padding, padding:-padding] = mask

        # Apply the blob detector on the preprocessed image
        keypoints = self.blob_detector.detect(255 - padded_mask)
        for kp in keypoints:  # subtract the padding from the points returned by the blob detector
            kp.pt = (kp.pt[0] - padding, kp.pt[1] - padding)

        # Draw dots on the mask to visualize where the blob detector found mannequins
        img_visualize = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        for kp in keypoints:
            cv2.circle(img_visualize,
                       center=(int(kp.pt[0]), int(kp.pt[1])),
                       radius=5,
                       color=bgr(BLUE),
                       thickness=cv2.FILLED)

        return keypoints, img_visualize

    def __align_lidar_scan_with_camera_frame(self, img: np.ndarray, scan: np.ndarray,
                                             ) -> Tuple[Union[None, np.ndarray], Union[None, np.ndarray]]:
        """
        The lidar scan gives us 360 ranges around the turtlebot. The raspicam, in the turtlebot's default configuration,
        gives us a field of view of roughly 63 degrees in front of the robot, with a few degrees offset caused by
        inaccuracies in the hardware. To make accurate estimations of the locations of the detected mannequins, this
        method aligns the LiDAR ranges with the camera frame. In addition, it returns an image representing the
        last camera frame overlayed with the aligned LiDAR ranges for visualization in the GUI.
        :param img:
        :return:
        """

        # Get the camera configuration parameters from the GUI
        camera_offset: int = self.gui["configuration_slider_camera_offset"].get()
        camera_fov: int = self.gui["configuration_slider_fov"].get()

        # Align the horizontal pixels of the camera frame with the values in the lidar scan array
        camera_angle_left: int = camera_offset - (camera_fov - 1) // 2
        camera_angle_right: int = camera_offset + (camera_fov - 1) // 2
        scan_indices: np.ndarray = np.arange(camera_angle_left, camera_angle_right + 1)
        ranges_within_camera_view: np.ndarray = scan[scan_indices]

        # For each horizontal pixel in the camera frame, find the accompanying value from the lidar scan
        pixel2scan_indices: np.ndarray = np.round(
            np.linspace(0, ranges_within_camera_view.size - 1, num=img.shape[1])).astype(int)
        pixel_ranges: np.ndarray = ranges_within_camera_view[pixel2scan_indices]
        # To make the alignment a bit more robust, set the range of each index in the scan to the minimum of the
        # surrounding few measurements (e.g. [3.5 3.5 3.5 1.0 3.5 3.5] -> [3.5 3.5 1.0 1.0 1.0 3.5])
        for _ in range(self.CAMERA_LIDAR_MAPPING_FUZZYNESS):
            pixel_ranges[:-1] = np.minimum(pixel_ranges[:-1], pixel_ranges[1:])
            pixel_ranges[1:] = np.minimum(pixel_ranges[1:], pixel_ranges[:-1])

        # Plot the aligned ranges for visualization in the GUI
        self.lidar_fig.axes[0].lines = []  # remove previous plot
        self.lidar_fig.axes[0].set_ylim(0., self.LIDAR_MAX_RANGE + .1)
        self.lidar_fig.axes[0].margins(x=0)
        self.lidar_fig.axes[0].plot(list(range(pixel_ranges.size)), pixel_ranges, color=bgr(BLACK))

        # Overlay plot on camera frame for visualization
        plot = self.__overlay_plot_on_frame(img, self.lidar_fig)

        return pixel_ranges, plot

    def __overlay_plot_on_frame(self, img: np.ndarray, fig: Figure) -> np.ndarray:
        """
        Convert pyplot figure to OpenCV image
        :param fig:
        :return:
        """
        fig.canvas.draw()
        assert fig.canvas.get_width_height()[::-1] + (3,) == img.shape

        mask_line = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        mask_line = mask_line.reshape(img.shape)[:, :, 0] < 255

        img_overlay = img.copy()
        img_overlay[mask_line, 0], img_overlay[mask_line, 1], img_overlay[mask_line, 2] = bgr(BLUE)

        return img_overlay

    def __make_lidar_polar_plot(self, scan: np.ndarray) -> Union[None, np.ndarray]:
        """
        Plot lidar scan data in a polar plot
        :return:
        """

        # A polar diagram plots angle (theta) against a value (LiDAR scan)
        theta = np.linspace(0. * np.pi, 2. * np.pi, num=scan.size, endpoint=False)

        # Make the polar plot
        ax = self.range_fig.axes[0]
        ax.lines, ax.collections = [], []  # clear the previous plot
        ax.plot(theta, scan[::-1], color=BLUE)

        # Indicate a cone in the plot representing the field of view of the raspicam
        camera_offset: int = self.gui["configuration_slider_camera_offset"].get()
        camera_fov: int = self.gui["configuration_slider_fov"].get()
        theta, _ = np.linspace(
            np.deg2rad(camera_offset - (camera_fov - 1) // 2),
            np.deg2rad(camera_offset + (camera_fov + 1) // 2),
            retstep=np.pi / 180)
        ax.fill_between(theta, [self.LIDAR_MAX_RANGE] * theta.size, color=SKY)

        # Convert plot to an OpenCV image
        self.range_fig.canvas.draw()
        plot = np.frombuffer(self.range_fig.canvas.tostring_rgb(), dtype=np.uint8)
        plot = plot.reshape(self.range_fig.canvas.get_width_height()[::-1] + (3,))[:, :, ::-1]

        return plot

    def __display_plots_in_gui(self, blob_img: np.ndarray, range_img: np.ndarray, polar_img: np.ndarray) -> None:
        """
        Display all our visualizations of the sensor data in the gui
        :param blob_img:
        :param range_img:
        :param polar_img:
        :return:
        """
        self.gui["blob_photo"] = PIL.ImageTk.PhotoImage(PIL.Image.fromarray(blob_img[..., ::-1]))
        self.gui["blob_canvas"].create_image(0, 0, image=self.gui["blob_photo"], anchor=tkinter.NW)

        self.gui["range_photo"] = PIL.ImageTk.PhotoImage(PIL.Image.fromarray(range_img[..., ::-1]))
        self.gui["range_canvas"].create_image(0, 0, image=self.gui["range_photo"], anchor=tkinter.NW)

        self.gui["lidar_photo"] = PIL.ImageTk.PhotoImage(PIL.Image.fromarray(polar_img[..., ::-1]))
        self.gui["lidar_canvas"].create_image(0, 0, image=self.gui["lidar_photo"], anchor=tkinter.NW)

    def __publish_measured_mannequin_locations(self, keypoints: Tuple[KeyPoint, ...], aligned_distances: np.ndarray
                                               ) -> None:
        """
        This method computes the angle and distance of all detected mannequins relative to the turtlebot3 frame
        :param keypoints:
        :param aligned_distances:
        :return:
        """
        mannequins = []

        camera_frame_width: int = self.cam_image.shape[1]
        camera_offset: int = self.gui["configuration_slider_camera_offset"].get()
        camera_fov: int = self.gui["configuration_slider_fov"].get()

        for kp in keypoints:
            # Compute the angle relative to the turtlebot
            kp_x_in_image: int = kp.pt[0]
            angle_to_camera_center_degrees = -(kp_x_in_image - camera_frame_width / 2) \
                                             / camera_frame_width * camera_fov
            angle_to_turtlebot3_degrees = angle_to_camera_center_degrees + camera_offset
            angle_to_turtlebot3_radians = np.deg2rad(angle_to_turtlebot3_degrees)

            # Get the distance measured by the LiDAR scan
            distance_to_turtlebot3 = aligned_distances[int(round(kp_x_in_image))]

            mannequins.extend([angle_to_turtlebot3_radians, distance_to_turtlebot3])

        # Publish an array to /mannequins containing angle-distance pairs corresponding to detected mannequins, where
        # even indices (Python starts counting at 0) contain measured angle, and the odd indices contain distance
        mannequins = np.array(mannequins, dtype=np.float32)
        self.mannequin_pub.publish(mannequins)


def main():
    mannequin_detector = MannequinDetector()
    mannequin_detector.start_ros()
    mannequin_detector.run()


if __name__ == '__main__':
    main()
