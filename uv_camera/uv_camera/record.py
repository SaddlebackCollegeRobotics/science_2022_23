import numpy as np
import os
import cv2
import datetime


class Camera:
    """
    A class used to represent a camera object for video recording.

    Attributes
    ----------
    filename : str
        The filename to be used for the video file.
    cap : cv2.VideoCapture object
        The object used to read frames from the camera.
    out : cv2.VideoWriter object
        The object used to write the frames to a video file.

    Methods
    -------
    get_video_type(filename: str) -> cv2.VideoWriter_fourcc
        Returns the fourcc codec used for the output video file.
    record() -> None
        Records a frame from the camera and writes it to the output video file.
    close() -> None
        Releases the resources used by the camera and output video file.
    """
    def __init__(self):
        """
        Initializes the Camera object with a filename, a cv2.VideoCapture object, and a cv2.VideoWriter object.
        """
        self.filename = datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S") + ".avi"
        self.cap = cv2.VideoCapture(0)
        self.out = cv2.VideoWriter(self.filename, self.get_video_type(self.filename), 25, (640, 480))


    def get_video_type(self, filename: str) -> cv2.VideoWriter_fourcc:
        """
        Returns the fourcc codec used for the output video file.

        Parameters
        ----------
        filename : str
            The name of the output video file.

        Returns
        -------
        cv2.VideoWriter_fourcc
            The fourcc codec used for the output video file.
        """
        VIDEO_TYPE = {
            'avi': cv2.VideoWriter_fourcc(*'XVID'),
            'mp4': cv2.VideoWriter_fourcc(*'XVID'),
        }

        filename, ext = os.path.splitext(filename)
        if ext in VIDEO_TYPE:
            return VIDEO_TYPE[ext]

        return VIDEO_TYPE['avi']


    def record(self) -> None:
        """
        Records a frame from the camera and writes it to the output video file.
        """
        ret, frame = self.cap.read()
        self.out.write(frame)

        
    def close(self) -> None:
        """
        Releases the resources used by the camera and output video file.
        """
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()
            


def main() -> None:
    """
    The main function that creates a Camera object and provides a user interface to start and stop recording.

    Returns
    -------
    None
    """
    camera = Camera()

    while True:
        print("(1) Record\n(2) Close\n")
        inp = input("Command: ")

        if inp == "1":
            while True:
                camera.record()
        
        if inp == "2":
            camera.close()
            break


if __name__ == "__main__":
    main()
