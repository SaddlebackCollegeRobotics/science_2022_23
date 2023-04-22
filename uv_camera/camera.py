import numpy as np
import os
import cv2
import datetime


class Camera:
    def __init__(self):
        self.filename = datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S") + ".avi"
        self.cap = cv2.VideoCapture(0)
        self.out = cv2.VideoWriter(self.filename, self.get_video_type(self.filename), 25, (640, 480))


    def get_video_type(self, filename):
        VIDEO_TYPE = {
            'avi': cv2.VideoWriter_fourcc(*'XVID'),
            'mp4': cv2.VideoWriter_fourcc(*'XVID'),
        }

        filename, ext = os.path.splitext(filename)
        if ext in VIDEO_TYPE:
            return VIDEO_TYPE[ext]

        return VIDEO_TYPE['avi']


    def record(self):
        ret, frame = self.cap.read()
        self.out.write(frame)

        
    def close(self):
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()
            


def main():
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