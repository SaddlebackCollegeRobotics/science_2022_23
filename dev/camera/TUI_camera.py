import numpy as np
import os
import cv2
import datetime


filename = datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S") + ".avi"


VIDEO_TYPE = {
    'avi': cv2.VideoWriter_fourcc(*'XVID'),
    #'mp4': cv2.VideoWriter_fourcc(*'H264'),
    'mp4': cv2.VideoWriter_fourcc(*'XVID'),
}


def get_video_type(filename):
    filename, ext = os.path.splitext(filename)
    if ext in VIDEO_TYPE:
        return  VIDEO_TYPE[ext]
    return VIDEO_TYPE['avi']



cap = cv2.VideoCapture(0)
out = cv2.VideoWriter(filename, get_video_type(filename), 25, (640, 480))


while True:
    ret, frame = cap.read()
    out.write(frame)

    print("Capturing frames ::", datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S"))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
out.release()
cv2.destroyAllWindows()