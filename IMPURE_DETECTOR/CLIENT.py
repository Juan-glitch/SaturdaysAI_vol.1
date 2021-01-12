################################################################################
### client.py
################################################################################


#!/usr/bin/env python
# -*- coding:utf-8 -*-


"""
Dummy client, interacts with the server sending and receiving
compressed numpy arrays.

Run:
python client.py
# Camera /dev
 ls -ltrh /dev/video*
"""


from __future__ import print_function
import io
import numpy as np
import zlib
import requests
import jsonpickle
import cv2
import imageio
import json
import time
import serial

# SERIAL
ser=serial.Serial('/dev/ttyACM0',115200, bytesize = 8, stopbits = 1,
                  timeout = 0, parity='N')
print(ser.name)         # check which port was really used
# To check port $python -m serial.tools.list_ports
# sudo chmod a+rw /dev/ttyACM1
# ## CONFIG

SERVER_HOST= "localhost"
SERVER_PORT = 8080
API_PATH = "/api/test"
test_url = SERVER_HOST + API_PATH

# ## HELPERS

def compress_nparr(nparr):
    """
    Returns the given numpy array as compressed bytestring,
    the uncompressed and the compressed byte size.
    """
    bytestream = io.BytesIO()
    np.save(bytestream, nparr)
    uncompressed = bytestream.getvalue()
    compressed = zlib.compress(uncompressed)
    return compressed, len(uncompressed), len(compressed)

def uncompress_nparr(bytestring):
    """
    """
    return np.load(io.BytesIO(zlib.decompress(bytestring)))

def preprocess(data):
    aux = data.decode('UTF-8').split(',')
    sens.append(float(aux[0]) / 10)
    sens.append(float(aux[1]) / 10)
    sens.append(float(aux[2].split(';')[0]) / 10)

    return sens


# ## MAIN CLIENT ROUTINE
url = "http://"+SERVER_HOST+":"+str(SERVER_PORT)+API_PATH
#url = "http://6eedbc29acb3.ngrok.io"

# # START CAPTURING
# define the minimum safe distance (in pixels) that two people can be
# from each other
MIN_DISTANCE = 100 # Test/Error
n = 2 #camera
cam = cv2.VideoCapture(n)   #WebCam, if is not working change the number
THRESHOLD = 0.4
H, W = 480,640
classes = []
with open("coco.names","r") as f:
    classes = [line.strip() for line in f.readlines()]

#GET FRAMES IN REAL TIME:
start_time = time.time() #colect start time
frame_id = 0 #colects frame quanty
colors = np.random.uniform(0,255, size=(len(classes),3))
font =cv2.FONT_HERSHEY_PLAIN
results = []



while True:
    ret, frame = cam.read()
    sens = []
    if not ret:
        # Check camera
        # $v4l2-ctl --list-devices
        print("failed to grab frame")
        break

    # encode
    is_success, buffer = cv2.imencode(".jpg", frame)
    io_buf = io.BytesIO(buffer)  #not an image
    # decode
    decode_img = cv2.imdecode(np.frombuffer(io_buf.getbuffer(), np.uint8), -1)  #image
    #print("decode_img type:{}", type(decode_img))

    compressed, u_sz, c_sz = compress_nparr(decode_img)
    #
    '''
    print("\nsending array to", url)
    print("size in bits (orig, compressed):", u_sz, c_sz)
    print ("decode_img type:{}",type(decode_img))
    print("compressed type:{}", type(compressed))
    '''
    #
    resp = requests.post(url, data=compressed,
                         headers={'Content-Type': 'application/octet-stream'})  #HTTP
    ser.write(b'SEND')
    while True:

        data = ser.read()  # Wait forever for anything
        time.sleep(0.01)  # Sleep (or inWaiting() doesn't give the correct value)
        data_left = ser.inWaiting()  # Get the number of characters ready to be read
        data += ser.read(data_left)  # Do the read and combine it with the first character
        # INFO: https://stackoverflow.com/questions/13017840/using-pyserial-is-it-possible-to-wait-for-data

        # Just in case
        if len(data) >= 14:
            break
    sens = preprocess(data)
    print(sens)
    if (sens[2] < 5.0):
        ser.write(b'ALRM')
        print('Impurity is not acceptable!!')


    results = resp.json()

    #[[{'py/tuple': [0.8558927178382874, {'py/tuple': [134, 169, 550, 420]}, {'py/tuple': [342, 295]}]},
    # {'py/tuple': [-1, {'py/tuple': [0, 0, 0, 0]}, [0, -1]]}], {'py/set': [-1, -2]}]print(results)
    '''
    confidence = results[0][i]['py/tuple'][0]
    bboxes = results[0][i]['py/tuple'][1]['py/tuple']
    centroids = results[0][i]['py/tuple'][2]['py/tuple'])
    violations = results[1]['py/set'][i]
    '''
    violations = [val for val in results[1]['py/set']]

    for i, params in enumerate(results[0]):
        # extract the bounding box and centroid coordinates, then
        # initialize the color of the annotation

        (startX, startY, endX, endY) = params['py/tuple'][1]['py/tuple']
        (cX, cY) = params['py/tuple'][2]['py/tuple']
        #conf = params['py/tuple'][0]['py/tuple']
        color = (0, 255, 0)
        # if the index pair exists within the violation set, then
        # update the color
        if i in violations:
            color = (0, 0, 255)
        # draw (1) a bounding box around the person and (2) the
        # centroid coordinates of the person,
        cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)
        cv2.circle(frame, (cX, cY), 3, (90, 0, 252), 2)

    # draw the total number of social distancing violations on the
    # output frame
    text = "Social Distancing Violations: {}".format(len(violations))
    cv2.putText(frame, text, (10, frame.shape[0] - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 255), 3)

    # SPEED TESTER
    elapse = time.time() - start_time

    fps = frame_id / elapse
    # function_putText = (img, text, org, fontFace, fontScale, color, thikness, lineType, botonLeft)
    cv2.putText(frame, "FPS: " + str(round(fps, 2)), (10, 30), font, 3, (0, 255, 0), 1)

    # OUTPUT IMAGE
    cv2.imshow("Scratch_YOLO_CV_2.1", frame)
    key = cv2.waitKey(1)


    if (key % 256 == 27):
        # ESC pressed
        print("Escape hit, closing...")
        cam.release()
        cv2.destroyAllWindows()
        break



