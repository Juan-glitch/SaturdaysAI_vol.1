################################################################################
### server.py
################################################################################
#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""
Dummy server, interacts with the client receiving, altering and returning
compressed numpy arrays.
Setup/run:
 1. pip install Flask --user
 2. export FLASK_APP=server.py; flask run
"""

# ### MODULES
import io
import zlib
from flask import Flask, request, Response, jsonify
# VISION
import numpy as np
import os
import cv2
import jsonpickle
import json
import time
import ssl
import sys
from modules import impure_detector



# ## CONFIG
# FLASK
SERVER_HOST= "localhost"
SERVER_PORT = 8080
API_PATH = "/api/test"
# VISION
