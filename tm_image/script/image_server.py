import os
import signal
import sys
import socket
import queue
import signal
import threading

import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError

from flask import Flask, request, jsonify
import numpy as np
import cv2
from waitress import serve
from datetime import datetime

class ImageServer(object):
    def __init__(self) -> None:
        rospy.init_node('image_server_node')
        self._image_publiser = rospy.Publisher('tm_image', Image, queuesize=10)
        self._app = Flask(__name__)
        self._set_route()

        # Data
        self._data_lock = threading.RLock()
        self._data_available = False
        self._data_queue = queue.Queue()
        
        # Pub thread
        self._data_pub_thread = threading.Thread(target=self._pub_thread)

    def _post_image(self, m_method):
        print('\n[{0}] [{1}] -> Post({2})'.format(request.environ['REMOTE_ADDR'], datetime.now(), m_method))          
        # get key/value
        model_id = request.args.get('model_id')
        print('model_id: {}'.format(model_id))

        # check key/value
        if model_id is None:
            print("model_id is not set")    
            result={                    
                "message": "fail",
                "result": "model_id required"
            }  
            return jsonify(result)
        image = request.files['file'].read()
        with self._data_lock:
            self._data_queue.put(image)
            self._data_available = True

        result = {
            "result": "status",
            "message": "im ok"
        }
        return jsonify(result)
    
    def _check_api_status(self, m_method):
        print('\n[{0}] [{1}] -> Get({2})'.format(request.environ['REMOTE_ADDR'], datetime.now(), m_method))
        # user defined method
        if m_method == 'status':
            result = {
                "result": "status",
                "message": "im ok"
            }
        else:
            result = {
                "result": "fail",
                "message": "wrong request"            
            }
        return jsonify(result)
    
    def _check_api_status_running(self, m_method):
        print('\n[{0}] [{1}] -> Get()'.format(request.environ['REMOTE_ADDR'], datetime.now()))
        # user defined method
        result = {
            "result": "api",
            "message": "running",
        } 
        return jsonify(result)  
        
    def _set_route(self):
        self._app.route('/api/<string:m_method>', methods=['POST'])(self._post_image)
        self._app.route('/api/<string:m_method>', methods=['GET'])(self._check_api_status)
        self._app.route('/api', methods=['GET'])(self._check_api_status_running)

    def _pub_thread(self):
        # Should be while ok
        while True:
            # Wait until data is available
            while self._data_queue.empty():
                pass
            with self._data_lock:
                file2np = np.fromstring(self._data_queue.get(), np.uint8)        
                img = cv2.imdecode(file2np, cv2.IMREAD_UNCHANGED)
                self._publish_image(img)

    def _publish_image(self,image):
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(image)
        self._image_publiser.publish(msg)


    def run(self):
        self._data_pub_thread.start()
        serve(self._app, port=6189)

if __name__ == '__main__':
    server = ImageServer()
    server.run()