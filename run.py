import os
import threading
import time
import sys


def start_working():

    def worker_social_aware():
        print('starting social aware...')
        os.system('./run_social.sh') # > /dev/null 2>&1')
        return

    def worker_usb_camera():
        print('starting face_tracking...')
        os.system('./run_face_tracking.sh > /dev/null 2>&1')
        return

    def worker_face_tracking():
        print('starting face_tracking...')
        os.system('./run_face_tracking.sh > /dev/null 2>&1')
        return

    def worker_expose():
        print('starting expose...')
        os.system('python expose.py')
        return

    def worker_patricc_controller():
        print('starting patricc controller...')
        os.system('python patricc_controller.py')
        return


    def run_thread(worker):
        threading.Thread(target=worker).start()
        threading._sleep(2.0)

    #run_thread(worker_social_aware)
    run_thread(worker_face_tracking)
    run_thread(worker_expose)
    run_thread(worker_patricc_controller)

start_working()