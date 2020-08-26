#! /home/lyjslay/py3env/bin python
# coding=utf-8
#================================================================
#   Copyright (C) 2019 * Ltd. All rights reserved.
#
#   File name   : tracker.py
#   Author      : Liu Yijun
#   E-Mail      : 2446078134@qq.com
#   Description : KCF tracker
#
#================================================================
import KCF
import cv2
from time import time
from shared_ram import *
from multiprocessing import Process, Value, Array

class Tracker(Process):
	'''KCF Tracker Subprocess
	'''
	def __init__(self, name, detecting, tracking, initracker, boundingbox, is_enemy, flag, image_in):
		
		super().__init__()
		self.name = name # process name

		# Defined in main process and shared in all processes
		self.detecting   = detecting
		self.tracking    = tracking
		self.initracker  = initracker
		self.boundingbox = boundingbox
		self.flag        = flag
		self.image_in    = image_in

	def run(self):
		# create kcftracker instance
		tracker = KCF.kcftracker(False, False, False, False)# hog, fixed_window, multiscale, lab
		while True:
			# detector process is runing, tracker process blocks
			if self.detecting.value is True:
				continue
            # successfully get boundingbox from detector
			elif self.initracker.value is True:
				print('initing')
				frame = self.image_in[:].copy()
				tracker.init(self.boundingbox[:], frame)
				self.detecting.value = False
				self.initracker.value = False
				self.tracking.value = True
			# start tracking
			elif self.tracking.value is True:
				print('tracking')
				frame = self.image_in[:].copy()
				tracker_box = tracker.update(frame)
				# [xmin,ymin,w,h]
				tracker_box = list(map(int, tracker_box)) 
				# transform format
				box = [tracker_box[1], tracker_box[0], tracker_box[1]+tracker_box[3], tracker_box[0]+tracker_box[2]] 
				self.flag.value += 1
				if self.flag.value > 20:
					self.flag.value = 0
					self.detecting.value = True
					self.initracker.value = False
					self.tracking.value = False


def draw_boundingbox(event, x, y, flags, param):
	global selectingObject, initTracking, onTracking, ix, iy, cx,cy, w, h
	if event == cv2.EVENT_LBUTTONDOWN:
		selectingObject = True
		onTracking = False
		ix, iy = x, y
		cx, cy = x, y
	elif event == cv2.EVENT_MOUSEMOVE:
		cx, cy = x, y
	elif event == cv2.EVENT_LBUTTONUP:
		selectingObject = False
		if(abs(x-ix)>10 and abs(y-iy)>10):
			w, h = abs(x - ix), abs(y - iy)
			ix, iy = min(x, ix), min(y, iy)
			initTracking = True
		else:
			onTracking = False
	elif event == cv2.EVENT_RBUTTONDOWN:
		onTracking = False
		if(w>0):
			ix, iy = x-w/2, y-h/2
			initTracking = True

#单独测试Tarcker
if __name__ == '__main__':
	selectingObject = False
	initTracking = False
	onTracking = False
	ix, iy, cx, cy = -1, -1, -1, -1
	w, h = 0, 0
	inteval = 1
	duration = 0.01
	cap = cv2.cv2.VideoCapture('tracker.avi')

	tracker = KCF.kcftracker(True, True, True, True)  # hog, fixed_window, multiscale, lab
	cv2.namedWindow('tracking')
	cv2.setMouseCallback('tracking',draw_boundingbox)
	ret, frame = cap.read()
	cv2.putText(frame, 'Use mouse to select the armor', (8,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
	while(cap.isOpened()):
		if(selectingObject):
			cv2.rectangle(frame,(ix,iy), (cx,cy), (0,255,255), 1)
		elif(initTracking):
			ret, frame = cap.read()
			cv2.rectangle(frame,(ix,iy), (ix+w,iy+h), (0,255,255), 2)
			tracker.init([ix,iy,w,h], frame)
			initTracking = False
			onTracking = True
		elif(onTracking):
			ret, frame = cap.read()
			t0 = time()
			boundingbox = tracker.update(frame) 
			t1 = time()
			boundingbox = list(map(int, boundingbox))
			print(boundingbox)
			cv2.rectangle(frame,(boundingbox[0],boundingbox[1]), (boundingbox[0]+boundingbox[2],boundingbox[1]+boundingbox[3]), (0,255,255), 1)
			duration = 0.8*duration + 0.2*(t1-t0)
			cv2.putText(frame, 'Use mouse to select target', (8,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
			cv2.putText(frame, 'FPS: '+str(1/duration)[:4].strip('.'), (8,40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
		cv2.imshow('tracking', frame)
		c = cv2.waitKey(inteval) & 0xFF
		if c==27 or c==ord('q'):
			break
