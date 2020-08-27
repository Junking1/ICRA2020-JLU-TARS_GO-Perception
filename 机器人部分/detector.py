#! /home/lyjslay/py3env/bin python
# coding=utf-8
#================================================================
#   Copyright (C) 2019 * Ltd. All rights reserved.
#
#   File name   : detector.py
#   Author      : Liu Yijun
#   E-Mail      : 2446078134@qq.com
#   Description : Object detection based on deeplearning
#
#================================================================
import cv2
import time
import numpy as np
import tensorflow as tf
import tensorflow.contrib.tensorrt as trt
from multiprocessing import Process, Value, Array
from shared_ram import *



class Detector(Process):
    '''Detector Subprocess
    '''
    def __init__(self, name, detecting, tracking, initracker, boundingbox, image_in, direction, ENEMY_COLOR):
        super().__init__()
        self.name = name # process name
        
        # Defined in main process and shared in all processes 
        self.detecting   = detecting
        self.tracking    = tracking
        self.initracker  = initracker
        self.boundingbox = boundingbox
        self.image_in    = image_in
        self.direction   = direction
        # inference concerned param
        self.cls_dict  = {1:'blue',2:'red',3:'front',4:'back',5:'left',6:'right',7:'tracking'}
        self.pb_path   = './model_data/robomaster_trt.pb'
        self.enemy_color = ENEMY_COLOR
        self.conf_th   = 0.5

    def run(self):
        trt_graph = load_trt(self.pb_path)
        tf_sess = create_tfsess(trt_graph)
        while True:
            img = self.image_in[:].copy()
            box_list, cls_list, score_list = detect(img, tf_sess, self.conf_th)
            if len(cls_lsit) != 0:
                box, direc = select_target(box_list, cls_list, score_list, self.enemy_color)
                self.direction = direc[1]
                # xmin,ymin,width,height
                self.boundingbox[:] = [box[1], box[0], box[3]-box[1], box[2]-box[0]] 
                # first start init_tracker.
                self.detecting.value = False
                self.initracker.value = True
                self.tracking.value = False
            else:
                self.boundingbox[:] = None
                self.direction = 7
                rospy.loginfo('no enemy detected')
                continue


def load_trt(pb_path):
    trt_graph_def = tf.GraphDef()
    with tf.gfile.GFile(pb_path, 'rb') as pf:
        trt_graph_def.ParseFromString(pf.read())
    for node in trt_graph_def.node:
        node.device = '/device:CPU:0'
    with tf.Graph().as_default() as trt_graph:
        tf.import_graph_def(trt_graph_def, name='')
    return trt_graph

def load_pb(pb_path):
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(pb_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    return detection_graph

def create_tfsess(trt_graph):
    tf_config = tf.ConfigProto()
    tf_config.gpu_options.allow_growth = True
    tf_config.allow_soft_placement=True
    tf_config.log_device_placement=True
    tf_sess = tf.Session(config=tf_config, graph=trt_graph)
    return tf_sess

def preprocess(src, shape=None, to_rgb=True):
    img = src.astype(np.uint8)
    #resize img for inputs
    if shape:
        img = cv2.resize(img, shape)
    if to_rgb:
        # BGR2RGB
        img = img[..., ::-1]
    return img

def postprocess(img, boxes, scores, classes, conf_thre):
    '''process the output
    '''
    h, w, _ = img.shape
    out_box = boxes[0] * np.array([h, w, h, w])
    out_box = out_box.astype(np.int32)
    out_conf = scores[0]
    out_cls = classes[0].astype(np.int32)
    mask = np.where(out_conf >= conf_thre)
    return out_box[mask],out_cls[mask],out_conf[mask]

def select_target(box_list, cls_list, score_list, ENEMY_COLOR):
        '''select enemy bbox and get enemy direction
        '''
        for box, cls, score in zip(box_list, cls_list, score_list):
            tmp_armor_score = 0
            if cls == ENEMY_COLOR and score > tmp_armor_score:
                tmp_armor_score = score
                armor_box = box
        for box, cls, score in zip(box_list, cls_list, score_list):
            tmp_direc_score = 0
            if cls >=3 and score > tmp_direc_score:
                if box[0] < armor_box[0] and box[2] > armor_box[2]:
                    direction = [box, cls]
        return armor_box, direction

def detect(origimg, tf_sess, conf):
    img = preprocess(origimg, (300, 300))
    boxes_out, scores_out, classes_out = tf_sess.run(
        [tf_boxes, tf_scores, tf_classes],
        feed_dict={image_tensor: img[None, ...]})
    # process outputs
    box, cls, score = postprocess(origimg, boxes_out, scores_out, classes_out, conf)
    return (box, cls, score) 

#单独测试detector
if __name__ == '__main__':
    ENEMY_COLOR = 2 #blue
    MODEL_PATH = './model_data/robomaster_trt.pb'
    cls_dict = {1:'red',2:'blue',3:'front',4:'back',5:'left',6:'right',7:'tracking'}
    color_dict = {1:(255,0,0),2:(0,255,0),3:(0,0,255),4:(255,255,0),
                  5:(255,0,255),6:(0,255,255),7:(255,255,255)}
    detection_graph = load_pb(MODEL_PATH)
    tf_sess = create_tfsess(detection_graph)
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
    tf_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    tf_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    tf_classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
    video = cv2.VideoCapture('test.flv')
    while(video.isOpened()):
        ret, frame = video.read()
        start = time.time()
        box,cls,score = detect(frame,tf_sess,0.2)
        end = time.time()
        print(box,cls,score)
        infer_time = round((end-start)*1000,2)
        text = 'inference time: '+str(infer_time)+' ms'
        if len(box) != 0:
            for b,c,s in zip(box,cls,score):
                box_text = str(cls_dict[c])+' '+str(round(s,3))
                color = color_dict[c]
                cv2.rectangle(frame,(b[1],b[0]), (b[3],b[2]), color, 2)
                cv2.putText(frame, box_text, (b[1], b[0]-15), cv2.FONT_HERSHEY_PLAIN, 1.4, color,1, cv2.LINE_AA)
            cv2.putText(frame, text, (11, 20), cv2.FONT_HERSHEY_PLAIN, 1.4, (32,32,32),4, cv2.LINE_AA)
            cv2.putText(frame, text, (10, 20), cv2.FONT_HERSHEY_PLAIN, 1.4, (240,240,240),1, cv2.LINE_AA)
        cv2.imshow('tensorrt detector', frame)
        if cv2.waitKey(1) == ord('q'):
            break
    video.release()
    cv2.destroyAllWindows()
