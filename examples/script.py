from maskrcnn_benchmark.config import cfg
from demo.predictor import COCODemo

from RDTSeg import RTSeg
from sptam.dynaseg import DynaSegt,DynaSeg
from sptam.msptam import SPTAM, stereoCamera
from sptam.components import Camera
from sptam.components import StereoFrame
from sptam.feature import ImageFeature
from sptam.params import ParamsKITTI
from sptam.dataset import KITTIOdometry

# import orbslam2
# import g2o

import sys
import cv2 as cv
import traceback
import numpy as np
import os
import shutil
from threading import Thread

def load_images(path_to_sequence):
    res = [os.path.join(path_to_sequence, img) for img in os.listdir(path_to_sequence)]
    res.sort()
    return res

def load_times(path_to_sequence):
    timestamps = []
    with open(os.path.join(path_to_sequence, 'times.txt')) as times_file:
        for line in times_file:
            if len(line) > 0:
                timestamps.append(float(line))
    return timestamps

def pose_to_transformation(pose):
    res = np.zeros((4,4))
    for i in range(3):
        res[i,:3] = pose[4*i+1:4*(i+1)]
        res[i,3] = pose[4*i]
    res[3,3] = 1
    res = np.linalg.inv(res)
    return res

sequence = sys.argv[1]
mode = sys.argv[2]

# sequence_path = os.path.join('/storage/remote/atcremers17/linp/dataset/kittic/sequences/',sequence)
sequence_path = os.path.join('/storage/remote/atcremers54/linp/kittic/sequences/',sequence)
file_path = os.path.join(sequence_path, 'image_2')
left_filenames = load_images(file_path)
file_path = os.path.join(sequence_path, 'image_3')
right_filenames = load_images(file_path)
timestamps = load_times(sequence_path)

prob_path = os.path.join('/usr/stud/linp/storage/user/linp/prob/', sequence)
prob_filenames = load_images(prob_path)

config_file = '../../maskrcnn-benchmark/configs/caffe2/e2e_mask_rcnn_R_50_FPN_1x_caffe2.yaml'
cfg.merge_from_file(config_file)
# manual override some options
cfg.merge_from_list(["MODEL.DEVICE", 'cuda'])
coco_demo = COCODemo(
    cfg,
    min_image_size=800,
    confidence_threshold=0.7,
)

dilation = 2
kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2 * dilation + 1, 2 * dilation + 1))

depth_path = os.path.join('/usr/stud/linp/storage/user/linp/depth/',sequence)

iml = cv.imread(left_filenames[0], cv.IMREAD_UNCHANGED)
f = 0
config = stereoCamera(sequence)
num_images = len(left_filenames)
if mode != 'm':
    if mode == 'rt' or mode == 'r':
        rtseg = RTSeg(iml,coco_demo,depth_path,kernel,config)




    dpath = 'pmask/{}{}/'.format(mode,sequence)
    if os.path.exists(dpath):
        shutil.rmtree(dpath)
    os.mkdir(dpath)

    print('sequence ',sequence)
    print(mode)



    for idx in range(num_images):
        print('{} frame'.format(idx))
        left_image = cv.imread(left_filenames[idx], cv.IMREAD_UNCHANGED)
        right_image = cv.imread(right_filenames[idx], cv.IMREAD_UNCHANGED)
        prob_image = cv.imread(prob_filenames[idx])
        # rt
        if mode == 'rt':
            c = rtseg.rt_seg_track(left_image, prob_image,idx)
            cv.imwrite(os.path.join(dpath, '{0:06}.png'.format(idx)), c*255)
        # r
        elif mode == 'r':
            c = rtseg.rt_seg_t(left_image, prob_image)
            cv.imwrite(os.path.join(dpath, '{0:06}.png'.format(idx)), c)
else:
    dpath = 'mask/o{}/'.format(sequence)
    if os.path.exists(dpath):
        shutil.rmtree(dpath)
    os.mkdir(dpath)

    print('sequence ',sequence)
    for idx in range(num_images):
        print('{} frame'.format(idx))
        left_image = cv.imread(left_filenames[idx], cv.IMREAD_UNCHANGED)
        a = coco_demo.compute_prediction(left_image)
        top = coco_demo.select_top_predictions(a)
        masks = top.get_field("mask").numpy()
        c = np.zeros(left_image.shape[:2])
        for m in masks:
            c[m.squeeze()] = 255
        cv.imwrite(os.path.join(dpath, '{0:06}.png'.format(idx)), c)