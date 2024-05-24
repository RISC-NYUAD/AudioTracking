#!/usr/bin/env python
import os,sys
import rospy
import torch
import numpy as np
import torch.backends.cudnn as cudnn
import cv2
from cv_bridge import CvBridge

from std_msgs.msg import Float32MultiArray

import torch.nn.functional as F
import torch.distributed as dist
from timm.utils import AverageMeter

import matplotlib.pyplot as plt

from swin import SwinTransformer
from utils import *
from sensor_msgs.msg import Image

sr = 48000
channels = 1
pan_tilt_angles = Float32MultiArray()
bridge = CvBridge()

def detect_audio(signal):
    in_data = np.array(signal.data)
    spectrum = audio2spectrum(in_data, sr, channels)
    spectrum = cv2.resize(spectrum, (640, 320))
    #cv2.imshow('frame', spectrum)
    #cv2.waitKey(30)
    librosa.display.specshow(librosa.power_to_db(spectrum,                                              
        ref=np.max), y_axis='mel', fmax=8000, x_axis='time')
    plt.colorbar(format='%+2.0f dB')
    plt.title('Mel spectrogram')
    plt.tight_layout()
    plt.show()
    cv2.waitKey(30)
    plt.clf()
    
    '''
    spectrum = torch.from_numpy(spectrum).permute(2,0,1)
    spectrum = spectrum.cuda()[None,...].float()

    outputs = model(spectrum)
    outputs = F.interpolate(outputs, (90,140)).data.cpu().numpy()[0,0]
    
    heat_map = colorize(outputs)
    imgMsg = bridge.cv2_to_imgmsg(heat_map, "bgr8")
    pub_heat_map.publish(imgMsg)
    
    ind_out = np.unravel_index(outputs.argmax(), outputs.shape)
    pan_tilt_angles.data = ind_out
    pub_pan_tilt.publish(pan_tilt_angles)
    '''

    rospy.loginfo("process spectrum")

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    '''
    cfg_path = rospy.get_param('/cfg_path', 'src/mics_track/config/trackv2_1115.yaml')
    cfg = load_setting(cfg_path)
    model = SwinTransformer(img_size=(cfg.height, cfg.width), patch_size=cfg.patch_size, in_chans=cfg.channels,
                        embed_dim=cfg.embed_dim, depths=cfg.depths, num_heads=cfg.num_heads,
                        window_size=cfg.window_size, mlp_ratio=cfg.mlp_ratio)

    weights = rospy.get_param('/weights', 'src/mics_track/weights/ckpt_epoch_40_0.000.pth')
    model.load_state_dict(torch.load(weights)['model'])
    model.eval()
    model.cuda() 


    pub_heat_map = rospy.Publisher('/camera/rgb/heat_map', Image, queue_size=10)
    pub_pan_tilt = rospy.Publisher('/audio/pantiltangles', Float32MultiArray, queue_size=10)
    '''       
    rospy.Subscriber("/audio/spectrum", Float32MultiArray, detect_audio)
    rospy.spin()    
    