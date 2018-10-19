#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Implementation for Single Image Haze Removal Using Dark Channel Prior."""

import numpy as np
from PIL import Image
from guidedfilter import guided_filter

R, G, B = 0, 1, 2  # index for convenience
L = 256  # color depth


class DCPDehaze():
    _R,_G,_B = 0,1,2
    _max_color_val = 256
    _transmission_bias = 0.95
    _atm_pixel_percent = 0.0001
    _guided_filter_eps = 1e-3

    ##NOTE:Constructor with default values
    def __init__(self,trans_min=0.2,atm_max=220,window_size=15,guided_filter_radius=40,
                enable_underwater=False,attenuation_coeffs=None):
        self.trans_min = trans_min
        self.atm_max = atm_max
        self.window_size = window_size
        self.guided_filter_radius = guided_filter_radius
        self.attenuation_coeffs = attenuation_coeffs
        self.enable_underwater = enable_underwater
        self.atm_light = None
    
    ##NOTE: Functions necessary to perform dehazing

    ##NOTE: Main dehaze function, runs the DCP pipeline
    def dehaze(self,src_image,depth_image=None):

        ##NOTE:Check if depth image was provided
        flag_use_depth = True if depth_image is not None else False

        ##NOTE:Get dark channel image
        ##NOTE:Change color space if dealing with underwater images
        p_image = np.zeros(src_image.shape, dtype=np.float64)
        if self.enable_underwater:
            p_image[:, :, 0] = 255. - src_image[:, :, 0]
            p_image[:, :, 1] = 255. - src_image[:, :, 1]
            p_image[:, :, 2] = src_image[:, :, 2]
        else:
            p_image = src_image
        dark_image = [] if flag_use_depth else self.get_dark_channel(p_image)  

        ##NOTE:Estimate the atmospheric light, this is done always with the original image
        self.atm_light = self.get_atmosphere(src_image, dark_image, depth_image)
        self.atm_light = np.minimum(self.atm_light, self.atm_max)

        ##NOTE:Estimate transmission image which correlates to depth
        refined_trans = None
        if not flag_use_depth:
            ##NOTE: Refine transmission rate through guided filter (edge-preserving filter)
            raw_trans = self.get_transmission(p_image)
            raw_trans = refined_trans = np.maximum(raw_trans, self.trans_min)
            norm_image = (src_image - src_image.min()) / (src_image.max() - src_image.min())
            refined_trans = guided_filter(norm_image, refined_trans, self.guided_filter_radius, self._guided_filter_eps)

        ##NOTE:Recover dehazed (radiant) image        
        dehazed_image = self.get_radiance(src_image, refined_trans, depth_image)
        dehazed_image = np.maximum(np.minimum(dehazed_image, self._max_color_val - 1), 0).astype(np.uint8)

        return dehazed_image


    ##NOTE: Function to get the dark channel prior of the image
    def get_dark_channel(self,src_image):        
        padded = np.pad(src_image, ((self.window_size / 2, self.window_size / 2), 
                                    (self.window_size / 2, self.window_size / 2), (0, 0)), 'edge')
        dark_channel = np.zeros((src_image.shape[0], src_image.shape[1]))
        ##NOTE:Choose the minimum pixel value in the window
        for i, j in np.ndindex(dark_channel.shape):
            ##NOTE:If no axis is given for he minimum, the array is flattened
            ##NOTE: CVPR09, eq.5
            dark_channel[i, j] = np.min(padded[i:i + self.window_size, j:j + self.window_size, :])

        return dark_channel
    
    def get_atmosphere(self,src_image,dark_channel,depth_image):
        ##NOTE:If depth is given, copmute the average of pixels values in the horizon
        ##else follow the normal DCP implementation
        if depth_image is not None:
            flat_depth = depth_image.ravel()
            flat_depth = np.nan_to_num(flat_depth)*255.
            flat_depth = np.maximum(np.minimum(flat_depth, 255.), 0.0001)/255.
            flat_image = src_image.reshape(depth_image.shape[0]*depth_image.shape[1], 3)
            ##TODO:Have a closer look at how atmosphere light affects the end result
            return np.average(flat_image, axis=0, weights=1./flat_depth)
        else:
            ##NOTE: CVPR09, eq. 4.4
            flat_image = src_image.reshape(src_image.shape[0]*src_image.shape[1], 3)
            flat_dark_channel = dark_channel.ravel()
            ##NOTE: Find top width * height * p% indexes
            search_idx = (-flat_dark_channel).argsort()[:int(src_image.shape[0]*src_image.shape[1] * self._atm_pixel_percent)]
            ##NOTE: Return the highest intensity for each channel
            return np.max(flat_image.take(search_idx, axis=0), axis=0)
    
    def get_transmission(self,src_image):
        ##NOTE: CVPR09, eq.12
        return 1 - self._transmission_bias * self.get_dark_channel(src_image / self.atm_light, self.window_size)
    
    def get_radiance(self,src_image, trans_image, depth_image=None):
        ##NOTE: Tile transmission image
        tiled_trans = np.zeros_like(src_image)
        if depth_image is not None:
            transmission_B = np.exp(-self.attenuation_coeffs[0]*depth_image)
            transmission_G = np.exp(-self.attenuation_coeffs[1]*depth_image)
            transmission_R = np.exp(-self.attenuation_coeffs[2]*depth_image)
            tiled_trans[:, :, 0] = transmission_B
            tiled_trans[:, :, 1] = transmission_G
            tiled_trans[:, :, 2] = transmission_R
        else:
            tiled_trans[:, :, B] = tiled_trans[:, :, G] = tiled_trans[:, :, R] = trans_image

        ##NOTE: CVPR09, eq.16
        return (src_image -self.atm_light) / tiled_trans + self.atm_light
