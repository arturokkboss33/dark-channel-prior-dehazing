#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Implementation for Single Image Haze Removal Using Dark Channel Prior.

Reference:
http://research.microsoft.com/en-us/um/people/kahe/cvpr09/
http://research.microsoft.com/en-us/um/people/kahe/eccv10/
"""

import numpy as np
from PIL import Image
from PIL import ImageFilter

from guidedfilter import guided_filter

R, G, B = 0, 1, 2  # index for convenience
max_color_val = 256  # color depth


def get_dark_channel(I, w):
    """Get the dark channel prior in the (RGB) image data.

    Parameters
    -----------
    I:  an M * N * 3 numpy array containing data ([0, max_color_val-1]) in the image where
        M is the height, N is the width, 3 represents R/G/B channels.
    w:  window size

    Return
    -----------
    An M * N array for the dark channel prior ([0, max_color_val-1]).
    """
    M, N, _ = I.shape
    padded = np.pad(I, ((w / 2, w / 2), (w / 2, w / 2), (0, 0)), 'edge')
    dark_ch = np.zeros((M, N))
    # NOTE:Choose the minimum pixel value in the window
    for i, j in np.ndindex(dark_ch.shape):
        # NOTE:If no axis is given for he minimum, the array is flattened
        # NOTE: CVPR09, eq.5
        dark_ch[i, j] = np.min(padded[i:i + w, j:j + w, :])
    return dark_ch


def get_atmosphere(I, dark_ch, p, depth_img=None):
    """Get the atmosphere light in the (RGB) image data.

    Parameters
    -----------
    I:       the M * N * 3 RGB image data ([0, max_color_val-1]) as numpy array
    dark_ch: the dark channel prior of the image as an M * N numpy array
    p:       percentage of pixels for estimating the atmosphere light

    Return
    -----------
    A 3-element array containing atmosphere light ([0, max_color_val-1]) for each channel
    """
    # NOTE:If depth is given, copmute the average of pixels values in the horizon
    # else follow the normal DCP implementation
    if depth_img is not None:
        M, N = depth_img.shape
        flat_depth = depth_img.ravel()
        flat_depth = np.nan_to_num(flat_depth)*255.
        flat_depth = np.maximum(np.minimum(flat_depth, 255.), 0.0001)/255.
        flat_img = I.reshape(M*N, 3)

        return np.average(flat_img, axis=0, weights=flat_depth)
    else:
        # NOTE: CVPR09, eq. 4.4
        M, N = dark_ch.shape
        flat_I = I.reshape(M * N, 3)
        flat_dark = dark_ch.ravel()
        # NOTE: Find top M * N * p indexes
        search_idx = (-flat_dark).argsort()[:int(M * N * p)]
        # print 'atmosphere light region:', [(i / N, i % N) for i in search_idx] #DEBUGGING LINE

        # NOTE: Return the highest intensity for each channel
        return np.max(flat_I.take(search_idx, axis=0), axis=0)


def get_transmission(I, atm_light, omega, w):
    """Get the transmission estimate in the (RGB) image data.

    Parameters
    -----------
    I:          the M * N * 3 RGB image data ([0, max_color_val-1]) as numpy array
    atm_light:a 3-element array containing atmosphere light
                ([0, max_color_val-1]) for each channel
    omega:      bias for the estimate
    w:          window size for the estimate

    Return
    -----------
    An M * N array containing the transmission rate ([0.0, 1.0])
    """
    return 1 - omega * get_dark_channel(I / atm_light, w)  # CVPR09, eq.12


def dehaze_raw(I, t_min=0.2, atm_max=220, w=15, p=0.0001,
               omega=0.95, guided=True, r=40, eps=1e-3, flag_uw=False, depth_img=None):
    """Get the dark channel prior, atmosphere light, transmission rate
       and refined transmission rate for raw RGB image data.

    Parameters
    -----------
    I:      M * N * 3 data as numpy array for the hazy image
    t_min:  threshold of transmission rate
    atm_max:threshold of atmosphere light
    w:      window size of the dark channel prior
    p:      percentage of pixels for estimating the atmosphere light
    omega:  bias for the transmission estimate
    flag_uw:enable DCP for underwater imgs

    guided: whether to use the guided filter to fine the image
    r:      the radius of the guidance
    eps:    epsilon for the guided filter

    Return
    -----------
    (Idark, A, rawt, refinedt) if guided=False, then rawt == refinedt
    """

    # NOTE:Check if depth image was provided
    flag_use_depth = False
    if depth_img is not None:
        flag_use_depth = True

    # NOTE:First, get dark channel image
    # NOTE:Change color space if dealing with underwater images
    Iprime = np.zeros(I.shape, dtype=np.float64)
    if flag_uw:
        Iprime[:, :, 0] = 255. - I[:, :, 0]
        Iprime[:, :, 1] = 255. - I[:, :, 1]
        Iprime[:, :, 2] = I[:, :, 2]
    else:
        Iprime = I

    Idark = [] if flag_use_depth else get_dark_channel(Iprime, w)

    # NOTE:Estimate the atmospheric light, this is done always with the original image
    atm_light = get_atmosphere(I, Idark, p, depth_img)
    atm_light = np.minimum(atm_light, atm_max)
    print('atmosphere', atm_light)

    # NOTE:Estimate transmission image which correlates to depth
    if flag_use_depth:
        M, N, _ = I.shape
        white = np.full_like(
            np.zeros((M, N), dtype=np.float64), max_color_val - 1)
        return white, atm_light, white, white
    else:
        rawt = get_transmission(Iprime, atm_light, omega, w)
        print('raw transmission rate between [%.4f, %.4f]' % (
            rawt.min(), rawt.max()))
        # NOTE: Refine transmission rate through guided filter (edge-preserving filter)
        rawt = refinedt = np.maximum(rawt, t_min)
        if guided:
            normI = (I - I.min()) / (I.max() - I.min())
            refinedt = guided_filter(normI, refinedt, r, eps)

        print('refined transmission rate between [%.4f, %.4f]' % (
            refinedt.min(), refinedt.max()))

        return Idark, atm_light, rawt, refinedt


def get_radiance(I, atm_light, trans, depth_img=None):
    """Recover the radiance from raw image data with atmosphere light
       and transmission rate estimate.

    Parameters
    ----------
    I:              M * N * 3 data as numpy array for the hazy image
    atm_light:      a 3-element array containing atmosphere light
                    ([0, max_color_val-1]) for each channel
    trans:          estimate fothe transmission rate

    Return
    ----------
    M * N * 3 numpy array for the recovered radiance
    """

    # NOTE: Tile transmission image
    tiledt = np.zeros_like(I)
    attenuation_coeff = np.array([0.5, 0.5, 0.5])
    if depth_img is not None:
        transmission_B = np.exp(-attenuation_coeff[0]*depth_img)
        transmission_G = np.exp(-attenuation_coeff[1]*depth_img)
        transmission_R = np.exp(-attenuation_coeff[2]*depth_img)
        tiledt[:, :, 0] = transmission_B
        tiledt[:, :, 1] = transmission_G
        tiledt[:, :, 2] = transmission_R
    else:
        tiledt[:, :, R] = tiledt[:, :, G] = tiledt[:, :, B] = trans

    # NOTE: CVPR09, eq.16
    return (I - atm_light) / tiledt + atm_light


def to_img(raw):
    # NOTE: Threshold image to be in the range 0 - max_color_val-1
    cut = np.maximum(np.minimum(raw, max_color_val - 1), 0).astype(np.uint8)

    return Image.fromarray(cut)


def dehaze(img, t_min=0.2, atm_max=220, w=15, p=0.0001,
           omega=0.95, guided=True, r=40, eps=1e-3, flag_uw=False, depth_img=None):
    """Dehaze the given RGB image.

    Parameters
    ----------
    img:        the Image object of the RGB image
    guided:     refine the dehazing with guided filter or not
    other parameters are the same as `dehaze_raw`
    flag_uw:    change DCP computation to adjust to underwater imgs
    depth_img:  if a depth image is provided, transmission is not computed

    Return
    ----------
    (dark, rawt, refinedt, rawrad, rerad)
    Images for dark channel prior, raw transmission estimate,
    refiend transmission estimate, recovered radiance with raw t,
    recovered radiance with refined t.
    """

    I = np.asarray(img, dtype=np.float64)
    Idepth = None
    if depth_img is not None:
        depth_img = depth_img.convert("L")
        Idepth = np.asarray(depth_img, dtype=np.float64)
    Idark, atm_light, rawt, refinedt = dehaze_raw(I, t_min, atm_max, w, p,
                                                  omega, guided, r, eps, flag_uw, Idepth)
    M, N, _ = I.shape
    white = np.full_like(np.zeros((M, N), dtype=np.float64), max_color_val - 1)

    return [to_img(raw) for raw in (Idark, white * rawt, white * refinedt,
                                    get_radiance(I, atm_light, rawt, Idepth),
                                    get_radiance(I, atm_light, refinedt, Idepth))]
