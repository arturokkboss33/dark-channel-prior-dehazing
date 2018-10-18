#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
from functools import partial
from PIL import Image
from dehaze import dehaze

def set_filenames(img_name,depth_img=None):
    """Return tuples for source and template destination"""
    SRC_DIR = 'imgs'
    DEST_DIR = 'results'
    file_dir = os.path.dirname(os.path.realpath(__file__))
    parent_dir, _ = os.path.split(file_dir)
    src_path = os.path.join(parent_dir, SRC_DIR)
    dest_path = os.path.join(parent_dir, DEST_DIR)
    base, ext = os.path.splitext(img_name)
    tempname = base + '_result'
    src_file = os.path.join(src_path, img_name)
    dest_file = os.path.join(dest_path, tempname)

    if depth_img is None:
        return src_file, dest_file
    else:
        depth_file = os.path.join(src_path,depth_img)
        return src_file, dest_file, depth_file

def generate_results(src, dest, generator, depth_img_src=None):
    print('processing', src + '...')
    img = Image.open(src)
    depth_img = None
    if depth_img_src is not None:
        depth_img = Image.open(depth_img_src)
    
    dark, raw_trans, refined_trans, raw_rad, refined_rad = generator(img,depth_img=depth_img)
    if depth_img_src is None:
        dark.save(dest + '_dark-channel.png')
        raw_trans.save(dest + '_raw-transmission.png')
        refined_trans.save(dest + '_refined-transmission.png')
    
    raw_rad.save(dest + '_raw-radiance.png')
    refined_rad.save(dest + '_refined-radiance.png')
    print('saved', dest)


def main():
    """Parse users arguments and apply DCP to each specified file."""
    parser = argparse.ArgumentParser()

    parser.add_argument("-i", "--input", type=str,
                        help="image filename")
    parser.add_argument("-t", "--tmin", type=float, default=0.2,
                        help="minimum transmission rate")
    parser.add_argument("-A", "--Amax", type=int, default=220,
                        help="maximum atmosphere light")
    parser.add_argument("-w", "--window", type=int, default=15,
                        help="window size of dark channel")
    parser.add_argument("-r", "--radius", type=int, default=40,
                        help="radius of guided filter")
    parser.add_argument("-fuw", "--enable_underwater", type=bool, default=False,
                        help="enable DCP for underwater images")
    parser.add_argument("-depth", "--depth", type=str,
                        help="depth image")

    args = parser.parse_args()

    ##NOTE:Loop through each file and apply DCP
    if args.input is not None:
        img_files = args.input.split(" ")
        #print(img_files) #DEBUGGING LINE

        ##NOTE:Check if a depth image was given
        flag_use_depth = False
        depth_img_src = None
        depth_files = []
        if args.depth is not None:
            depth_files = args.depth.split(" ")
            flag_use_depth = True
        for img_idx,img in enumerate(img_files):
            if flag_use_depth is False:
                src, dest = set_filenames(img)
                print(src);print(dest); #DEBUGGING LINE
            else:
                src, dest, depth_img_src = set_filenames(img,depth_files[img_idx])
                print(src);print(dest);print(depth_img_src) #DEBUGGING LINE

            dest = dest + ("_%d%s_%d_%d_%d" % (args.tmin * 100, "e-2", args.Amax, args.window, args.radius))
            #print(dest) #DEBUGGING LINE
            generate_results(src, dest, partial(dehaze, t_min=args.tmin, atm_max=args.Amax, w=args.window, 
                                               r=args.radius, flag_uw=args.enable_underwater), depth_img_src)
    else:
        print("No input files were given")
    

if __name__ == '__main__':
    main()
