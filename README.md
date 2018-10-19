## About

* Forked from [github repository](https://github.com/joyeecheung/dark-channel-prior-dehazing)
* Integrate methods into a class and create a ROS node to use (**tested in ROS INDIGO**).
* Add option to dehaze **underwater images** where *haze* or *fog* is perceived differently.
* Add option to use an already precomputed depth image to recover image original colors. This will effectively skip the DCP algorithm and use depth to compute *tranmission* directly. For more info please refer to these [slides](http://kaiminghe.com/cvpr09/cvpr09slides.pdf) or the articles mention in the reference section. 

## Run Dark Channel Prior Dehazing (DCP)

1. Change the DCP parameter values in the config file `config/YOUR_CONFIG_FILE.yml` or create your own.
2. Run the package launch file as `roslaunch dcp_dehaze run_dcp_dehaze.launch`.

### Parmeters description

* **hazed_image_topic**: String. ROS topic name of the original image, which is distorted due to haze.
* **depth_image_topic**: String. Topic with the image filename containing depth information (e.g. Gazebo depth camera). If this used DCP is skipped and color is recovered based on depth. 
* **dehazed_image_topic**: String. ROS topic name fo the dehazed image after applying DCP.  
* **minimum_transmission**: Float. Minimum value for the trasmission rate; see paper [slides](http://kaiminghe.com/cvpr09/cvpr09slides.pdf).
* **max_atm_light**: Int. Maximum RGB value the atmospheric light can have in the image; see paper [slides](http://kaiminghe.com/cvpr09/cvpr09slides.pdf).
* **dehaze_window_size**: Int. Window or kernel size used to compute the dark channel prior; see paper [slides](http://kaiminghe.com/cvpr09/cvpr09slides.pdf).
* **guided_filter_radius**: Int. Kernel size used in the guided filter to smooth the transmission/depth estimation; see [explanation](http://kaiminghe.com/eccv10/).
* **enable_underwater**: Boolean. If true the algorithm changes the source image color space to dehaze based on underwater conditions, where *fog* is perceived differently.
* **light_attenuation_coeffs**: Float Array. Array containing the attenuation coefficients for each channel in a BGR image. 
* **enable_white_balance**: Boolean. Apply white balance to the image.


## References

For research or when using this code, please consider citing the following papers:

```
- K. He, J. Sun and X. Tang, 
"Single Image Haze Removal Using Dark Channel Prior," 
in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 33, no. 12, pp. 2341-2353, Dec. 2011. 
doi: 10.1109/TPAMI.2010.168
- Tobias Doernbach, Arturo Gomez Chavez, Christian A. Mueller, Andreas Birk,
"High-Fidelity Deep-Sea Perception Using Simulation in the Loop",
in IFAC-PapersOnLine, Volume 51, Issue 29, 2018, Pages 32-37.
doi: 10.1016/j.ifacol.2018.09.465
```
