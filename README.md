## About

* Forked from [github repository](https://github.com/joyeecheung/dark-channel-prior-dehazing)
* Improve readability and parameter testing through YAML config file.
* Add option to dehaze **underwater images** where *haze* or *fog* is perceived differently.
* Add option to use an already precomputed depth image to recover image original colors. This will effectively skip the DCP algorithm and use depth to compute *tranmission* directly. For more info please refer to these [slides](http://kaiminghe.com/cvpr09/cvpr09slides.pdf) or the articles mention in the reference section. 

## Dependencies

* The file **requirements.txt** contains the main dependencies to run the python scripts.
* When using *Anaconda* as a package manager, the file **dark-channel-prior-dehazing.yml** can be used to create a python environment with the same package versions as when tested. 
* If the scripts throw `AttributeError: __float__`, make sure your pillow has jpeg support e.g. try:

## Run Dark Channel Prior Dehazing (DCP)

1. Change the DCP parameter values in the config file `config/YOUR_CONFIG_FILE.yml` or create your own.
2. Enter the `src` directory, run `python main.py -config YOUR_CONFIG_FILE.yml`. 
3. It will use images under `imgs` directory as default to produce the results. The results will show up in `results` directory.

### Parmeters description

The parameters to be adjusted in the YAML file under the `/config` directory are the next ones:

* **input**: String with the image filename to be dehazed. It can contain a sequence of filenames separated by space.
* **depth_image**: String with the image filename containing depth information. If this used DCP is skipped and color is recovered based on depth. 
* **min_transmission**: Float. Minimum value for the trasmission rate; see paper [slides](http://kaiminghe.com/cvpr09/cvpr09slides.pdf).
* **max_atm_light**: Int. Maximum RGB value the atmospheric light can have in the image; see paper [slides](http://kaiminghe.com/cvpr09/cvpr09slides.pdf).
* **window**: Int. Window or kernel size used to compute the dark channel prior; see paper [slides](http://kaiminghe.com/cvpr09/cvpr09slides.pdf).
* **filter_radius**: Int. Kernel size used in the guided filter to smooth the transmission/depth estimation; see [explanation](http://kaiminghe.com/eccv10/).
* **underwater_dehaze**: Boolean. If true the algorithm changes the source image color space to dehaze based on underwater conditions, where *fog* is perceived differently.

## Naming convetion of the results

For input image `name.jpg` and based ont the previously described parameters, the naming convention is:

* dark channel: `name_result_MinTransmissionRate_MaxAtmLight_WindowSize_FilterRadius_dark-channel.png`
* raw transmission map: `name_result_MinTransmissionRate_MaxAtmLight_WindowSize_FilterRadius_raw-transmission.png`
* refined tranmission map: `name_result_MinTransmissionRate_MaxAtmLight_WindowSize_FilterRadius_refined-transmission.png`
* image dehazed with the raw transmission map: `name_result_MinTransmissionRate_MaxAtmLight_WindowSize_FilterRadius_raw-radiance.png`
* image dehazed with the refined transmission map: `name_result_MinTransmissionRate_MaxAtmLight_WindowSize_FilterRadius_refined-radiance.png`

## Directory structure

    .
	├─ README.md
	├─ requirements.txt
    ├─ dark-channel-prior-dehazing.yml
	├─ img (source images)
	│   └── ... (input images, source and depth)
	├─ result (the results)
    │   └── ...
	└─ src (the python source code)
        ├── dehaze.py (dehazing using the dark channel prior)
        ├── main.py (generate the results for the report)
        ├── guidedfilter.py (guided filter)

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
