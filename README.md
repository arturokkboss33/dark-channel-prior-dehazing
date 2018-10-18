##Dependencies

* The file **requirements.txt** contains the main dependencies to run the python scripts.
* When using *Anaconda* as a package manager, the file **dark-channel-prior-dehazing.yml** can be used to create a python environment with the same package versions as when tested. 
* If the scripts throw `AttributeError: __float__`, make sure your pillow has jpeg support e.g. try:

##Run Dark Channel Prior Dehazing (DCP)

1. Change the DCP parameter values in the config file `config/YOUR_CONFIG_FILE.yml` or create your own.
2. Enter the `src` directory, run `python main.py -config YOUR_CONFIG_FILE.yml`. 
3. It will use images under `imgs` directory as default to produce the results. The results will show up in `results` directory.



## Naming convetion of the results

For input image `name.jpg` using the default parameters, the naming convention is:

1. dark channel: `name-dark.jpg`
2. raw transmission map: `name-rawt.jpg`
3. refined tranmission map: `name-refinedt.jpg`
4. image dehazed with the raw transmission map: `name-radiance-rawt.jpg`
5. image dehazed with the refined transmission map: `name-radiance-refinedt.jpg`

If there are special configurations for the parameters, for example, , then the base name will be appended with `-20-170-50-40` e.g. the dark channel is `name-dark-20-170-50-40.jpg`

##Directory structure

    .
	├─ README.md
	├─ requirements.txt
	├─ doc
	│   └── report.pdf
	├─ img (source images)
	│   └── ... (input images from CVPR 09 supplementary materials)
	├─ result (the results)
    │   └── ...
	└─ src (the python source code)
        ├── dehaze.py (dehazing using the dark channel prior)
        ├── main.py (generate the results for the report)
        ├── guidedfilter.py (guided filter)
        └── util.py (utilities)

##About

* [Github repository](https://github.com/joyeecheung/dark-channel-prior-dehazing)
* Author: Qiuyi Zhang
* Time: Jan. 2015
