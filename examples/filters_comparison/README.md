# Filter Comparision

## all_filters.py
This file(runs on rpi) caluclates orientation through various ways and sends it to the subscriber on your computer.

## filter comparision.ipynb
This python notebook running on your computer collects the data and shows graphs of all the filters together for a better 
visualization.

## pygame_viz.py
This file(running on your computer) captures the data from rpi and accordingly moves the box based on the orientation.

**Note:** uses ```pip install zmq``` for this code to work. Also make sure to give the ip address of rpi in the files that are 
run on your computer.
