# Magnetometer Caliberation
As a lot of people face mag caliberation difficult, the following files will help in viewing the results of the caliberation.
It provides a way to check if the caliberation is done properly or not.  

The file magCalibData(runs on rpi) sends data to MagCalibVizOnPC.ipynb(it runs on your PC).   
Run the fourth cell of ipython notebook before you run the magCalibData.py  

If you see a circle formed by mx-my, my-mz, mx-mz, then that shows that you have got a sphere centered at zero.   
This shows that your transformed mag values are not affected by hard iron and soft iron efftects.  
