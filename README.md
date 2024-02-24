This is a work in progress BMP423 accelerometer driver written in MicroPython.
The aim of this project is to explore the capabilities of the device, including
what is normally not available in simpler drivers: the feature engine especially
for step detection.

The datasheet was not really useful to be able to get the steps counter working,
so this driver makes use certain things that BOSCH does in their own public
driver in order to load the features configuration blob and things like that.

Right now **this is just a work in progress**, even if it works in the base
case and to enable the step counter. More work is needed in order to support
interrupts, to provide a cleaner API, documentation and a few examples.

## Usage

Transfer files to the device. Note that **:** at the end is needed, or
mpremote does a mess and will copy all the files to the last file name.

    mpremote bma423.py bma423config.bin :

Then upload an example as `main.py` like this:

    mpremote example_you_want.py :main.py

Then reset the device and look at the output in your serial log with:

    mpremote repl
