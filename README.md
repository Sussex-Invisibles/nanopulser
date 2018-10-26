# nanopulser

Assumes python version 2.7

## Quick run
To run simply:

```
sudo python test_run.py -n [number of pulses] -p [pulse height] -d [pulse delay (ms)]
```

Where:
* Number of pulses can be from 1 to 65025
* Pulse height is from 0 (min) to 16383 (max)
* Pulse delay is from 0.1 (min) to 256.02 (max)

For example:
sudo python test_run.py -n 10000 -p 16383 -d 2