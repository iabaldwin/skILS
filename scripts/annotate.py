#!/usr/bin/env python3

import argparse
import os
from collections import deque
from functools import partial
from matplotlib import pyplot

import imageio 
import json

def annotate(image:str):
    assert os.path.exists(image)
    data = imageio.imread(image)
    
    coordinates = deque()

    def onclick(event, coordinates:deque):
        x, y = event.xdata, event.ydata
        if (len(coordinates) >= 4):
            coordinates.popleft()
        coordinates.append((x,y))
        print (coordinates)
    fig = pyplot.figure()
    cid = fig.canvas.mpl_connect('button_press_event', partial(onclick, coordinates=coordinates))
    fig.gca().imshow(data)
    pyplot.show()
    
    with open('coordinates.json', 'w') as handle:
        json.dump(list(coordinates), handle)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True)
    annotate(**vars(parser.parse_args()))
