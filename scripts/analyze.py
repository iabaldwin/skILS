#!/usr/bin/env python3

import os
import json
import imageio
from matplotlib import pyplot

def analyze(path:str):
    contents = sorted([f for f in os.listdir(path)])
    jsons = [f for f in contents if f.endswith('.json')]
    images = [f for f in contents if f.endswith('.png')]
    for (j, i) in zip(jsons, images):
        img = imageio.imread(os.path.join(path, i))
        with open(os.path.join(path, j)) as handle:
            data = json.load(handle)
       
        points = data['points']
        pyplot.imshow(img)
        for i in range(1, len(points)):
            print (points[i])
            pyplot.plot([points[i-1]['x'], points[i]['x']], 
                        [points[i-1]['y'], points[i]['y']])
        pyplot.show()
        
if __name__ == "__main__":
    analyze(os.path.abspath('../build/data/'))
    
