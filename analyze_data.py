#!/usr/bin/python3

infile="data.csv"

import numpy as np

import csv
    

with open(infile, 'r') as f:
    data = list(csv.reader(f, delimiter=";"))

titles = data[0]

data = np.array(data[1:])

data = np.where(data[titles.index("Satellites")])

print(data)


