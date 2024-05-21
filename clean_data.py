#!/usr/bin/python3

import re

infile = "logs.txt"

time = re.compile("Time: (.*)$")
date = re.compile("Date: (.*)$")
fix = re.compile("Fix: (.*)$")
quality = re.compile("quality: (.*)$")
Location = re.compile("Location: (.*), (.*)$")
Location_in_degrees = re.compile("Location \(in degrees, works with Google Maps\): (.*), (.*)$")
Speed_knots = re.compile("Speed \(knots\): (.*)$")
Angle = re.compile("Angle: (.*)$")
Altitude = re.compile("Altitude: (.*)$")
Satellites = re.compile("Satellites: (.*)$")
Status_Message = re.compile("Status Message: (.*) ;$")
Packet_Number = re.compile("Packet Number: (.*) ;$")
Data = re.compile("Data: (.*) ;$")
RSSI = re.compile("RSSI: (.*) ;$")
Address = re.compile("Address: (.*) ;$")
SNR = re.compile("SNR: (.*) ;$")

expressions = [(True, time, ["time"]) ,
               (True, date, ["date"]) ,
               (True, fix, ["fix"]) ,
               (True, quality, ["quality"]) ,
               (True, Location, ["Longitude", "Lattitude"]) ,
               (True, Location_in_degrees, ["Longitude_in_degrees", "Lattitude_in_degrees"]) ,
               (True, Speed_knots, ["Speed_knots"]) ,
               (True, Angle, ["Angle"]) ,
               (True, Altitude, ["Altitude"]) ,
               (True, Satellites, ["Satellites"]) ,
               (True, Status_Message, ["Status_Message"]) ,
               (True, Packet_Number, ["Packet_Number"]) ,
               (True, Data, ["Data"]) ,
               (True, RSSI, ["RSSI"]) ,
               (True, Address, ["Address"]) ,
               (True, SNR, ["SNR"])]

length = len(expressions)

def treat_line (expression, line, i):
    to_treat, regex, _ = expression
    if to_treat:
        matches = regex.search(line)
        if (matches!= None):
            groups = list(matches.groups())
            for j in range(len(groups)):
                print(groups[j], end="")
                if (i != length - 1 or j != len(groups) - 1):
                    print(";", end="")

for i in range(length):
    to_treat, _, title = expressions[i]
    if to_treat:
        for j in range(len(title)):
            print(title[j], end="")
            if i != length-1 or j != len(title) -1 :
                print(";", end="")

print("")

with open(infile) as reader:
    line = reader.readline()
    while line != '':
        for i in range(length):
            expression = expressions[i]
            treat_line(expression, line, i)
            line = reader.readline()

        print("")
        _ = reader.readline()
        _ = reader.readline()
        line = reader.readline()
