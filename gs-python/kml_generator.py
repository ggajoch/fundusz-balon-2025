#!/usr/bin/env python
# -*- coding: utf-8 -*-

# uruchamiać z argumentami: <ścieżka do pliku csv z 3 kolumnami lon;lat;alt> <kolor w hex> <p>

import sys
import csv
import simplekml

csv_path = sys.argv[1]
color = sys.argv[2]
output_path = sys.argv[3]

skml = simplekml.Kml()
ls = skml.newlinestring(name='FlightPath')
ls.extrude = 1
ls.altitudemode = simplekml.AltitudeMode.absolute
ls.style.linestyle.width = 5
ls.style.linestyle.color = color + 'ff'
ls.style.polystyle.color = color + '00'

coordinates = []

with open(csv_path, newline='') as csvfile:
    data = csv.reader(csvfile, delimiter=';', quotechar='|')
    for row in data:
        # print(row)
        (lon, lat, alt) = row
        coordinates.append((float(lon), float(lat), float(alt)))

ls.coords = coordinates
skml.save(output_path)

