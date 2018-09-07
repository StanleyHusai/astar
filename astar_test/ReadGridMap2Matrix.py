import math
import csv
from numpy import matrix


Fcsv_GNSS = csv.reader(open('BinaryOccupancyGrid.csv',
                                        'r'))
grid_map = []
for rowCsv in Fcsv_GNSS:
	grid_map.append(rowCsv[0:])