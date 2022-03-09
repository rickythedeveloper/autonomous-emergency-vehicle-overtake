from ...algorithms.continuous.PDFs.PDF_V8 import *

traffic = [
	[2, 25],
	[6, 28],
	[10, 27],
	[10, 21],
	[12, 26],
	[13, 23],
	[14, 28],
	[1, 17],
	[11, 11],
	[13, 26]
]
emergency_vehicle_coords = [6, 17]
precision = 10

y = get_prob_from_angle(traffic, emergency_vehicle_coords, 19)
print(y)