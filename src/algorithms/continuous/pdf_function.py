from ...algorithms.continuous.PDFs.NewPDFs import *

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
	[13, 26],
	[-2, -5],
	[-1, -2]
]
emergency_vehicle_coords = [3, 12]
heading = 30
find_probability_for_this_angle = 45

test_prob = angle_probability_from_pdf(traffic, emergency_vehicle_coords, heading, find_probability_for_this_angle)
print(test_prob)
