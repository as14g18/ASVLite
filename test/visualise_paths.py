import os
import sys

from math import sqrt
from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import pandas as pd
import seaborn as sns
import numpy as np

from sklearn.metrics import mean_squared_error
from numpy.polynomial.polynomial import polyfit


WAVE_HEIGHT = sys.argv[1]
HEADING = sys.argv[2]
DIRECTORIES = [
	f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-1',
	f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-2',
	f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-3',
	f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-4',
	f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-5',
	f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-6',
	f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-7',
	f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-8',
	f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-9',
	f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-10',
]

START_Y = 1000
GOAL_Y = 3000
X_CORRIDOR_THRESHOLD = 3
ASV_COUNT = 5


def calculate_max_speed(df, column, prev_column):
	"""Calculates the maximum speed of the ASVs in the column"""
	max_speed = 0
	if not prev_column:
		max_speed = 0
	else:
		for i in range(ASV_COUNT):
			cur_x = df[column][f'asv{i}_x']
			cur_y = df[column][f'asv{i}_y']

			if not pd.isnull(cur_y):
				cur_time = float(column)
				prev_x = df[prev_column][f'asv{i}_x']
				prev_y = df[prev_column][f'asv{i}_y']
				prev_time = float(prev_column)

				speed = sqrt((cur_x - prev_x)**2 + (cur_y - prev_y)**2) / (cur_time - prev_time)
				max_speed = max(max_speed, speed)

	return max_speed


history = {}


def calculate_vertical_mse(df, column):
	"""Calculates the RMSE of the ASVs compared to the predicted y values produced by np.polyfit"""
	global history
	x = []
	y = []
	for i in range(ASV_COUNT):
		cur_x = df[column][f'asv{i}_x']
		cur_y = df[column][f'asv{i}_y']
		if not pd.isnull(cur_x) and not pd.isnull(cur_y):
			x.append(cur_x)
			y.append(cur_y)
			history[i] = (cur_x, cur_y)
		else:
			x.append(history[i][0])
			y.append(history[i][1])

	x = np.array(x)
	y = np.array(y)

	if x.size > 0:
		b, m = polyfit(x, y, 1)
		pred_y = b + m * x
		mse = mean_squared_error(y, pred_y)
	else:
		mse = None

	return mse


def calculate_distance_sd(df, column):
	"""Calculates the maximum speed of the ASVs in the column"""
	coords = []
	for i in range(ASV_COUNT):
		cur_x = df[column][f'asv{i}_x']
		cur_y = df[column][f'asv{i}_y']
		if not pd.isnull(cur_x) and not pd.isnull(cur_y):
			coords.append(np.array((cur_x, cur_y)))

	distances = []
	for i in range(1, len(coords)):
		dist = np.linalg.norm(coords[i]-coords[i-1])
		distances.append(dist)

	sd = np.std(distances) if distances else None

	return sd if sd else None


def calculate_performance(data):
	"""Calculates swarm performance using a performance function"""
	df = pd.DataFrame(data)
	prev_column = None

	V = 0
	G = 0
	C = 0
	vcount = 0
	gcount = 0
	ccount = 0

	for column in df:
		v = calculate_max_speed(df, column, prev_column)
		g = calculate_vertical_mse(df, column)
		c = calculate_distance_sd(df, column)

		if v is not None:
			V += v
			vcount += 1
		if g is not None:
			G += g
			gcount += 1
		if c is not None:
			C += c
			ccount += 1

		prev_column = column

	V /= vcount
	G /= gcount
	C /= ccount

	print(f'V: {round(V, 2)} | C: {round(C, 2)} | G: {round(G, 2)}')

	return round((V * 1000) / (C * G), 2)


def generate_dataframe(directory, drop_y=False):
	"""Generates a pandas dataframe containing the needed information from the specified log file"""
	filenames = []
	for filename in os.listdir(directory):
		filenames.append(filename)

	filenames.sort(key=lambda x: '{0:0>8}'.format(x).lower())

	i = 0
	data = defaultdict(dict)
	for filename in filenames:
		item = {}

		df = pd.read_csv(os.path.join(directory, filename), delim_whitespace=True)
		df = df[df['sig_wave_ht(m)'] != 0]

		if drop_y:
			# Remove all entries that go beyond the waypoint y
			cols = ['cog_y(m)']
			df[cols] = df[df[cols] <= GOAL_Y][cols]
			df.dropna()

		for index, row in df.iterrows():
			data[row['time(sec)']][f'asv{i}_x'] = row['cog_x(m)']
			data[row['time(sec)']][f'asv{i}_y'] = row['cog_y(m)']

		i += 1

	return data


def show_animated_plot(data):
	"""Displays how the paths of ASVs progress as time goes on"""
	df = pd.DataFrame(data)

	x = []
	y = []

	plt.draw()

	S = 500
	skip = S
	for column in df:
		skip -= 1
		if skip == 0:
			skip = S
		else:
			continue
		for i in range(ASV_COUNT):
			cur_x = df[column][f'asv{i}_x']
			cur_y = df[column][f'asv{i}_y']
			x.append(cur_x)
			y.append(cur_y)

	plt.ion()
	animated_plot = plt.plot(x, y, 'ro')[0]

	for i in range(len(x)):
	    animated_plot.set_xdata(x[0:i])
	    animated_plot.set_ydata(y[0:i])
	    plt.draw()
	    plt.pause(0.01)


def show_plot(directory, drop_y=False):
	"""Displays the paths of the ASVs"""
	filenames = []
	for filename in os.listdir(directory):
		filenames.append(filename)

	filenames.sort()

	for filename in filenames:
		df = pd.read_csv(os.path.join(directory, filename), delim_whitespace=True)
		df = df[df['sig_wave_ht(m)'] != 0]
		df = df[['cog_x(m)', 'cog_y(m)', 'time(sec)']]

		if drop_y:
			# Remove all entries that go beyond the waypoint y
			cols = ['cog_y(m)']
			df[cols] = df[df[cols] <= GOAL_Y][cols]
			df.dropna()

		df.columns = ['x', 'y', 'time']

		values = df.values
		x = values[:,0]
		y = values[:,1]

		plt.scatter(x, y, label=filename[-1], linewidths=1)

	plt.show()


if __name__ == "__main__":
	directory = f'/home/akhi/Documents/p3project/ASVLite/build/run-{WAVE_HEIGHT}-{HEADING}-1'

	print(f'Swarm Performance: {calculate_performance(generate_dataframe(directory))}');
	show_animated_plot(generate_dataframe(directory))
	show_plot(directory)

	count = 0
	total = 0
	scores = []
	for directory in DIRECTORIES:
		s = calculate_performance(generate_dataframe(directory))
		scores.append(s)
		total += s
		count += 1

	print(f'Scores: {scores}')

	average = total / count
	print(f'Average: {round(average, 2)}')

	total2 = 0
	for s in scores:
		total2 += (s-average)**2

	print(f'Variance: {round(total2 / count, 2)}')
