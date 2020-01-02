import matplotlib.pyplot as plt
import numpy as np
from planning_utils import *




def plot_grid_path(grid, start, end, path, plt_name='', show_fig=False):
	plt.imshow(grid, cmap='Greys', origin='lower')

	plt.plot(start[1], start[0], 'x')
	plt.plot(end[1], end[0], 'x')

	pp = np.array(path)
	plt.plot(pp[:, 1], pp[:, 0], 'g', linewidth=0.5)
	plt.scatter(pp[:, 1], pp[:, 0],c='red', marker='X')

	plt.xlabel('east')
	plt.ylabel('north')

	if show_fig == True:
		plt.show()


	# Draw plts for report
	plt.savefig('figs/'+plt_name+'.png')

	plt.clf()


def plot_graph(grid, G,  nmin, emin, start=None, end=None, height=5, safety=5):

	
	plt.imshow(grid, cmap='Greys', origin='lower')


	#If you have a graph called "g" these plots should work
	#Draw edges
	for (n1, n2) in G.edges:
		plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black' , alpha=0.5)

	#Draw connected nodes in red
	for n1 in G.nodes:
		plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')

	if start != None and end != None:
		plt.scatter(start[0], start[1], c='blue', marker='x')
		plt.scatter(end[0], end[1], c='blue', marker='x')

	plt.show()
	plt.clf()


