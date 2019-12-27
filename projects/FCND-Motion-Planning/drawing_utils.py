import matplotlib.pyplot as plt
import numpy as np




def plot_grid_path(grid, start, end, path, plt_name='', show_fig=False):
	plt.imshow(grid, cmap='Greys', origin='lower')

	plt.plot(start[1], start[0], 'x')
	plt.plot(end[1], end[0], 'x')


	pp = np.array(path)
	print(pp)

	plt.plot(pp[:, 1], pp[:, 0], 'g')
	#plt.scatter(pp[:, 1], pp[:, 0], '.')

	plt.xlabel('east')
	plt.ylabel('north')

	if show_fig == True:
		plt.show()


	# Draw plts for report
	plt.savefig('figs/'+plt_name+'.png')