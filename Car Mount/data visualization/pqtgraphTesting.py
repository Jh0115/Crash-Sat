#getting started with pyqt graphing module

import pyqtgraph as pg

data = [0,1,2,3,4,5,5,5,5,5,5]
data2 = [0,8,7,6,5,4,3,2,1]

pw = pg.plot(data, pen='r')  # plot x vs y in red
pw.plot(data2, pen='b')

