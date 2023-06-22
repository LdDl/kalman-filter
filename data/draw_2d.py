import matplotlib.pyplot as plt

mx, my = [], []
px, py = [], []
zx, zy = [], []

with open('./data/kalman-2d.csv', 'r') as f:
    next(f) # skip header
    for line in f:
        data_str = line.rstrip().split(';')
        data = [float(x) for x in data_str]
        mx.append(data[0])
        my.append(data[1])
        px.append(data[2])
        py.append(data[3])
        zx.append(data[4])
        zy.append(data[5])

dpi = 100
plt.figure(figsize=(720/dpi, 480/dpi), dpi=dpi)
plt.plot(mx, my, 'r', label='Measurement')
plt.plot(px, py, 'g', label='Prediction')
plt.plot(zx, zy, 'y', label='Updated state')
plt.legend(loc="upper left")
plt.grid(alpha=0.2)
plt.xlim([250, 350])
ax = plt.gca()
ax.invert_yaxis()

plt.title("Kalman 2-D", fontsize=15, fontweight='bold')
plt.savefig('./data/kalman-2d.png', dpi=dpi)
# plt.show()