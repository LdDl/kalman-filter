import matplotlib.pyplot as plt

mx, my = [], []
px, py = [], []
statex, statey = [], []

with open('./data/kalman-1d.csv', 'r') as f:
    next(f) # skip header
    for line in f:
        data_str = line.rstrip().split(';')
        data = [float(x) for x in data_str]

        mx.append(data[0])
        my.append(data[2])
        px.append(data[0])
        py.append(data[1])
        statex.append(data[0])
        statey.append(data[3])

dpi = 100
plt.figure(figsize=(720/dpi, 480/dpi), dpi=dpi)
plt.plot(mx, my, 'r', label='Measurement')
plt.plot(px, py, 'b', label='Perfect')
plt.plot(statex, statey, 'y', label='Prediction')
plt.legend(loc="upper left")
plt.grid(alpha=0.2)
ax = plt.gca()

plt.title("Kalman 1-D", fontsize=15, fontweight='bold')
plt.savefig('./data/kalman-1d.png', dpi=dpi)
# plt.show()