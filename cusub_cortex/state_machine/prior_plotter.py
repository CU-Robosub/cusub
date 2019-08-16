import matplotlib.pyplot as plt
import yaml

config_file = "config/mission_config_C_WINNING.yaml"
x = []
y = []
with open(config_file,'r') as stream:
    try:
        conf_dict = yaml.safe_load(stream)
    except YAMLError as exc:
        print(exc)


fig, ax = plt.subplots()

colors=['r','b','g','m','c']
names=['startgate', 'jiangshi', 'triangle', 'droppers', 'octagon']

sg_pri = conf_dict["tasks"]["start_gate"]["prior"]
x.append(sg_pri[0])
y.append(sg_pri[1])

ji_pri = conf_dict["tasks"]["jiangshi"]["prior"]
x.append(ji_pri[0])
y.append(ji_pri[1])
tri_pri = conf_dict["tasks"]["triangle"]["prior"]
x.append(tri_pri[0])
y.append(tri_pri[1])
drop_pri = conf_dict["tasks"]["dropper"]["prior"]
x.append(drop_pri[0])
y.append(drop_pri[1])
oct_pri = conf_dict["tasks"]["octagon"]["prior"]
x.append(oct_pri[0])
y.append(oct_pri[1])

ax.scatter(x,y, c=colors)
offset=.3
for i, txt in enumerate(names):
    ax.annotate(txt, (x[i], y[i]), xytext=(x[i]-offset, y[i]+offset))
ax.set_xlim(-35,0)
ax.set_ylim(0,30)
ax.set_xlabel('x')
ax.set_ylabel('y')
#
fig2,ax2=plt.subplots()
ax2.scatter(y,x,c=colors)
ax2.set_xlim(0,30)
ax2.set_ylim(-35,0)

plt.show()
