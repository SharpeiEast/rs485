'''
动态折线图演示示例
'''
import random
import numpy as np
import matplotlib.pyplot as plt



class dynamic_figure():
    def __init__(self,xlim, l_ylim, h_ylim):
        self.xlim = xlim
        self.l_ylim = l_ylim
        self.h_ylim = h_ylim
        plt.ion()
        self.fig = plt.figure(1,figsize=(18,8),facecolor='white')
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(ls='--')
        self.ax.set_xlim(0,self.xlim)
        self.ax.set_ylim(self.l_ylim, self.h_ylim)
        self.f1text = self.ax.text(5, self.h_ylim - 5, '', fontsize=12)
        self.f2text = self.ax.text(5, self.h_ylim - 10, '', fontsize=12)
        self.f3text = self.ax.text(5, self.h_ylim - 15, '', fontsize=12)
        self.f4text = self.ax.text(5, self.h_ylim - 20, '', fontsize=12)
        self.x = []
        self.y1 = []
        self.y2 = []
    def update(self,newx,newy1,newy2):
        xmin, xmax = self.ax.get_xlim()
        self.x.append(newx)
        self.y1.append(newy1)
        self.y2.append(newy2)

        if xmax - t < 0:
            self.ax.set_xlim(xmax, xmax + self.xlim)
            self.f1text = self.ax.text(xmax + 5, self.h_ylim - 5, '', fontsize=12)
            self.f2text = self.ax.text(xmax + 5, self.h_ylim - 10, '', fontsize=12)
            self.f3text = self.ax.text(xmax + 5, self.h_ylim - 15, '', fontsize=12)
            self.f4text = self.ax.text(xmax + 5, self.h_ylim - 20, '', fontsize=12)
        self.ax.plot(self.x, self.y1, c='r', ls='-', marker='o', mec='b', mfc='w')  ## 保存历史数据
        self.ax.plot(self.x, self.y2, c='b', ls='--', marker='o', mec='b', mfc='w')  ## 保存历史数据
            # plt.plot(t, np.sin(t), 'o')
        self.f1text.set_text('T1=%.3f' % random.randint(1, 4))
        self.f2text.set_text('T2=%.3f' % random.randint(4, 7))
        self.f3text.set_text('TS=%.3f' % random.randint(7, 10))
        self.f4text.set_text('TL=%.3f' % random.randint(10, 13))
        plt.pause(0.1)

t = 0
d_F = dynamic_figure(80,-50,120)
while True:
    t += np.pi / 4
    y1 = np.sin(t)
    y2 = np.sin(t) + 0.2
    d_F.update(t, y1, y2)








