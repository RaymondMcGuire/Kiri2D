'''
Author: Xu.WANG
Date: 2021-09-04 18:07:03
LastEditTime: 2021-09-11 20:52:16
LastEditors: Xu.WANG
Description: 
FilePath: \Kiri2D\scripts\test\test.py
'''
# -*- coding: utf-8 -*-

import math
import time
import tkinter
import random

G = 9.80665  # 重力加速度


class Element(object):
    def __init__(self):
        self.n = 0  # 要素No.
        self.r = 0.0  # 半径
        self.x = 0.0  # X座標
        self.y = 0.0  # Y座標
        self.a = 0.0  # 角度
        self.dx = 0.0  # X方向増加量
        self.dy = 0.0  # Y方向増加量
        self.da = 0.0  # 角度増加量
        self.vx = 0.0  # X方向速度
        self.vy = 0.0  # Y方向速度
        self.va = 0.0  # 角速度
        self.fx = 0.0  # Ｘ方向節点力
        self.fy = 0.0  # Ｙ方向節点力
        self.fm = 0.0  # モーメント
        self.en = []  # 弾性力（直方向）
        self.es = []  # 弾性力（せん断方向）


class Particle(Element):
    def __init__(self, x, y, r=5.0):
        super(Particle, self).__init__()
        self.etype = 1  # 要素タイプ
        self.r = r  # 半径
        self.x = x  # X座標
        self.y = y  # Y座標
        # 粒子専用
        self.m = 0.0  # 質量
        self.Ir = 0.0  # 慣性モーメント


class Line(Element):
    def __init__(self, x1=0, y1=0, x2=0, y2=0):
        super(Line, self).__init__()
        self.etype = 2  # 要素タイプ

        # 線要素専用
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2


class Interface(object):
    def __init__(self, kn=0, ks=0, etan=0, etas=0, frc=0):
        self.kn = kn  # 弾性係数（法線方向）
        self.etan = etan  # 粘性係数（法線方向）
        self.ks = ks  # 弾性係数（せん断方向）
        self.etas = etas  # 粘性係数（せん断方向）
        self.frc = frc  # 摩擦係数


class InterfaceContainer():
    def __init__(self):
        self.data = {}

    def set(self, interface, etype1, etype2):
        if not etype1 in self.data:
            self.data[etype1] = {}
        if not etype2 in self.data:
            self.data[etype2] = {}
        self.data[etype1][etype2] = interface
        self.data[etype2][etype1] = interface

    def get(self, etype1, etype2):
        return self.data[etype1][etype2]


class DEM:

    def __init__(self):
        self.particles = []
        self.lines = []
        self.interfaces = InterfaceContainer()

        self.rho = 10  # 粒子間密度
        self.dt = 0.001  # ⊿ｔ
        self.step = 0  # ステップ
        self.area = [0 for i in range(4)]  # x_min,x_max,y_min,y_max

    def setup(self):
        # 境界（暫定処理）
        l1 = Line(self.area[0], self.area[2], self.area[0], self.area[3])
        l2 = Line(self.area[1], self.area[2], self.area[1], self.area[3])
        l3 = Line(self.area[0], self.area[2], self.area[1], self.area[2])
        l4 = Line(self.area[0], self.area[3], self.area[1], self.area[3])
        self.lines += [l1, l2, l3, l4]

        self.par_count = len(self.particles)
        self.line_count = len(self.lines)
        self.elem_count = self.par_count + self.line_count

        # 粒子要素
        pe = self.particles
        for i in range(self.par_count):
            pe[i].n = i
            pe[i].m = 4.0/3.0*math.pi*self.rho*pe[i].r**3.0  # 質量
            pe[i].Ir = math.pi*self.rho*pe[i].r**4.0/2.0  # 慣性モーメント
            pe[i].en = [0.0 for j in range(self.elem_count)]
            pe[i].es = [0.0 for j in range(self.elem_count)]

        # 線要素
        for i in range(self.line_count):
            self.lines[i].n = self.par_count + i

    def cycle(self, step=1):
        for i in range(step):
            self.next_step()

    def next_step(self):
        self.calc_force()
        self.update_coord()
        self.step += 1

    def calc_force(self):
        self._reset_force()
        self._particles_collision()
        self._par_and_line_collision()
        self._apply_gravitiy()

    def _reset_force(self):
        for p in self.particles:
            p.fx = 0
            p.fy = 0
            p.fm = 0

    def _particles_collision(self):
        for i in range(self.par_count):
            for j in range(i+1, self.par_count):
                p1 = self.particles[i]
                p2 = self.particles[j]
                lx = p1.x - p2.x
                ly = p1.y - p2.y
                ld = (lx**2+ly**2)**0.5
                if (p1.r+p2.r) > ld:  # 接触
                    cos_a = lx/ld
                    sin_a = ly/ld
                    self._force_par2par(p1, p2, cos_a, sin_a)
                else:
                    p1.en[p2.n] = 0.0
                    p1.es[p2.n] = 0.0

    def _par_and_line_collision(self):
        # 粒子と線の接触判定
        for i in range(self.par_count):
            for j in range(self.line_count):
                hit = False
                p = self.particles[i]
                l = self.lines[j]
                th0 = math.atan2(l.y2-l.y1, l.x2-l.x1)
                th1 = math.atan2(p.y-l.y1, p.x-l.x1)
                a = math.sqrt((p.x-l.x1)**2+(p.y-l.y1)**2)
                d = abs(a*math.sin(th1-th0))
                if d < p.r:
                    b = math.sqrt((p.x-l.x2)**2+(p.y-l.y2)**2)
                    s = math.sqrt((l.x2-l.x1)**2+(l.y2-l.y1)**2)
                    if a < s and b < s:
                        s1 = math.sqrt(a**2-d**2)
                        x = l.x1 + s1*math.cos(th0)
                        y = l.y1 + s1*math.sin(th0)
                        hit = True
                    elif a < b and a < p.r:
                        x = l.x1
                        y = l.y1
                        hit = True
                    elif b < p.r:
                        x = l.x2
                        y = l.y2
                        hit = True
                if hit:
                    lx = x - p.x
                    ly = y - p.y
                    ld = math.sqrt(lx**2+ly**2)
                    cos_a = lx/ld
                    sin_a = ly/ld
                    self._force_par2par(p, l, cos_a, sin_a)
                else:
                    p.en[l.n] = 0.0
                    p.es[l.n] = 0.0

    def _apply_gravitiy(self):
        for p in self.particles:
            p.fy += -G*p.m  # 重力

    def _force_par2par(self, p1, p2, cos_a, sin_a):

        # 相対的変位増分
        un = +(p1.dx-p2.dx)*cos_a+(p1.dy-p2.dy)*sin_a
        us = -(p1.dx-p2.dx)*sin_a+(p1.dy-p2.dy)*cos_a+(p1.r*p1.da+p2.r*p2.da)
        # 相対的速度増分
        vn = +(p1.vx-p2.vx)*cos_a+(p1.vy-p2.vy)*sin_a
        vs = -(p1.vx-p2.vx)*sin_a+(p1.vy-p2.vy)*cos_a+(p1.r*p1.va+p2.r*p2.va)

        inf = self.interfaces.get(p1.etype, p2.etype)

        # 合力
        p1.en[p2.n] += inf.kn*un
        p1.es[p2.n] += inf.ks*us
        hn = p1.en[p2.n] + inf.etan*vn
        hs = p1.es[p2.n] + inf.etas*vs

        if hn <= 0.0:
            # 法線力がなければ、せん断力は０
            hs = 0.0
        elif abs(hs)-inf.frc*hn >= 0.0:
            # 摩擦力以上のせん断力は働かない
            hs = inf.frc*abs(hn)*hs/abs(hs)
        # 合力
        p1.fx += -hn*cos_a + hs*sin_a
        p1.fy += -hn*sin_a - hs*cos_a
        p1.fm -= p1.r*hs
        p2.fx += hn*cos_a - hs*sin_a
        p2.fy += hn*sin_a + hs*cos_a
        p2.fm -= p2.r*hs

    def update_coord(self):
        pe = self.particles
        for i in range(self.par_count):
            # 位置更新（オイラー差分）
            ax = pe[i].fx/pe[i].m
            ay = pe[i].fy/pe[i].m
            aa = pe[i].fm/pe[i].Ir
            pe[i].vx += ax*self.dt
            pe[i].vy += ay*self.dt
            pe[i].va += aa*self.dt
            pe[i].dx = pe[i].vx*self.dt
            pe[i].dy = pe[i].vy*self.dt
            pe[i].da = pe[i].va*self.dt
            pe[i].x += pe[i].dx
            pe[i].y += pe[i].dy
            pe[i].a += pe[i].da


class Window(tkinter.Tk):
    dem = DEM

    def __init__(self, dem):
        tkinter.Tk.__init__(self)
        self.dem = dem
        self.loop_n = 0

        self.scale = 1.0
        dem_width = self.dem.area[1] - self.dem.area[0]
        dem_height = self.dem.area[3] - self.dem.area[2]
        self.width = dem_width + 10
        self.height = dem_height + 10
        self.offset_x = (dem_width - self.width)/2
        self.offset_y = (dem_height - self.height)/2

        self.canvas = tkinter.Canvas(self, bg="white")
        self.canvas.pack(fill=tkinter.BOTH, expand=True)
        self.geometry('%dx%d' % (self.width, self.height))
        self.title('DEM')

        for l in self.dem.lines:
            xy = self.view_pos([l.x1, l.y1, l.x2, l.y2])
            self.canvas.create_line(xy, width=1)

        self.redraw()
        self.update_idletasks()

    def calcloop(self):
        if self.loop_n % 5 == 0:
            print('Step %d' % self.dem.step)
        if self.loop_n % 1 == 0:
            self.redraw()
        if self.loop_n >= 1000:
            print('解析終了.設定最大ループに達しました')
        else:
            self.after(0, self.calcloop)
        self.dem.cycle(10)
        self.loop_n += 1
        self.update_idletasks()

    def redraw(self):
        self.canvas.delete('par')
        h = self.height
        for p in self.dem.particles:
            x1, y1 = self.view_pos([p.x-p.r, p.y-p.r])
            x2, y2 = self.view_pos([p.x+p.r, p.y+p.r])
            self.canvas.create_oval(x1, y1, x2, y2, tags='par')

            x1, y1 = self.view_pos([p.x, p.y])
            x2, y2 = self.view_pos([p.x+p.r*math.cos(p.a),
                                    p.y+p.r*math.sin(p.a)])
            self.canvas.create_line(x1, y1, x2, y2, tags='par')

    def view_pos(self, coords, offset=(0, 0)):
        s = self.scale  # 表示倍率
        h = self.height  # 表示画面高さ
        w = self.width  # 表示画面幅
        xy_list = []
        for i in range(0, len(coords), 2):
            x = round(s*coords[i]) - self.offset_x
            y = round(h-s*coords[i+1]) + self.offset_y
            x = x + offset[0]
            y = y + offset[1]
            xy_list.append(x)
            xy_list.append(y)
        return xy_list


def main():
    print('初期設定中...')
    dem = DEM()
    dem.area = [0, 300, 0, 200]
    dem.dt = 0.01
    jitter = 0.001

    # p1 = Particle(250, 190, 10)
    # p2 = Particle(190, 190, 10)
    # p3 = Particle(120, 190, 10)
    # p4 = Particle(60, 190, 10)
    # dem.particles = [p1, p2, p3, p4]

    radius = 5
    dem.particles = []
    for idx in range(10):
        for idy in range(10):
            posx = 60 + idx*radius*2 + random.random()*jitter
            posy = 20 + idy*radius*2 + random.random()*jitter
            p = Particle(posx, posy, radius)
            dem.particles.append(p)

    l1 = Line(50, 140, 300, 170)
    l2 = Line(0, 110, 170, 80)
    l3 = Line(30, 30, 300, 50)
    dem.lines = []
    #dem.lines = [l1, l2, l3]

    # 粒子同士
    inf1 = Interface()
    inf1.kn = 100000  # 弾性係数（法線方向）
    inf1.etan = 50000  # 粘性係数（法線方向）
    inf1.ks = 5000  # 弾性係数（せん断方向）
    inf1.etas = 1000  # 粘性係数（せん断方向）
    inf1.frc = 100  # 摩擦係数
    # 粒子と線要素
    inf2 = Interface()
    inf2.kn = 5000000
    inf2.etan = 50000
    inf2.ks = 1000
    inf2.etas = 900
    inf2.frc = 10

    dem.interfaces.set(inf1, 1, 1)
    dem.interfaces.set(inf2, 1, 2)

    dem.setup()

    print('完了')
    print('粒子要素数： %d ' % dem.par_count)
    print('解析開始')

    w = Window(dem)
    w.after(0, w.calcloop)
    w.mainloop()


if __name__ == '__main__':
    main()
