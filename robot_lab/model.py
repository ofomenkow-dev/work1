"""Independent teaching model. Units: millimetres, radians, seconds."""
from dataclasses import asdict, dataclass, field, fields
from copy import deepcopy
import csv
import json
import math
from pathlib import Path


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class Config:
    version: int = 1
    kind: str = 'scara'
    l1: float = 350.0
    l2: float = 250.0
    l3: float = 60.0
    q1min: float = -150.0
    q1max: float = 150.0
    q2min: float = -150.0
    q2max: float = 150.0
    rmin: float = 120.0
    rmax: float = 600.0
    xmax: float = 650.0
    ymax: float = 500.0
    zmax: float = 400.0
    kp: float = 8.0
    tau: float = 0.12
    dt: float = 0.01
    bits: int = 16
    frequency: float = 100.0
    cycles: int = 1
    speed: float = 120.0
    angular_speed: float = 180.0
    duration: float = 4.0
    mode: str = 'contour'
    path: str = 'table'
    start_x: float = 500.0
    start_y: float = 0.0
    start_z: float = 180.0
    end_x: float = 350.0
    end_y: float = 200.0
    end_z: float = 180.0  # Retained for old project files; plane height is start_z.
    center_x: float = 380.0
    center_y: float = 0.0
    radius: float = 80.0
    orientation: float = 0.0
    coordinate_mode: str = 'cartesian'
    points: list = field(default_factory=lambda: [[280.,100.],[320.,180.],
        [360.,100.],[400.,180.],[440.,100.],[480.,180.],[520.,100.]])

    def validate(self):
        for f in fields(self):
            v = getattr(self, f.name)
            if f.type in (float, int):
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                    raise ValueError(f'Некорректное число: {f.name}')
                if f.type is int and not isinstance(v, int):
                    raise ValueError(f'Требуется целое число: {f.name}')
        if self.version != 1:
            raise ValueError('Поддерживается формат проекта версии 1.')
        if self.kind not in ('scara', 'cartesian', 'cylindrical'):
            raise ValueError('Неизвестная кинематическая схема.')
        if self.mode not in ('position', 'contour') or self.path not in ('line', 'circle', 'table'):
            raise ValueError('Неизвестный режим или траектория.')
        if self.mode=='position' and self.path=='circle':
            raise ValueError('Окружность задаётся в контурном режиме; для позиционного используйте таблицу или отрезок.')
        if self.coordinate_mode not in ('cartesian', 'joint'):
            raise ValueError('Неизвестный формат пар координат.')
        if not -180 <= self.orientation <= 180:
            raise ValueError('Ориентация схвата: от −180° до 180°.')
        if not 0 <= self.start_z <= self.zmax:
            raise ValueError('Высота горизонтальной плоскости должна быть в пределах хода Z.')
        if not isinstance(self.points, list) or not 2 <= len(self.points) <= 200:
            raise ValueError('Таблица должна содержать от 2 до 200 пар.')
        for row in self.points:
            if not isinstance(row, (list, tuple)) or len(row) != 2 or any(
                isinstance(v,bool) or not isinstance(v,(float,int)) or
                not math.isfinite(v) or abs(v)>20000 for v in row):
                raise ValueError('В каждой строке таблицы должны быть два конечных числа (модуль ≤ 20000).')
        for k in ('l1','l2','rmax','xmax','ymax','zmax','kp','tau','dt','frequency',
                  'speed','angular_speed','duration','radius'):
            if getattr(self, k) <= 0:
                raise ValueError(f'{k}: значение должно быть положительным.')
        if self.l3 < 0 or not 0 <= self.rmin < self.rmax:
            raise ValueError('Проверьте длину инструмента и радиусы.')
        for lo, hi in ((self.q1min, self.q1max), (self.q2min, self.q2max)):
            if not -180 <= lo < hi <= 180:
                raise ValueError('Углы: −180° ≤ min < max ≤ 180°.')
        if not 4 <= self.bits <= 24 or not 0 <= self.cycles <= 100:
            raise ValueError('Разрядность: 4–24 бит; задержка: 0–100 тактов.')
        if not .001 <= self.dt <= .1 or not 2*self.dt <= self.duration <= 60:
            raise ValueError('Шаг: 0,001–0,1 с; время: от двух шагов до 60 с.')
        if not 1 <= self.frequency <= 10000 or not .001 <= self.tau <= 100 or self.kp > 1000:
            raise ValueError('Частота: 1–10000 Гц; τ: 0,001–100 с; Kp ≤ 1000.')
        for k in ('l1','l2','l3','rmax','xmax','ymax','zmax','speed','angular_speed','radius'):
            if getattr(self,k) > 10000:
                raise ValueError(f'{k}: максимум 10000.')
        for k in ('start_x','start_y','start_z','end_x','end_y','end_z','center_x','center_y'):
            if abs(getattr(self,k)) > 20000:
                raise ValueError(f'{k}: модуль координаты не более 20000 мм.')
        return self

    def save(self, path):
        self.validate()
        Path(path).write_text(json.dumps(asdict(self), ensure_ascii=False, indent=2), encoding='utf-8')

    @classmethod
    def load(cls, path):
        p = Path(path)
        if p.stat().st_size > 100000:
            raise ValueError('Файл проекта слишком большой.')
        data = json.loads(p.read_text(encoding='utf-8'))
        names = {f.name for f in fields(cls)}
        added = {'orientation', 'coordinate_mode', 'points'}
        if not isinstance(data, dict) or not names-added <= set(data) <= names:
            raise ValueError('Неверная структура проекта ROBOT Lab JSON.')
        return cls(**data).validate()


def ranges(c):
    r = math.radians
    if c.kind == 'cartesian':
        return [(0,c.xmax),(0,c.ymax),(-math.pi,math.pi),(0,c.zmax)]
    if c.kind == 'cylindrical':
        return [(r(c.q1min),r(c.q1max)),(c.rmin,c.rmax),(-math.pi,math.pi),(0,c.zmax)]
    return [(r(c.q1min),r(c.q1max)),(r(c.q2min),r(c.q2max)),(-2*math.pi,2*math.pi),(0,c.zmax)]


def forward(c, q):
    a,b,d,z = q
    if c.kind == 'cartesian':
        return (a,b,z)
    if c.kind == 'cylindrical':
        return (b*math.cos(a),b*math.sin(a),z)
    return (c.l1*math.cos(a)+c.l2*math.cos(a+b)+c.l3*math.cos(a+b+d),
            c.l1*math.sin(a)+c.l2*math.sin(a+b)+c.l3*math.sin(a+b+d),z)


def inverse(c, p):
    """Analytical inverse kinematics with horizontal tool yaw."""
    x,y,z = p
    phi = math.radians(c.orientation)
    rs = ranges(c)
    def valid(q):
        return all(lo-1e-7 <= v <= hi+1e-7 for v,(lo,hi) in zip(q,rs))
    if c.kind == 'cartesian':
        q=(x,y,phi,z)
        return q if valid(q) else None
    if c.kind == 'cylindrical':
        a=math.atan2(y,x)
        q=(a,math.hypot(x,y),phi-a,z)
        return q if valid(q) else None
    wx=x-c.l3*math.cos(phi)
    wy=y-c.l3*math.sin(phi)
    co=(wx*wx+wy*wy-c.l1*c.l1-c.l2*c.l2)/(2*c.l1*c.l2)
    if abs(co)>1+1e-9:
        return None
    for sign in (1,-1):
        b=sign*math.acos(clamp(co,-1,1))
        a=math.atan2(wy,wx)-math.atan2(c.l2*math.sin(b),c.l1+c.l2*math.cos(b))
        a=math.atan2(math.sin(a),math.cos(a))
        q=(a,b,phi-a-b,z)
        if valid(q):
            return q
    return None


def workspace(c, n=24):
    """Sample cloud at lower/upper Z slices, not a certified boundary."""
    rs=ranges(c)
    pts=[]
    for i in range(n+1):
        for j in range(n+1):
            wrist=12 if c.kind=='scara' else 1
            for k in range(wrist+1):
                q=[rs[0][0]+i/n*(rs[0][1]-rs[0][0]),
                   rs[1][0]+j/n*(rs[1][1]-rs[1][0]),
                   rs[2][0]+k/wrist*(rs[2][1]-rs[2][0]),0]
                p=forward(c,q)
                pts.extend((p,(p[0],p[1],c.zmax)))
    return pts


def tool_angle(c,q):
    if c.kind == 'scara':
        return q[0]+q[1]+q[2]
    return q[0]+q[2] if c.kind == 'cylindrical' else q[2]


def joint_pair(c,pair):
    """Input pair units: SCARA deg/deg; cylinder deg/mm; Cartesian mm/mm."""
    a,b=pair
    phi=math.radians(c.orientation)
    if c.kind=='scara':
        a,b=math.radians(a),math.radians(b)
        q=(a,b,phi-a-b,c.start_z)
    elif c.kind=='cylindrical':
        a=math.radians(a)
        q=(a,b,phi-a,c.start_z)
    else:
        q=(a,b,phi,c.start_z)
    if not all(lo-1e-7<=v<=hi+1e-7 for v,(lo,hi) in zip(q,ranges(c))):
        raise ValueError('Пара суставных координат выходит за ограничения робота.')
    return q


def table_pair(c,t):
    phase=clamp(t/c.duration,0,1)*(len(c.points)-1)
    if c.mode=='position':
        # Step to next point immediately after each equal-duration boundary.
        index=min(len(c.points)-1,math.ceil(phase-1e-10))
        return c.points[index]
    i=min(len(c.points)-2,math.floor(phase))
    u=phase-i
    return [a+(b-a)*u for a,b in zip(c.points[i],c.points[i+1])]


def desired(c,t):
    s=(c.start_x,c.start_y,c.start_z)
    e=(c.end_x,c.end_y,c.start_z)
    if c.path=='table':
        if c.duration/(len(c.points)-1) < max(c.dt,1/c.frequency)-1e-10:
            raise ValueError('Слишком мало времени между строками: увеличьте длительность или частоту расчёта/управления.')
        pair=table_pair(c,t)
        if c.coordinate_mode=='joint':
            return forward(c,joint_pair(c,pair))
        return (*pair,c.start_z)
    if c.mode=='position':
        return s if t==0 else e
    f=clamp(t/c.duration,0,1)
    if c.path=='circle':
        return (c.center_x+c.radius*math.cos(2*math.pi*f),
                c.center_y+c.radius*math.sin(2*math.pi*f),c.start_z)
    return tuple(a+(b-a)*f for a,b in zip(s,e))


@dataclass
class Sample:
    t: float
    desired: tuple
    actual: tuple
    q: tuple
    error: float


@dataclass
class Result:
    samples: list
    maximum: float
    rms: float
    final: float
    config: Config

    def export_csv(self,path):
        with open(path,'w',newline='',encoding='utf-8-sig') as f:
            w=csv.writer(f)
            w.writerow(['t_s','ref_x_mm','ref_y_mm','ref_z_mm',
                        'actual_x_mm','actual_y_mm','actual_z_mm','error_mm'])
            for s in self.samples:
                w.writerow([s.t,*s.desired,*s.actual,s.error])


def simulate(c):
    """Discrete first-order P-servo, bounded velocities, quantization, delay."""
    c.validate()
    if c.dt*c.frequency > 1+1e-9:
        raise ValueError('Шаг моделирования должен быть ≤ периода контроллера (1 / частота).')
    n=math.ceil(c.duration/c.dt)
    if c.path=='table':
        for i,pair in enumerate(c.points):
            if c.coordinate_mode=='joint':
                joint_pair(c,pair)
            elif inverse(c,(*pair,c.start_z)) is None:
                raise ValueError(f'Строка {i+1} таблицы вне рабочей области.')
    refs=[]
    for i in range(n+1):
        t=min(i*c.dt,c.duration)
        p=desired(c,t)
        q=(joint_pair(c,table_pair(c,t)) if c.path=='table' and
           c.coordinate_mode=='joint' else inverse(c,p))
        if q is None:
            raise ValueError(f'Траектория вне рабочей области при t = {t:.3f} с. Измените координаты или ограничения.')
        if refs and c.kind!='cartesian' and c.mode=='contour' and not (
            c.path=='table' and c.coordinate_mode=='joint') and any(
                abs(q[j]-refs[-1][2][j])>math.pi for j in (0,2)):
            raise ValueError('Траектория пересекает разрыв угловой координаты. Измените траекторию.')
        refs.append((t,p,q))
    actual=list(refs[0][2])
    command=actual[:]
    rs=ranges(c)
    def quant(q):
        out=[]
        for v,(lo,hi) in zip(q,rs):
            step=(hi-lo)/(2**c.bits-1)
            out.append(clamp(lo+math.floor((v-lo)/step+.5)*step,lo,hi))
        return out
    samples=[]
    next_tick=0.0
    for i,(t,p,_) in enumerate(refs):
        h=t-refs[i-1][0] if i else 0
        feedback=quant(actual)
        alpha=-math.expm1(-c.kp*h/c.tau)
        for j in range(3):
            angular=j<3 if c.kind=='scara' else j in ((0,2) if c.kind=='cylindrical' else (2,))
            vmax=math.radians(c.angular_speed) if angular else c.speed
            change=clamp((command[j]-feedback[j])*alpha,-vmax*h,vmax*h)
            actual[j]=clamp(actual[j]+change,*rs[j])
        actual[3]=c.start_z  # Drawing plane is mechanically fixed, not a Z servo.
        if t+1e-10>=next_tick:
            delayed=max(0,t-c.cycles/c.frequency)
            idx=min(n,math.floor(delayed/c.dt+1e-9))
            command=quant(refs[idx][2])
            next_tick=(math.floor(t*c.frequency+1e-8)+1)/c.frequency
        measured=forward(c,actual)
        samples.append(Sample(t,p,measured,tuple(actual),math.dist(measured,p)))
    return Result(samples,max(s.error for s in samples),
                  math.sqrt(sum(s.error*s.error for s in samples)/len(samples)),
                  samples[-1].error,deepcopy(c))
