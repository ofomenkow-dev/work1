"""Independent teaching model. Units: millimetres, radians, seconds."""
from dataclasses import asdict, dataclass, fields, replace
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
    mode: str = 'position'
    path: str = 'line'
    start_x: float = 500.0
    start_y: float = 0.0
    start_z: float = 180.0
    end_x: float = 350.0
    end_y: float = 200.0
    end_z: float = 260.0
    center_x: float = 380.0
    center_y: float = 0.0
    radius: float = 80.0

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
        if self.mode not in ('position', 'contour') or self.path not in ('line', 'circle'):
            raise ValueError('Неизвестный режим или траектория.')
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
        if not isinstance(data, dict) or set(data) != {f.name for f in fields(cls)}:
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
    """Analytical inverse kinematics with phi=0; tries both elbow branches."""
    x,y,z = p
    rs = ranges(c)
    def valid(q):
        return all(lo-1e-7 <= v <= hi+1e-7 for v,(lo,hi) in zip(q,rs))
    if c.kind == 'cartesian':
        q=(x,y,0,z)
        return q if valid(q) else None
    if c.kind == 'cylindrical':
        a=math.atan2(y,x)
        q=(a,math.hypot(x,y),-a,z)
        return q if valid(q) else None
    wx=x-c.l3
    co=(wx*wx+y*y-c.l1*c.l1-c.l2*c.l2)/(2*c.l1*c.l2)
    if abs(co)>1+1e-9:
        return None
    for sign in (1,-1):
        b=sign*math.acos(clamp(co,-1,1))
        a=math.atan2(y,wx)-math.atan2(c.l2*math.sin(b),c.l1+c.l2*math.cos(b))
        a=math.atan2(math.sin(a),math.cos(a))
        q=(a,b,-a-b,z)
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


def desired(c,t):
    s=(c.start_x,c.start_y,c.start_z)
    e=(c.end_x,c.end_y,c.end_z)
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
    refs=[]
    for i in range(n+1):
        t=min(i*c.dt,c.duration)
        p=desired(c,t)
        q=inverse(c,p)
        if q is None:
            raise ValueError(f'Траектория вне рабочей области при t = {t:.3f} с. Измените координаты или ограничения.')
        if refs and c.kind!='cartesian' and any(abs(q[j]-refs[-1][2][j])>math.pi for j in (0,2)):
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
        for j in range(4):
            angular=j<3 if c.kind=='scara' else j in ((0,2) if c.kind=='cylindrical' else (2,))
            vmax=math.radians(c.angular_speed) if angular else c.speed
            change=clamp((command[j]-feedback[j])*alpha,-vmax*h,vmax*h)
            actual[j]=clamp(actual[j]+change,*rs[j])
        if t+1e-10>=next_tick:
            delayed=max(0,t-c.cycles/c.frequency)
            idx=min(n,math.floor(delayed/c.dt+1e-9))
            command=quant(refs[idx][2])
            next_tick=(math.floor(t*c.frequency+1e-8)+1)/c.frequency
        measured=forward(c,actual)
        samples.append(Sample(t,p,measured,tuple(actual),math.dist(measured,p)))
    return Result(samples,max(s.error for s in samples),
                  math.sqrt(sum(s.error*s.error for s in samples)/len(samples)),
                  samples[-1].error,replace(c))
