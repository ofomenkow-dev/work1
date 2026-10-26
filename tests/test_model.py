import csv
import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from robot_lab.model import Config, desired, forward, inverse, simulate, workspace


class Kinematics(unittest.TestCase):
    def test_known_scara_pose(self):
        p=forward(Config(),(0,math.pi/2,-math.pi/2,100))
        for a,b in zip(p,(410,250,100)):self.assertAlmostEqual(a,b)

    def test_roundtrip(self):
        for kind in ('scara','cartesian','cylindrical'):
            c=Config(kind=kind)
            for p in ((350,200,260),(500,0,180),(400,100,0)):
                q=inverse(c,p)
                self.assertIsNotNone(q)
                self.assertLess(math.dist(p,forward(c,q)),1e-8)

    def test_limits(self):
        for kind in ('scara','cartesian','cylindrical'):
            c=Config(kind=kind)
            for p in ((20000,0,0),(350,200,-1),(350,200,401)):
                self.assertIsNone(inverse(c,p))
        self.assertIsNone(inverse(Config(q1min=90,q1max=120),(500,0,180)))

    def test_workspace_cartesian_bounds(self):
        c=Config(kind='cartesian')
        pts=workspace(c,4)
        self.assertEqual(tuple(map(min,zip(*pts))),(0,0,0))
        self.assertEqual(tuple(map(max,zip(*pts))),(c.xmax,c.ymax,c.zmax))

    def test_second_elbow(self):
        c=Config(q2min=-150,q2max=-10)
        q=inverse(c,(350,200,260))
        self.assertIsNotNone(q)
        self.assertLess(q[1],0)
        self.assertLess(math.dist(forward(c,q),(350,200,260)),1e-8)


class Simulation(unittest.TestCase):
    def test_settles(self):
        for kind in ('scara','cartesian','cylindrical'):
            r=simulate(Config(kind=kind))
            self.assertEqual(len(r.samples),401)
            self.assertLess(r.final,.1)
            self.assertGreater(r.maximum,100)
            self.assertLess(r.final,r.rms)
            self.assertEqual(r.samples[-1].t,4)

    def test_contours(self):
        for kind in ('scara','cartesian','cylindrical'):
            for path in ('line','circle'):
                c=Config(kind=kind,mode='contour',path=path,center_y=100)
                r=simulate(c)
                self.assertTrue(0<r.rms<100)
                self.assertTrue(all(math.isfinite(s.error) for s in r.samples))
                if path=='circle':self.assertLess(math.dist(r.samples[0].desired,r.samples[-1].desired),1e-9)

    def test_delay(self):
        c=Config(kind='cartesian',mode='contour')
        self.assertGreater(simulate(replace(c,cycles=10)).rms,simulate(replace(c,cycles=0)).rms)

    def test_bits(self):
        c=Config(kind='cartesian',end_x=351.123,end_y=201.345,end_z=261.777)
        self.assertLess(simulate(replace(c,bits=20)).final,simulate(replace(c,bits=6)).final)

    def test_velocity(self):
        c=Config(speed=30,angular_speed=15)
        r=simulate(c)
        for a,b in zip(r.samples,r.samples[1:]):
            for j in range(4):
                bound=math.radians(c.angular_speed) if j<3 else c.speed
                self.assertLessEqual(abs(b.q[j]-a.q[j]),bound*(b.t-a.t)+1e-9)

    def test_unreachable_mid_circle(self):
        c=Config(kind='cartesian',mode='contour',path='circle',center_x=100,center_y=100,radius=150)
        self.assertIsNotNone(inverse(c,desired(c,0)))
        with self.assertRaisesRegex(ValueError,'вне рабочей области'):simulate(c)

    def test_invalid_clock(self):
        with self.assertRaisesRegex(ValueError,'периода'):simulate(Config(frequency=1000))

    def test_time_grid(self):
        r=simulate(Config(duration=1.005))
        self.assertEqual(r.samples[-1].t,1.005)
        self.assertTrue(all(a.t<b.t for a,b in zip(r.samples,r.samples[1:])))

    def test_result_snapshot_and_metrics(self):
        c=Config()
        r=simulate(c)
        c.bits=4
        self.assertEqual(r.config.bits,16)
        self.assertAlmostEqual(r.rms,math.sqrt(sum(s.error**2 for s in r.samples)/len(r.samples)))
        self.assertEqual(r.maximum,max(s.error for s in r.samples))


class Projects(unittest.TestCase):
    def test_roundtrip_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'project.json'
            c=Config()
            c.save(p)
            self.assertEqual(Config.load(p),c)
            out=Path(tmp)/'results.csv'
            r=simulate(c)
            r.export_csv(out)
            with out.open(encoding='utf-8-sig',newline='') as f:rows=list(csv.reader(f))
            self.assertEqual(len(rows),len(r.samples)+1)
            self.assertEqual(len(rows[0]),8)
            self.assertAlmostEqual(float(rows[-1][-1]),r.final)

    def test_invalid_values(self):
        for c in (Config(dt=0),Config(bits=2),Config(bits=12.5),Config(tau=float('nan')),
                  Config(q1min=160),Config(kind='unknown'),Config(cycles=True),Config(mode=[])):
            with self.assertRaises((ValueError,TypeError)):c.validate()

    def test_bad_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'bad.json'
            p.write_text('{"version":99}')
            with self.assertRaises(ValueError):Config.load(p)


if __name__=='__main__':unittest.main()
