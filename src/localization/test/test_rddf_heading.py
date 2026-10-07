#!/usr/bin/python3
from pathlib import Path
import sys,math,unittest,yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from calibrated_imu_core import HeadingCalibration,rpy

class RddfHeadingTest(unittest.TestCase):
    def core(self):
        p=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/imu_heading_calibration.yaml').read_text())
        return HeadingCalibration(p['imu_heading_calibration'])
    def test_mid_course_heading_uses_latest_imu_once_without_consuming_gnss(self):
        c=self.core();q=(0.,0.,math.sin(.7/2),math.cos(.7/2))
        c.observe_imu(100.,q,(0.,0.,0.),100.)
        self.assertTrue(c.select_initial_heading(-1.2,'MANUAL_RDDF:7:23',100.,(0.,0.,0.,1.)))
        self.assertAlmostEqual(rpy(c.output_orientation(q))[2],-1.2)
        self.assertFalse(c.calibrated)
        self.assertFalse(c.select_initial_heading(1.,'GPS_RDDF',100.,(0.,0.,0.,1.)))
    def test_no_or_stale_imu_does_not_fit(self):
        c=self.core();self.assertFalse(c.select_initial_heading(1.,'x',100.,(0.,0.,0.,1.)))
        c.observe_imu(100.,(0.,0.,0.,1.),(0.,0.,0.),100.)
        self.assertFalse(c.select_initial_heading(1.,'x',101.,(0.,0.,0.,1.)))
    def test_clock_reset_accepts_a_new_location(self):
        c=self.core();q=(0.,0.,0.,1.)
        c.observe_imu(100.,q,(0.,0.,0.),100.)
        self.assertTrue(c.select_initial_heading(1.,'x',100.,q))
        c.observe_time(50.);c.observe_imu(50.,q,(0.,0.,0.),50.)
        self.assertTrue(c.select_initial_heading(-2.,'other',50.,q))
        self.assertAlmostEqual(rpy(c.output_orientation(q))[2],-2.)
if __name__=='__main__':unittest.main()
