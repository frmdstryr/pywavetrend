'''
Created on Apr 6, 2013

@author: jrm5555@psu.edu & bdf5047@psu.edu
'''
import unittest
from rfid.network import Network
from rfid.tracker import Tracker
from pprint import pprint
import time

class TrackerTest(unittest.TestCase):
    nw = None
    r = None
    
    def setUp(self):
        self.nw = Network(0,None,port='/dev/ttyUSB0',baudrate=115200)
        self.nw.reset()

    def tearDown(self):
        self.nw.stop()


    def testRun(self):
        t = Tracker(self.nw)
        t.start()
        time.sleep(3*60)
        t.stop()
        t.getTrackedTags().values()[0].showPredictionMap()
        
if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()