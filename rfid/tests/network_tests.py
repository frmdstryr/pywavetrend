'''
Created on Feb 10, 2013

@author: jrm5555@psu.edu
'''
import unittest

from rfid.network import Network
from rfid.decoder import Packet

class NetworkTests(unittest.TestCase):
    nw = None

    def setUp(self):
        self.nw = Network(0,logger=None,port='/dev/ttyUSB0',baudrate=115200)


    def tearDown(self):
        self.nw.stop()
    
    def testAsk(self):
        print "start ask"
        self.nw.ask(Packet([0xAA,0x00,self.nw.getId(),0x00,0xFF,0x00]),timeout=30)
        print "stop ask"
    
    def testReset(self):
        print "start reset"
        self.nw.reset()
        print "stop reset"

    def testRescan(self):
        self.nw.rescan()
        assert len(self.nw.getReaders(False)) == 3


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()