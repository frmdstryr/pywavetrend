'''
Created on Feb 10, 2013

@author: frmdstryr@gmail.com
'''
import unittest

from rfid.network import Network
from rfid.reader import Reader

class ReaderTests(unittest.TestCase):
    nw = None
    r = None
    
    def setUp(self):
        self.nw = Network(0,port='/dev/ttyUSB0',baudrate=115200)
        self.r = Reader(self.nw,id=1)

    def tearDown(self):
        self.nw.stop()
    
    def testGetReceiverGain(self):
        self.r.getReceiverGain()
        
    def testGetSiteCode(self):
        self.r.getSiteCode()
    
    def testGetRssiValue(self):
        self.r.getRssiValue()
    
    def testGetAlarmFilter(self):
        self.r.getAlarmFilter()
        
    def testGetNumInvalidTags(self):
        self.r.getNumInvalidTags()
        
    def testGetSupplyVoltage(self):
        self.r.getSupplyVoltage()

    def testVersion(self):
        version = self.r.getVersionInformation(timeout=1)
        valid = {'Controller Firmware Version':0x11,
                 'RF Module Firmware Version':0x16,
                 'Controller Hardware Version':0x17,
                 'RF Module Hardware Version':0x14}
        assert version==valid,"Version %s should be %s" %(version,valid)
        
    def testGetTag(self):
        p = self.r.getTagPacket(timeout=2)
        data = p.getData()
        tag = p.getDecoding()['data']['tag']
        assert tag['header'] == bytearray([0x21,0x2A,0x2A]),"Header bytes didn't match"
        assert tag['footer'] == bytearray([0x0A,0x0D]),"Footer bytes didn't match"
        assert data[24] == 0x20, "Reserved byte 31 didn't match"
        assert data[6] == 0x42, "Reserved byte 13 didn't match"
        assert data[7] == 0x43, "Reserved byte 14 didn't match"
        


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()