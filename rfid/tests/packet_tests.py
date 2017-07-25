'''
Created on Feb 10, 2013

@author: frmdstryr@gmail.com
'''
import unittest

from rfid.decoder import Packet

class PacketTests(unittest.TestCase):

    def testRescan(self):
        pkt = Packet(bytearray([0xAA,0x00,0x01,0x02,0x03,0x04,0x05]))
        
        assert pkt.getHeader() == 0xAA
        assert pkt.getDataLength() == 0x00
        assert pkt.getNetworkId() == 0x01
        assert pkt.getReaderId() == 0x02
        assert pkt.getNodeId() == 0x03
        assert pkt.getCommand() == 0x04
        assert pkt.getData()[0] == 0x05
        


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()