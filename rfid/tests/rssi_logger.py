'''
Created on Mar 3, 2013

@author: jrm5555@psu.edu & bdf5047@psu.edu
'''
import unittest

from rfid.network import Network
from rfid.reader import Reader
from rfid.decoder import PacketDecodedListener
import time
from datetime import datetime
import fileinput
import sys

class RssiLogger(PacketDecodedListener):
    log_file = "RSSI.csv"
    tag_ids = []
    
    def log(self,msg):
        print "[%s][RSSILogger]%s"%(datetime.now().strftime("%H:%M:%S.%f"),msg)
    
    def packetReady(self, event):
        """
        Prints the RSSI values to a CSV file in the row with the given tag ID like:
        reader_id[0],tag_ids[0], 81, 82, 81, etc...
        reader_id[1],tag_ids[1], 120, 119, 118, 123, etc...
        reader_id[0],tag_ids[1], 72, 76, 77, 77, etc...
        reader_id[1],tag_ids[2], 72, 62, 61, etc...
        
        Note: These are sorted by which unique readerID/tagID combination comes first
        """
        try:
            tag = event.getPacket().getDecoding()['data']['tag']
            new_id = False
            # Unique id for each reader/tag combination
            uid = 'R%sT%s'%(event.getPacket().getNodeId(),tag['id']) 
            if uid not in self.tag_ids:
                self.tag_ids.append(uid)
                new_id = True
            
            # Determine which line to write to/update:
            i = self.tag_ids.index(uid)
            if new_id: # Append a new row
                self.log("Reader ID %s found new Tag with ID %s"%(event.getPacket().getNodeId(),tag['id']))
                with open(self.log_file, (i==0 and 'w') or 'a') as csv:
                    csv.write("%sReaderID:%s,TagID:%s,%s"%(i>0 and '\n' or '',event.getPacket().getNodeId(),tag['id'],tag['rssi']))
            else:
                # Append to the end of the row
                for line in fileinput.input(self.log_file,inplace=1):
                    if fileinput.lineno()-1 == i:
                        line="%s,%s\n"%(line.rstrip(),tag['rssi']) # Append the new rssi value
                    if len(line.strip())>0:
                        sys.stdout.write(line)
            #self.log(event.getPacket().__repr__())
            self.log("UID=%s RSSI=%s "%(uid,tag['rssi']))
        except KeyError:
            pass
        if event.getPacket().getCommand()!=0x06:
            self.log(event.getPacket().pretty())

class RssiTest(unittest.TestCase):
    nw = None
    r = None
    
    def setUp(self):
        self.nw = Network(0,RssiLogger(),port='/dev/ttyUSB0',baudrate=115200)
        self.nw.reset()

    def tearDown(self):
        self.nw.stop()


    def testLogger(self):
        #readers = [Reader(self.nw,id=1),Reader(self.nw,id=2),Reader(self.nw,id=3)]
        #for r in readers:
        self.nw.startAutoPolling()
        time.sleep(60)
        self.nw.stopAutoPolling()
        pass


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()