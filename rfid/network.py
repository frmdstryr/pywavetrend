'''
Created on Jan 26, 2013

@author: jrm5555@psu.edu & bdf5047@psu.edu
'''
#!/usr/bin/env python
import time
from datetime import datetime
from rfid.serial_port_monitor import SerialPortMonitor,SerialPortListener
from rfid.reader import Reader
from rfid.decoder import Decoder,Packet,WaitForReplyListener,PacketDecodedListener

class Logger(PacketDecodedListener,SerialPortListener):
    def dataReceived(self, event):
        print "[%s][Logger] %s"%(time.strftime('%x %X'),str(event.getData()).encode('hex'))
        pass
    
    def packetReady(self, event):
        print "[%s][Logger] %s"%(datetime.now().strftime("%H:%M:%S.%f"),event.getPacket().__repr__())        
        

class Network():
    networkId = 0
    _readers = []
    _monitor = None
    _decoder = None
    _logger = None
    
    def __init__(self,networkId,logger=Logger(),*params,**kwargs):
        self.setId(networkId)
        self._monitor = SerialPortMonitor(*params,**kwargs)
        self._decoder = Decoder()
        
        # Start the logger
        self._logger = logger
        if isinstance(logger,SerialPortListener):
            self.getMonitor().addSerialEventListener(self._logger)
        if isinstance(logger,PacketDecodedListener):
            self.getDecoder().addPacketDecodedEventListener(self._logger)
       
       # Start monitoring 
        self.getMonitor().addSerialEventListener(self._decoder)
        self.getMonitor().start()
        self.getDecoder().start()
        
        
        
        
    
    def setId(self,networkId):
        self.networkId = networkId
        # for each reader: reader.setNetworkId(networkId)
    
    def getId(self):
        return self.networkId
    
    def rescan(self):
        """Rescan the network for readers """
        print "Pinging readers..."
        self.breakAutoPolling()
        self._readers = []
        for i in range(1,254):
            rspPkt = self.pingReader(i,timeout=0.1)
            time.sleep(0.5)
            if rspPkt is not None:
                if rspPkt.getData()[0] == 0x00: # No errors encountered
                    self._readers.append(Reader(network=self,id=rspPkt.getReaderId()))
                    print "Found reader at node %s"%i
                else:
                    try:
                        erNo = Packet.PING_ERROR_MAP[rspPkt.getData()[0]]
                    except:
                        erNo = "Unknown"
                    print "Reader %s returned %s-%s"%(rspPkt.getReaderId(),rspPkt.getData()[0],erNo)
            else:
                break 
        print "Found %s readers."%len(self._readers)
            
    def getMonitor(self):
        return self._monitor
    
    def getDecoder(self):
        return self._decoder
    
    def getReaders(self,rescan=False):
        if rescan:
            self.rescan()
        return self._readers

    def getConnection(self):
        return self.getMonitor().getConnection()
    
    def send(self,data):
        """ Sends a packet and doesn't wait for any response """
        #print "Sent %s"%(str(data).encode('hex'))
        return self.getConnection().write(data)
    
    def ask(self,packet,replyCmd=None,timeout=1):
        """ Sends the packet and listens for a response"""
        if (replyCmd is None): replyCmd = packet.getCommand()
        l = WaitForReplyListener(replyCmd,timeout)
        self._decoder.addPacketDecodedEventListener(l)
        l.start()
        self.send(packet)
        l.join()
        self._decoder.removePacketDecodedEventListener(l)
        return l.reply
    
    def reset(self,**kwargs):
        """Reset the entire reader network
        
        The Reset Network command will reset a reader within the reader network. If the command is send as a
        broadcast (network command), the entire reader network will reset. Once a reset has been initiated all the
        non-queried tag transmissions received by the reader (and still in memory) will be lost.
        """
        self.breakAutoPolling()
        return self.ask(Packet([0xAA,0x00,self.getId(),0x00,0xFF,0x00]),timeout=30)
        
    def pingReader(self,nodeId=None,readerId=None,**kwargs):
        """ Interrogate a single reader on a reader network 
        
        This command can be used to test if a specific reader, within the reader network, is still active and responding.
        """
        if nodeId is not None:
            return self.ask(Packet([0xAA,0x00,self.getId(),0x00,nodeId,0x03]),**kwargs)
        elif readerId is not None:
            return self.ask(Packet([0xAA,0x00,self.getId(),readerId,0x00,0x03]),**kwargs)
        
    def breakAutoPolling(self):
        """ 
        Breaking the polling sequence does not alter the Auto Polling Flag in the Data EEPROM, but
        only suspends polling. After a physical reset, this system will start polling again unless the auto
        polling is disabled with the appropriate command.
        """
        return self.send(bytearray(400*[0xFF,'*']))
    
    def startAutoPolling(self,**kwargs):
        """ Select the Automatic polling mode
        
        This command will enable the reader network to be configured into an auto-polling mode. In auto-polling
        mode the reader will pass on the received tag transmission data, to the controller, without being instructed to
        do so by the controller. Auto-polling mode will decrement the amount of traffic commands from the host 
        controller to the reader network and thus lessen the traffic on the network.
        
        This only works for nodeID 1, so it is a network method.
        """
        
        return self.ask(Packet([0xAA,0x00,self.getId(),0x00,0x01,0x01]),**kwargs)
        
    
    def stopAutoPolling(self,**kwargs):
        """ Select the Manual polling mode 
        
        This command will enable the reader into the Manual-polling mode. Within this mode the reader will not pass
        the received tag transmission data to the controller, unless instructed to do so. Manual-polling mode has the
        advantage of giving the controller (PC) total control over the network traffic.
        """
        self.breakAutoPolling()
        return self.ask(Packet([0xAA,0x00,self.getId(),0x00,0x01,0x02]),**kwargs)
    
    def stop(self):
        self.getMonitor().stop()
        self._decoder.stop()
    

