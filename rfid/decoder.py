'''
Created on Jan 27, 2013

@author: jrm5555@psu.edu & bdf5047@psu.edu
'''
import threading
import time
import Queue
from datetime import datetime

from rfid.serial_port_monitor import SerialPortListener

class Packet(bytearray):
    BAUDRATES = {0:115200,1:57600,2:38800,3:19200,4:9600}
    
    ALARM_STATUS_MAP = {
        0:"Report all tags",
        1:"Report only tags with an Alarm condition",
        2:"Report only tags without any Alarm condition",
    }
    
    RECEIVER_GAIN_MAP = {
        0:"Low Gain Mode - Short range reader",
        1:"High Gain Mode - Long range reader",
    }
    TAG_MODE_MAP = {0:'Reconfigure',1:'Non-reconfigure (Programmable)'}
    
    PUK_MAP = {0:"PUK Code Not Transmitted",1:"PUK Code Transmitted"}
    
    POLL_MAP = {
        0:"Tag can not be polled to Tx between normal transmissions",
        1:"Tag can be polled to transmit within transmission interval"
    }
    
    CMD_MAP = {
        0x00:"Reset Network",
        0x01:"Enable Auto Polling",
        0x02:"Disable Auto Polling",
        0x03:"Ping Reader",
        0x04:"Set Network ID",
        0x05:"Set Reader ID",
        0x06:"Get Tag Packet",
        0x07:"Get RSSI Value",
        0x08:"Set RSSI Value",
        0x09:"Set Site Code",
        0x0A:"Get Site Code",
        0x0B:"Set Receiver Gain",
        0x0C:"Get Receiver Gain",
        0x0D:"Set Alarm Filter",
        0x0E:"Get Alarm Filter",
        0x0F:"Get Number of Invalid Tags",
        0x10:"Get Supply Voltage",
        0x11:"Start RF white noise calculation",
        0x12:"Get RF white noise result",
        0xFE:"Set Baud Rate",
        0xFF:"Get Version Information",
    }
    
    PING_ERROR_MAP = (
        "No errors encountered",
        "Unknown reader command received",
        "Tag Table underflow error",
        "Command Packet checksum error",
        "RF Module - Unknown command response",
        "RF Module - Unknown general response",
        "RF Module - Re-sync failure",
        "RF Module - Command response failure",
        "RF Module - Receive response failure",
        "No response packet received from polled reader"
    )
    
    TRANSMISSION_INTERVAL_MAP = {
        0x30: "30 sec",
        0x31: "1.5 sec",
        0x32: "0.8 sec",
        0x33: "0.4 sec",
        0x20: "15 sec"
    }
    
    TAG_TYPE_MAP = {
        0x30:"Fused, non-programmable",
        0x33:"Not fused, programmable",
    }
    
    def __init__(self,*params):
        super(Packet,self).__init__(*params)
        
        if len(self) != (7 + self.getDataLength()):
            self._hasChecksum = False
            self.setChecksum()
        else:
            self._hasChecksum = True
            
    def getHeader(self):
        return self[0]
    
    def getDataLength(self):
        return self[1]
    
    def getNetworkId(self):
        return self[2]
    
    def getReaderId(self):
        return self[3]
    
    def getNodeId(self):
        return self[4]
    
    def getCommand(self):
        return self[5]
    
    def getData(self):
        i = -2
        if self._hasChecksum: i+=1 
        return self[6:i]
    
    def setChecksum(self):
        checksum = 0x00
        if self._hasChecksum: self.pop() 
        for i in range(1,len(self)): # skip the header and checksum byte
            checksum ^= self[i] # CHECKSUM = [Length] XOR [Network ID] XOR [Receiver ID ] XOR [Node ID ] XOR [Command] XOR [Data]
        self.append(checksum)
        self._hasChecksum = True
    
    def _decodeTagPacket(self):
        """
        Examples:
        Response     Data
                     !  *   *                                                                              LF CR         Checksum?
        552000000106 21 2a 2a 31 01 11 42 43 90 00 69 94 a2 1e ac ac 00 0f 2a ef 33 00 68 32 20 51 01 00 00 11 0a 0d     32 
        552000000106 21 2a 2a 31 01 11 42 43 90 00 69 94 a3 1e ac ac 00 0f 2a ef 33 00 68 33 20 51 01 00 00 11 0a 0d     32
        552000000106 21 2a 2a 31 01 11 42 43 90 00 69 94 a4 1e ac ac 00 0f 2a ef 33 00 68 34 20 51 01 00 00 11 0a 0d     32
        552000000106 21 2a 2a 20 01 11 42 43 90 00 69 94 a5 1e ac ac 00 0f 2a ef 33 00 68 24 20 50 01 00 00 11 0a 0d     33
        552000000106 21 2a 2a 20 01 11 42 43 d1 00 69 96 3e 1e ac ac 00 0f 2a ef 33 00 37 00 20 50 01 00 00 11 0a 0d     90
        552000000106 21 2a 2a 20 01 11 42 43 d1 00 69 96 3f 1e ac ac 00 0f 2a ef 33 00 37 01 20 50 01 00 00 11 0a 0d     90
        552000000106 21 2a 2a 20 01 11 42 43 d1 00 69 96 40 1e ac ac 00 0f 2a ef 33 00 37 02 20 50 01 00 00 11 0a 0d     ec
        552000000106 21 2a 2a 20 01 11 42 43 d1 00 69 96 41 1e ac ac 00 0f 2a ef 33 00 37 03 20 50 01 00 00 11 0a 0d     ec
        552000000106 21 2a 2a 20 01 11 42 43 d1 00 69 96 42 1e ac ac 00 0f 2a ef 33 00 37 04 20 50 01 00 00 11 0a 0d     e8
        552000000106 21 2a 2a 31 01 11 42 43 de 00 69 96 88 1e ac ac 00 0f 2a ef 33 00 37 68 20 51 01 00 00 11 0a 0d     51
                      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 15 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31     32
                      !  *  *           B  C    |  age     | site   | tag id    |  | 
        """           
        data = self.getData()
        l = self.getDataLength()
        
        d = {'header':data[0:3]}
        
        try:
            d['interval'] = Packet.TRANSMISSION_INTERVAL_MAP[data[3]]
        except:
            d['interval'] = "Invalid - %s"%data[3]
        
        d['counter'] = data[4]&0x7F
        d['reed'] = data[4]&0x80
        d['firmware version'] = data[5]/10.0
        # data[6] reserved
        # data[7] reserved
        d['alarm counter'] = data[8]
        i = 9
        d['age'] = (data[i]<<24)+(data[i+1]<<16)+(data[i+2]<<8)+data[i+3]
        i+=4
        d['site code'] = (data[i]<<16)+(data[i+1]<<8)+data[i+2]
        i+=3
        d['id'] = (data[i]<<24)+(data[i+1]<<16)+(data[i+2]<<8)+data[i+3]
        
        try:
            d['type'] = Packet.TAG_TYPE_MAP[data[20]]
        except:
            d['type'] = "Invalid - %s"%data[20]
            
        d['reader address'] = data[21]
        d['rssi'] = data[22]
        d['CRC'] = data[23]
        # data[24] reserved
        d['alarm byte'] = data[25]
        d['node ID'] = data[26]
        d['network ID'] = data[27]
        d['reader RSSI'] = data[28]
        #d['firmware version'] = data[29] #idk why this is reported twice...
        d['footer'] = data[-2:]
        return {'tag':d}
    
    def getDecoding(self):
        """ Makes a readable version of the binary data """
        d = {'Network Id':self.getNetworkId(),'Reader Id':self.getReaderId(),'Node Id':self.getNodeId(),} 
        if self.getHeader() == 0xAA:
            d['type'] = "Command"
        elif self.getHeader() == 0x55:
            d['type'] = "Response"
        else:
            d['type'] = "Unknown"
        try:
            d['cmd'] = Packet.CMD_MAP[self.getCommand()]
        except:
            d['cmd'] = "Unknown"
        if self.getDataLength()>0:
            c = self.getCommand()
            data = self.getData()
            
            if c==0x03: # Ping Command
                try:
                    repr = Packet.PING_ERROR_MAP[data[0]]
                except:
                    repr = "Unknown"
                d['data'] = {'Error Number':"%s-%s"%(data[0],repr)}
            
            elif (c==0x04 or c==0x05): # Set Network/Reader ID
                d['data'] = {'ID':data[0]}
            
            elif c==0x06: # Get Tag Packet
                d['data'] = self._decodeTagPacket()
                
            elif (c==0x07 or c==0x08): #RSSI Value Commands
                d['data'] = {'RSSI':data[0]}
                
            elif (c==0x09 or c==0x0A): # Site Code Commands
                d['data'] = {'Site1':data[0],'Site2':data[1],'Site3':data[2]}
            
            elif (c==0x0B or c==0x0C): # Receiver Gain Command Commands
                try:
                    repr = Packet.RECEIVER_GAIN_MAP[data[0]]
                except:
                    repr = "Unknown"
                d['data'] = {'Gain':"%s-%s"%(data[0],repr)}
            
            elif (c==0x0D or c==0x0E): # Alarm Tag Filter Status Commands
                try:
                    repr = Packet.ALARM_STATUS_MAP[data[0]]
                except:
                    repr = "Unknown"
                d['data'] = {'Status':"%s-%s"%(data[0],repr)}
            
            elif c==0x0F: # Get Invalid Tag Count
                d['data'] = {'Count':(data[0]<<8)+data[1]}
            
            elif c==0x10: # Get Power Supply Voltage
                d['data'] = {'Voltage':data[0]/10.0}
                
            elif c==0x12: # Get Environmental Noise Level Value
                d['data'] = {'Noise':data[0]}
                
            elif c==0xFF: # Get Version Info
                d['data'] = {'Controller Firmware Version':data[0]/10.0,'RF Module Firmware Version':data[1]/10.0,'Controller Hardware Version':data[2]/10.0,'RF Module Hardware Version':data[3]/10.0}
                
            else:
                d['data'] = "Not implemented!"
            
        return d
    
    def pretty(self):
        if (self.getHeader()==0xAA):
            return "[PC->Reader][Node:%s][CMD:%s]"%(self.getNodeId(),self.getDecoding()['cmd'])
        else: 
            return "[PC<-Reader][Node:%s][RSP:%s]"%(self.getNodeId(),self.getDecoding()['cmd'])
    
    def __repr__(self):
        # TODO: Make this look cool
        return "Packet(bin=%s)<%s>"%(str(self).encode('hex'),self.getDecoding())#pformat(self.getDecoding()))

class PacketDecodedEvent:
    _data = None
    _packet = None
    def __init__(self, eventSource, byteArray):
            self._eventSource = eventSource
            self._packet = Packet(byteArray)
            self._time = time.time()
    
    def getPacket(self):
        """ Get the packet's data """
        return self._packet
    
    def getTime(self):
        """ Get time at which the packet was received"""
        return self._time
            
class PacketDecodedListener:
    def packetReady(self, event):
        raise Exception("packetReady - not implemented")
            
class WaitForReplyListener(threading.Thread,PacketDecodedListener):
    """ Waits for the given timeout duration and sets the reply if found."""
    _waitForCmd = None
    reply = None
    
    def __init__(self,waitForCmd,timeout=1):
        threading.Thread.__init__(self)
        self._waitForCmd = waitForCmd
        self.queue = Queue.Queue() # Holds all of the incoming data until this thread can decode it
        t = threading.Timer(timeout, self.stop)
        t.start()
        self._waiting = True
        
    def run(self):
        # Wait's until something is put in the Queue (eg when we call self.stop())
        self.queue.get(block=True)
        
    
    def stop(self):
        self.queue.put("shutdown")
        
    def packetReady(self,event):
        pkt =  event.getPacket()
        #print "packet ready %s"%pkt.getDecoding()
        if pkt.getHeader()==0x55 and (self._waitForCmd is None or  pkt.getCommand() == self._waitForCmd):
            self.reply = pkt
            self.stop()
            
            
class Decoder(SerialPortListener,threading.Thread):
    """ Decodes the serial port response.. should fire events when packet is decoded."""
    def __init__(self):
        threading.Thread.__init__(self)
        self.queue = Queue.Queue() # Holds all of the incoming data until this thread can decode it
        self.packets = [bytearray()] # holds the decoded packets
        self._listeners = [] # TODO: when a packet is decoded fire an event
        self._bytesLeft = 0 # Track whether or not we're in the middle of a packet's data section
    
    def doesChecksumMatch(self,packet):
        """ Checks that the checksum is correct for the packet. Returns a boolean checksum matched"""
        checksum = 0x00 
        for i in range(1,len(packet)-1): # skip the header and checksum byte
            checksum ^= packet[i] # CHECKSUM = [Length] XOR [Network ID] XOR [Receiver ID ] XOR [Node ID ] XOR [Command] XOR [Data]
            
        return checksum == packet[-1]#,"Warning - Discarding packet, checksum was incorrect %s != %s"%(checksum,packet[-1])) 
    
    def decode(self,data,retries=100):
        if len(data)<1:
            return
        if len(self.packets[-1])==0:
            # Look for a new packet header
            i1 = data.find('\xAA')
            i2 = data.find('\x55')
            
            if i1 != -1 and (i2 == -1 or i1<i2):
                # We found a command packet
                if i1 != 0: 
                    self.log("[DEBUG] Out of sync! Unexpected Data: %s"%(str(data[0:i1]).encode('hex')))
                    data = data[i1:] # This should be the new packet start
            elif i2 != -1 and (i1 == -1 or i2<i1):
                # We found a response packet
                if i2 != 0: 
                    self.log( "[DEBUG] Out of sync! Unexpected Data: %s"%(str(data[0:i2]).encode('hex')))
                    data = data[i2:] # This should be the new packet start
            else:
                return#raise RuntimeError("Decoder error! This should never get executed!")
            
            self.packets[-1].append(data.pop(0))
            # Back to the top
            self.decode(data,retries)
        
        elif len(self.packets[-1])==1:
            # This byte should be the packet length
            self.packets[-1].append(data.pop(0))
            self._bytesLeft = 5+self.packets[-1][1] # have header and length bytes, need 3 id bytes, cmd, and checksum = 5 bytes, + data bytes
            #self.decode(data)
            
        
        # The previous packet still needs more bytes to be complete
        while self._bytesLeft>0 and len(data)>0:
            self.packets[-1].append(data.pop(0))   
            self._bytesLeft -=1
            
        # If the previous packet has all of it's bytes, start a new one
        if len(self.packets[-1])>6 and self._bytesLeft==0:
            # TODO: fire an event saying a packet is decoded!
            if self.doesChecksumMatch(self.packets[-1]):
                """
                if (self.packets[-1][0] == 0xAA):
                    print "[%s][Decoder][CMD]%s"%(time.strftime('%x %X'),str(self.packets[-1]).encode('hex'))
                    pass
                elif (self.packets[-1][0] == 0x55):
                    print "[%s][Decoder][RSP]%s"%(time.strftime('%x %X'),str(self.packets[-1]).encode('hex'))
                    pass
                else:
                    raise RuntimeError("Decoded an unknown packet type! %s"%self.packets[-1][0].__repr__())
                """
                self._fireEvent(PacketDecodedEvent(self,self.packets[-1]), 'packetReady')
            else:
                self.log( '[DEBUG] Checksum failed! Discarding packet.')             
            # Start a new packet!
            self.packets.append(bytearray())
        
        retries -=1
        if retries>0:
            self.decode(data,retries)
        else:
            self.log('[DEBUG] Recursion limit exceeded! Discarding packet!')
            self.packets.append(bytearray())
            #self.decode(data)
            
        
    
    
    def run(self):
        while True:
            event = self.queue.get(block=True)
            if event == 'shutdown':
                print self, 'shutting down'
                return
            #print "[%s][Decoder][RAW]%s"%(time.strftime('%x %X'),str(event.getData()).encode('hex'))
            self.decode(event.getData())
            
            
    def stop(self):
        self.queue.put("shutdown")
        self.join()
        
    def dataReceived(self, event):
        """
        So if I have a list of data... is it easier to wait for it? or search backwards and decode that way?
        """
        self.queue.put(event,False)
        
    def addPacketDecodedEventListener(self, listener):
        self._listeners.append( listener )
                                                                                                                                                                                                                                                     
    def removePacketDecodedEventListener(self, listener):
            self._listeners.remove( listener )
    
    def _fireEvent(self, event,method):
        #print "%s event fired"%method
        for listener in self._listeners:
            getattr(listener,method)(event)
            
    def log(self,msg,tag="Decoder"):
        print "[%s][%s]%s"%(datetime.now().strftime("%H:%M:%S.%f"),tag,msg)
