#!/usr/bin/python
# -*- coding: UTF-8 -*-
'''
Created on Jan 3, 2013

@author: jrm5555@psu.edu & bdf5047@psu.edu
'''
import logging
logger = logging.getLogger(__name__)

from rfid.decoder import Packet

class GainMode:
    HIGH = 1 # Long range reader
    LOW = 0 # Short range reader
    
class AlarmTagFilter:
    ALL = 0 # Status = 0 - Report all tags
    WITH_ALARM = 1 # Status = 1 - Report only tags with an Alarm condition
    WITHOUT_ALARM = 2 # Status = 2 - Report only tags without any Alarm condition

class CMDPacket(Packet):
    """Automatically determines the Packet size, and sets the ID's. """
    def __init__(self,reader,cmd,data=None):
        l = 0
        if data is not None:
            try:
                l = len(data)
            except:
                l = 1
            
        if l>0:
            print "Has Data"
            super(CMDPacket,self).__init__([0xAA,l,reader.getNetworkId(),reader.getReaderId(),reader.getNodeId(),cmd,data])
        else:
            print "No Data"
            super(CMDPacket,self).__init__([0xAA,l,reader.getNetworkId(),reader.getReaderId(),reader.getNodeId(),cmd])

class Reader:
    # Reader properties
    network = None
    node_id = None
    reader_id = 0x00
    _connection = None
    
    # State properties
    tags = [] # List of all the RFID tags the reader can detect 
    
    def __init__(self,network,id):
        #assert type(network) is Network,"network must be of type %s, given %s"%(type(Network),type(network))
        
        self.network = network
        self.node_id = id
        self.reader_id = 0x00 
        
    def send(self,data,*args,**kwagrs):
        return self.network.send(data,*args,**kwagrs)
    
    def ask(self,*args,**kwagrs):
        return self.network.ask(*args,**kwagrs)
        
    def getReaderId(self):
        return self.reader_id
    
    def getNodeId(self):
        return self.node_id
        
    #================================== Reader Commands ========================================================
    # These are a group of all the commands that can be interpreted by the Wavetrend L-RX201 RFID reader.
    # Ideally these should be reader independent... eg this should be a universal interface
    # ==========================================================================================================
    def getNetworkId(self):
        return self.network.getId()
    
    def setNetworkId(self,id,**kwargs):
        """ Assign a Network ID to a reader """
        return self.ask(CMDPacket(self,cmd=0x04,data=id),**kwargs)
    
    def setReaderId(self,id,**kwargs):
        """ Assign a Reader ID to a reader 
        
        This command is used to assign a permanent ID to a reader. This ID will remain fixed within the reader, even
        during a network power-down.
        """
        if id != self.getReaderId():
            self.reader_id = id
            return self.ask(CMDPacket(self,cmd=0x05,data=id),**kwargs)
        
    def getTagPacket(self,**kwargs):
        """ Receive a single tag transmission from a reader 
        
        This command is used to retrieve the received tag transmission data when a reader is in Manual-polling
        mode. The command will have no value add once a reader is in Auto-polling mode, since the reader will
        pass on the tag information once it has been validated
        """
        # TODO: decode the response
        return self.ask(CMDPacket(self,cmd=0x06),**kwargs)
    
    def setRssiValue(self,rssi,**kwargs):
        """ Assign a RSSI rejection value to a reader 
        
        The RSSI value is uploaded into a reader and will be used to filter out any tag messages with a RSSI value
        less than the uploaded value. An uploaded RSSI value of 0 will disable the filtering and subsequently pass
        all tag messages onto the controller.
        """
        return self.ask(CMDPacket(self,cmd=0x07,data=rssi),**kwargs)
    
    def getRssiValue(self,**kwargs):
        """ Query a reader for the onboard RSSI rejection value 
        
        Retrieve the RSSI rejection value of a reader within the reader network. The reader uses this RSSI value as
        the threshold value and will reject all tag messages with a RSSI value less than this threshold value. A re-
        turned value of 0 means that the RSSI filtering is disabled. This enables the reader to do filtering (instead of
        the controller) and thus reduce network traffic.
        """
        pkt =  self.ask(CMDPacket(self,cmd=0x08),**kwargs)
        if pkt is not None:
            return pkt.getData()[0]
    
    def setSiteCode(self,site1,site2,site3,**kwargs):
        """ Assign an acceptance Site Code value to the reader 
        
        This command uploads a user selectable site code. This site code will be used to reject any tag messages
        containing a different site code. Only tag messages containing the same site code value as the uploaded site
        code value will be passed on to the controller (PC). This enables the reader to do filtering and thus reduces
        the amount of traffic on the network. An uploaded site code value of 0 will disable the site code filtering of the
        reader and subsequently enable the reader to pass all received tag messages onto the controller.
        """
        return self.ask(CMDPacket(self,cmd=0x09,data=bytearray([site1,site2,site3])),**kwargs)
    
    def getSiteCode(self,**kwargs):
        """ Query the reader for the acceptance Site Code value
        
        This command queries the reader on what site code filtering is being done on that specific reader. A returned
        value of 0 means that the site code filtering is disabled. 
        """
        pkt = self.ask(CMDPacket(self,cmd=0x0A),**kwargs)
        if pkt is not None:
            return pkt.getData()
        
    def setReceiverGain(self,gain,**kwargs):
        """ Enable the reader for: Short range / Long range reader. """
        valid = [GainMode.HIGH,GainMode.LOW]
        assert gain in valid,"gain must be in %s, given %s"%(valid,gain)
        return self.ask(CMDPacket(self,cmd=0x0B,data=gain),**kwargs)
    
    def getReceiverGain(self,**kwargs):
        """ Query the state of the reader: Short range / Long range """
        pkt = self.ask(CMDPacket(self,cmd=0x0C),**kwargs)
        if pkt is not None:
            return pkt.getData()[0]
        
    def setAlarmFilter(self,status,**kwargs):
        """ Assign a certain filter setting to the reader 
        
        This command enabled the reader to perform certain alarm filtering functions. These functions include:
         * Pass only tag messages which is in alarming mode
         * Pass only tag messages which is not in an alarming mode
        An alarm filter value of 0 will disable the filtering option on the reader and thus all received tag messages will
        be passed onto the controller.
        """
        valid = [AlarmTagFilter.ALL,AlarmTagFilter.WITH_ALARM,AlarmTagFilter.WITHOUT_ALARM]
        assert status in valid,"status must be in %s, given %s"%(valid,status)
        return self.ask(CMDPacket(self,cmd=0x0D,data=status),**kwargs)
    
    def getAlarmFilter(self,**kwargs):
        """ Query the filter setting on the reader 
        
        This command will retrieve the alarm filter settings on the reader. An alarm filter value of 0 means that the
        filter option has been disabled.
        """
        pkt = self.ask(CMDPacket(self,cmd=0x0E),**kwargs)
        if pkt is not None:
            return pkt.getData()[0]
    
    def getNumInvalidTags(self,**kwargs):
        """ Interrogate the reader for the amount of invalid message readings 
        
        This command will retrieve the amount of error tag messages, received by the reader, since the last query.
        These error messages include RF collisions of 2 or more tags (the result of multiple tags transmitting simultaneously). 
        Combining this command with the RF white noise calculation command can result in the reader
        being used as an effective in-field diagnostic tool.
        """
        pkt = self.ask(CMDPacket(self,cmd=0x0F),**kwargs)
        if pkt is not None:
            return (pkt.getData()[0]<<8)+pkt.getData()[1]
    
    def getSupplyVoltage(self,**kwargs):
        pkt = self.ask(CMDPacket(self,cmd=0x10),**kwargs)
        if pkt is not None:
            return pkt.getData()[0]
        
    def startRfWhiteNoiseCalculation(self,**kwargs):
        """ Enable the reader for evaluation mode. (40 second delay on reader) 
        
        This command configures the reader into the evaluation mode. Within this mode the reader is capable of
        calculating the radio frequency white noise level on the 433 MHz band and representing the result as a RSSI
        value. The evaluation mode lasts for 40 seconds. During this period of time the reader will not receive any
        tags transmissions. If a command is send to the reader, during the evaluation period, the calculation will be
        aborted and the reader will reset. After the 40 second time period the reader will continue with normal operation.
        """
        return self.ask(CMDPacket(self,cmd=0x11),**kwargs)
    
    def getRfWhiteNoiseResult(self,**kwargs):
        """ Query the white noise level on 433 MHz 
        
        This command will retrieve the calculated result once a Start RF white noise calculation command has been
        send to the reader (and the evaluation was completed successfully). This value is represented as a RSSI
        value and can be used for diagnostic purposes.
        
        return noise
        """
        pkt = self.ask(CMDPacket(self,cmd=0x12),**kwargs)
        if pkt is not None:
            return pkt.getData()[0]
    
    def setBaudRate(self,baudrate,**kwargs):
        """ Assign a different baud rate to a reader 
        
        This command reconfigures the baud rate on a reader or reader network. It is strongly recommended that
        this command is only used as an network broadcast, as readers on different baud rates is likely to cause
        network problems. The different baud rate values are:
         * 9600 baud
         * 19200 baud
         * 38800 baud
         * 57600 baud
         * 115200 baud
        """
        assert baudrate in Packet.BAUDRATES.values(), "baudrate must be in %s, got %s"%(Packet.BAUDRATES.values(), baudrate)
        
        return self.ask(CMDPacket(self,cmd=0xFE,data=Reader.BAUDRATES.index(baudrate)),**kwargs)
    
    def getVersionInformation(self,**kwargs):
        """ Query the hardware / software version of a reader 
        
        This command returns the hardware and software version of the queried reader.
        """
        pkt = self.ask(CMDPacket(self,cmd=0xFF),**kwargs)
        if pkt is not None:
            return pkt.getDecoding()['data']
    
    #================================== End of Reader Commands ========================================================
    
    #========================================== Reader Methods ========================================================
    #  These are API methods provided in the software to make life easier
    #==================================================================================================================
    
    
    #================================== End of Reader Methods ========================================================
    
