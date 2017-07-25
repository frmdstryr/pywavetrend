'''
Created on Jan 27, 2013

@author: jrm5555@psu.edu & bdf5047@psu.edu
'''
import threading
import serial
import time

class SerialEvent:
    _data = None
    _serial = None
    
    def __init__(self, eventSource, data, serial):
            self._eventSource = eventSource
            self._data = data
            self._serial = serial
            
    def getData(self):
            return self._data
                                                                                                                                                                                                                                                     
    def getConnection(self):
            return self._serial
        
    def getSource(self):
            return self._eventSource
                                                                                                                                                                                                                                                         
                                                                                                                                                                                                                                                         
class SerialPortListener:
    def dataReceived(self, event):
        raise Exception("dataReceived - not implemented")
    
    def dataSent(self, event):
        raise Exception("dataSent - not implemented")
                                                                                                                                                                                                                                                         
class SerialPortMonitor(threading.Thread):
    _connection = None
    _monitoring = False
    _listeners = []
        
    def __init__(self,*params,**kwargs):
        threading.Thread.__init__(self)
        self._connection = serial.Serial(*params,**kwargs)
        self._listeners = []
        
    def getConnection(self):
        return self._connection
                                                                                                                                                                                                                                                     
    def addSerialEventListener(self, listener):
            self._listeners.append( listener )
                                                                                                                                                                                                                                                     
    def removeSerialEventListener(self, listener):
            self._listeners.remove( listener )
            
    def run(self):
        print self, 'starting monitor on %s'%self._connection
        self._monitoring = True
        while self._monitoring:
            if (self.getConnection().inWaiting()>0):
                self._fireEvent(SerialEvent(self,bytearray(self.getConnection().read(self.getConnection().inWaiting())),self.getConnection()),'dataReceived')
            # So it doesn't consume so much CPU resources
            time.sleep(0.01)
                                
        print self, 'shutting down'
        self._connection.close()
        return
        
        
    def _fireEvent(self, event,method):
            #print "%s event fired"%method
            for listener in self._listeners:
                getattr(listener,method)(event)
                #t = threading.Thread(target=getattr(listener,method),args=[event])
                #t.start()
    
    def stop(self):
        self._monitoring = False
        self.join()