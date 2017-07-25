# -*- coding: utf-8 -*-
'''
Created on Mar 31, 2013

@author: jrm5555@psu.edu & bdf5047@psu.edu
'''
from __future__ import division
import traceback
import threading
import Queue
from rfid.reader import Reader
from rfid.decoder import PacketDecodedListener
import time
from datetime import datetime
import fileinput
import sys
from collections import deque
import itertools
import numpy as np

# For plotting the prediction map
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm

class Config(object):
    """
    6) Window_size, w: The number of observations
    per reader that will be used for calculating Bel ( x t ) in
    one recursion (Fig. 4).
    """
    windowSize = 20 
    
    """
    7) Recursion_time, r: The number of times we
    execute the recursive Bayesian inference to estimate the
    location of the tracked object.
    """
    recursionTime = 5
    
    """
    8) Estimated Area Threshold, P: The estimation
    area is given by the grid cells with belief higher or
    equal to P.
    """
    estimatedAreaThreshold = 0.8
    
    
    """
    System dynamics model standard deviatoin
    """
    systemDynamicsSigma = 5
    
    gridX = 20
    gridY = 20
    
    
    """ 
    Default z value
    """
    z = 3

class Tag(object):
    def __init__(self,id,tracker,position=None,reference=False):
        self.id = id # The tracked object ID,
        self.position = position # State (grid cell) of an object at time t,
        self.positionBelief = None # Bel(t^(t-1))
        self.reference = reference # Is this a reference tag?
        self.tracker = tracker
        self.meas = {} # the RSSI of the object (or reference tag) reported by reader_j at time t.
        self.timeLastDetected = None
        self.predictionMap = None #Numpy array that stores the previous Bel predictions
        self.numMeas = 0
        self.timeLastDetected = None # Reference to the Tracker class
        self.areaWasEstimated = False # Set to once the area was given (as the initial position is invalid)
        self.z = Config.z
        
        # Fill the predictionMap with the initial belief that each cell
        # is equally likely
        if not reference:
            self.predictionMap = np.zeros((tracker.grid.x,tracker.grid.y))
            self.predictionMap.fill(1/float(tracker.grid.N))
            self.positionBelief = 1/float(tracker.grid.N)
            
    def setPredictionMap(self,pmap):
        if pmap.max()>0:
            self.areaWasEstimated = True
            self.predictionMap = pmap/pmap.sum()
        else:
            self.log("[WARN ] Prediction map was empty!")
            
    def setInitialPredicMap(self,pmap):
        if pmap.max()>0:
            self.initialPredic = pmap/pmap.sum()
        else:
            self.log("[WARN ] Initial prediction map was empty!")
            
    def setPerceptualPredicMaps(self,pmaps):
        self.perceptualPredic = pmaps
        #    self.log("[WARN ] Initial prediction map was empty!")
        
    def getPredictionMap(self):
        return self.predictionMap
        
    def showPredictionMap(self):
        """ Plots a 3d map showing the belief that this tag is in 
        the given area.
        """
        X = np.arange(0, self.predictionMap.shape[0])
        Y = np.arange(0, self.predictionMap.shape[1])
        X, Y = np.meshgrid(X, Y)
        
        fig = plt.figure()
        
        ax = fig.add_subplot(2, 2, 1, projection='3d')
        ax.plot_surface(X, Y, self.initialPredic, rstride=1, cstride=1,linewidth=0, antialiased=True,shade=True,vmin=0,vmax=1,alpha=0.5)
        #ax.set_zscale('log')
        ax.set_xlabel('Grid Position X')
        ax.set_xlim(0, self.predictionMap.shape[0])
        ax.set_ylabel('Grid Position Y')
        ax.set_ylim(0, self.predictionMap.shape[1])
        ax.set_zlabel('Initial Prediction')
        #ax.set_zlim(0, 1)
        
        
        # Perceptual predictions
        ax = fig.add_subplot(2, 2, 2, projection='3d')
        ax.plot_surface(X, Y, self.perceptualPredic['total'], rstride=1, cstride=1,linewidth=0, antialiased=True,shade=True,vmin=0,vmax=1,alpha=0.5)
        #ax.set_zscale('log')
        ax.set_xlabel('Grid Position X')
        ax.set_xlim(0, self.predictionMap.shape[0])
        ax.set_ylabel('Grid Position Y')
        ax.set_ylim(0, self.predictionMap.shape[1])
        ax.set_zlabel('Total Perceptual Prediction')
        #ax.set_zlim(0, 1)
        
        # Final prediction map
        ax = fig.add_subplot(2, 2, 3, projection='3d')
        ax.plot_surface(X, Y, self.predictionMap, rstride=1, cstride=1, linewidth=0, antialiased=True,shade=True,vmin=0,vmax=1,alpha=0.5)
        ax.contourf(X, Y, self.predictionMap, zdir='z', offset=0, antialiased=True,alpha=0.7,vmin=0,vmax=1)
        #ax.set_zscale('log')
        ax.set_xlabel('Grid Position X')
        ax.set_xlim(0, self.predictionMap.shape[0])
        ax.set_ylabel('Grid Position Y')
        ax.set_ylim(0, self.predictionMap.shape[1])
        ax.set_zlabel('Final Prediction')
        #ax.set_zlim(0, 1)
        
        # 2D Setup Display        
        ax = fig.add_subplot(2, 2, 4)
        first = True
        for reader in self.tracker.readers.values():
            ax.scatter(reader.position[0],reader.position[1],s=40,marker='o',label=(first and "Reader" or None))
            first = False 
        
        first = True
        for reftag in self.tracker.getReferenceTags().values():
            ax.scatter(reftag.position[0],reftag.position[1],marker='s',label=(first and "RefTag" or None))
            first = False
        if self.getPosition() is not None:
            ax.scatter(self.getPosition()[0],self.getPosition()[1],marker='s',c='r',label="TrackedTag")
        
        #ax.legend(loc='best',prop={'size':8})
        ax.grid()
        ax.set_xlabel('Grid Position X')
        ax.set_xlim(-0.5, self.predictionMap.shape[0]+0.5)
        ax.set_ylabel('Grid Position Y')
        ax.set_ylim(-0.5, self.predictionMap.shape[1]+0.5)
        
        plt.show()
        
        # Perceptual predictions
        fig = plt.figure()
        numReaders = len(self.tracker.readers.keys())
        subplotId = 1
        for reader in self.tracker.readers.values():
            ax = fig.add_subplot(1, numReaders, subplotId, projection='3d')
            #ax.set_zscale('log')
            ax.plot_surface(X, Y, self.perceptualPredic[reader.id], rstride=1, cstride=1,linewidth=0, antialiased=True,shade=True,vmin=0,vmax=1,alpha=0.5)
            if self.getPosition() is not None:
                ax.scatter(self.getPosition()[0],self.getPosition()[1],marker='s',c='r',label="TrackedTag")
            ax.scatter(reader.position[0],reader.position[1],s=40,marker='o',label="Reader")
            
            ax.set_xlabel('Grid Position X')
            ax.set_xlim(0, self.predictionMap.shape[0])
            ax.set_ylabel('Grid Position Y')
            ax.set_ylim(0, self.predictionMap.shape[1])
            ax.set_zlabel('Reader %s Perceptual Prediction'%reader.id)
            #ax.set_zlim(0, 1)
            subplotId+=1
        plt.show()

    
    def addMeas(self,rssi,readerId):
        if readerId not in self.getReaderIds():
            self.meas[readerId] = {
                'data':deque(maxlen=Config.windowSize),
                'mean':0, #the average received signal strength of tag_i reported by reader_j
                'std':0,
                'sum':0,
                'count':0,
            }
            
        # This deque automatically restricts the size of the list to the windowSize 
        self.meas[readerId]['data'].append(rssi)
        
        # Recalculate the mean, std, and 
        # TODO: Make more efficient if necessary
        a = np.array(self.meas[readerId]['data'])
        self.meas[readerId]['mean'] = a.mean()
        self.meas[readerId]['std'] = a.std()
        self.meas[readerId]['sum'] = a.sum()
        self.meas[readerId]['count'] +=1
            
        self.timeLastDetected = time.time()
        
    def isReady(self):
        totalMeas = 0
        for row in self.meas.values():
            totalMeas +=row['count']
        return totalMeas > Config.windowSize
    
    def setPosition(self,pos,belief=1):
        """ TOOD: The tracker should calculate the position when a new
            measurement is reported.
         """
        self.position = pos
        self.positionBelief = belief
        
    def getPosition(self):
        """
        Returns the cell with the highest belief. Note if two cells are tie, it returns the first match!!! (ie smallest x,y values)   
        """
        if self.reference:
            return self.position
        if self.areaWasEstimated:
            if (self.getPositionBelief() is not np.nan):
                y,x =  np.unravel_index(self.predictionMap.argmax(),self.predictionMap.shape)
                return (x,y,self.z)
            else: # Incorrect estimation!!
                self.areaWasEstimated = False
        return None
    
    def getPositionBelief(self):
        """
        Returns the belief of the cell with the highest belief. 
        Note if two cells are tie, it returns the first match!!! (ie smallest x,y values)   
        """
        if self.reference:
            return 1
        return self.predictionMap.max()
        
    def getEstimatedArea(self,threshold=Config.estimatedAreaThreshold):
        """
        Returns all of the cells with a belief higher than the threshold   
        """
        if self.reference:
            return [self.position]
        
        cells = []
        for cell in self.tracker.grid.getCells():
            if self.predictionMap[cell] > threshold:
                cells.append(cell)
        return cells
    
    def getBelief(self,cell):
        """ Returns the previously calculated belief that this tag is in this cell"""
        return self.predictionMap[cell]
        
    def getReaderIds(self):
        """Returns a list of reader IDs that have reported on RSSI measurements on this Tag"""
        return self.meas.keys()
    
    def log(self,msg,tag="RFIDTag"):
        print "[%s][%s]%s"%(datetime.now().strftime("%H:%M:%S.%f"),tag,msg)
        
class Reader(Reader):
    """ This is just a dummy class to hold the positions of the readers """
    def __init__(self,tracker,network,id,position,refTagId):
        self.id = id # Should be a concatenation of network ID, reader ID, and node ID so it is unique
        self.position = position
        self.refTagId = refTagId
        self._tracker = tracker
        self._calibrated = False
        self.tagIds = [] # list of tag's that this reader has reported on
        self.a = 1 # alpha_j: propagation constant for this reader
        self.n = 2 # n_j: path loss exponent of this reader
        
    def recalibrate(self):
        """ Recalculate the propagation constant and path loss for this reader
        
        According to (8), the propagation constant and the
        path loss exponent of reader_j can be calculated 
        using (9) and (10)
        
        """
        refTags = self._tracker.getReferenceTags()
        n = 0
        a = 0
        # avg(RSSIj(j))
        RSSIj_j = refTags[self.refTagId].meas[self.id]['mean']
        self.log("[DEBUG] Reader %s Reference tag %s avg(RSSIj_j) = %s"%(self.id,self.refTagId,RSSIj_j))
        
        # Distance from reader_j to tag_j (where tag_j has id refTagId)
        tagPos = refTags[self.refTagId].getPosition()
        readerPos = self.getPosition()
        dist_j_j =  np.sqrt((readerPos[0]-tagPos[0])**2+(readerPos[1]-tagPos[1])**2+(readerPos[2]-tagPos[2])**2)
        self.log("[DEBUG] Reader %s distj_j = %s"%(self.id,dist_j_j))
        i = 0
        for tag in refTags.values():
            
            # If not this readers reference tag, and 
            # the tag has measurements reported by this reader
            if (tag.id != self.refTagId) and (self.id in tag.getReaderIds()):
                i+=1
                # avg(RSSIj(i))
                RSSIj_i = tag.meas[self.id]['mean'] 
                self.log("[DEBUG] Reader %s Tag %s RSSIj_i = %s"%(self.id,tag.id,RSSIj_i))
                tagPos = tag.getPosition()
                dist_i_j =  np.sqrt((readerPos[0]-tagPos[0])**2+(readerPos[1]-tagPos[1])**2+(readerPos[2]-tagPos[2])**2)
                self.log("[DEBUG] Reader %s Tag %s dist_i_j = %s"%(self.id,tag.id,dist_i_j))
                
                # Eqn (9)
                n_j_i = abs((RSSIj_j-RSSIj_i)/(10*np.log(dist_i_j/dist_j_j)))
                n += n_j_i
                
                # Eqn (10)
                a += (RSSIj_i+10*n_j_i*np.log(dist_i_j))
                
        self.n = n/i
        self.a = a/i
        self._calibrated = True
#        if self.a<0:
#            self.log("[WARN] Calibration α=%s is incorrect! Check the setup! Setting to default value of α=0"%(self.a))
#            self.a = 0
#        if self.n<0 or self.n>6:
#            self.log("[WARN] Calibration n=%s is incorrect! Check the setup! Setting to default value of n=3"%(self.n))
#            self.n = 3
        
        self.log("[INFO ] Reader %s calibration: α=%s n=%s"%(self.id,self.a,self.n))
        
        pass
    
    def getPosition(self):
        return self.position
    
    def addTag(self,tagId):
        """ Add the tag to the list """
        if tagId not in self.tagIds: 
            self.tagIds.append(tagId)
    
    def calcProbOfRssi(self,sigma,rssi,x):
        """
        Perceptual Model p(RSSI't|x^t)
        
        From (4), we can see that given the actual distance
        d_PHY between the tag and the reader, the received
        signal strength Pr(d_PHY) is a Gaussian random variable
        with mean a-10nlog(d_PHY) and variance sigma^2 .
        So the probability density function (PDF) of observing a
        certain value of Pr(d_PHY) (i.e, RSSI reported by the
        reader) is given by:
        (5)    p(Pr(d_PHY)=RSSI) = 1/(sqrt(2*pi)*sigma)*exp(-(RSSI-a-10nlog(d_PHY))^2/(2*sigma^2))
        """
        d_PHY = np.sqrt((self.position[0]-x[0])**2+(self.position[1]-x[1])**2+(self.position[2]-x[2])**2)
        p = np.exp(-(rssi-self.a+10*self.n*np.log(d_PHY))**2/(2*sigma**2))/(np.sqrt(2*np.pi)*sigma)
        #self.log("[DEBUG] p(RSSI't=%s|x^t=%s)=%s where d_PHY=%s, σ=%s, a=%s,n=%s"%(rssi,x,p,d_PHY,sigma,self.a,self.n))
        if not 0<=p<=1:
            self.log("[WARN] An invalid p(RSSI't|x^t)=%s was calculated by reader!"%p)
        return p
        
    
    def getTagIds(self):
        """Returns a list of tags IDs that this reader has detected"""
        return self.tagIds
    
    def getMeasReported(self):
        """Returns the rows from the tags that have this reader id """
        # This won't work without making the tags and readers global...
        # I think they should be?
        pass
    
    def log(self,msg,tag="Reader "):
        print "[%s][%s]%s"%(datetime.now().strftime("%H:%M:%S.%f"),tag,msg)
        
    
class Grid(object):
    x = Config.gridX # number of horizontal rows
    y = Config.gridY # number of vertical rows
    
    N = x*y # Number of grid cells
    def __init__(self):
        pass
    
    def getCells(self):
        """ Returns an iterator that produces the positions of each
        grid cell... ie (0,0),(0,1),(0,2),...(0,y),...(1,0),(1,1),...(x,y)
        an iterator is used to save memory
        """
        return itertools.product(range(0,self.x),range(0,self.y))

# TODO: This guy should be a thread
class Tracker(PacketDecodedListener,threading.Thread):
    '''
    Implements the system described by "Scout: Outdoor Localization Using Active RFID Technology"
    
    1) Calibrates the propagation parameters using on-site reference tags, 
    2) Estimates the distance between the object and the readers based on a probabilistic model 
        deduced from general RF path loss model, and 
    3) Determines the target location by applying Bayesian inference.
    
    Inputs:
    1) Location of each reader (xj,yj)
    2) Location of reference tags (rxi,ryi)
    3) Grid points (gxk, gyk)
    4) Threshold P
    5) Observations
    
    '''
    log_file = "RSSI.csv"
    tag_ids = []
    queue = None
    tags = {} # Dict of all i tags detected, indexed by id
    readers = {} # Dict of all j readers
    
    stdOfDetectionArea = 0 # sigma
    
    grid = Grid()
    
    def __init__(self,network):
        threading.Thread.__init__(self)
        self.queue = Queue.Queue() # Holds all of the incoming data until this thread can decode it
        # TODO: Setup and run the tracking/localization system here
        self.network = network
        self.network.getDecoder().addPacketDecodedEventListener(self)
        
        # Define the reader network layout
        self.readers = {
                1:Reader(tracker=self,network=network,id=1,position=(9,11,7.5),refTagId=993680),
                2:Reader(tracker=self,network=network,id=2,position=(9,19,7.5),refTagId=994031),
                3:Reader(tracker=self,network=network,id=3,position=(1,19,7.5),refTagId=994031)
            }
        
        # Define the reference tags
        self.tags = {
                994031:Tag(tracker=self,id=994031,reference=True,position=(5,19,7.5)),
                993680:Tag(tracker=self,id=993680,reference=True,position=(5,11,7.5))
        } # 994031
        
    def run(self):
        self.log("Starting Tracker %s"%self)
        self.network.startAutoPolling()
        while True:
            event = self.queue.get(block=True)
            if event == 'shutdown':
                self.log('shutting down')
                return
            try:
                self.processEvent(event)
            except:
                self.log("[FATAL] %s"%traceback.format_exc())
            
            
    def stop(self):
        self.network.stopAutoPolling()
        self.queue.put("shutdown")
        self.join()
    
    
    def getReferenceTags(self):
        """ Returns all of the reference tags """
        tags = {}
        for tag in self.tags.values():
            if tag.reference:
                tags[tag.id] = tag
        return tags
    
    def getTrackedTags(self):
        """ Returns the tracked tags (ie non reference tags) """
        tags = {}
        self.log(self.tags)
        for tag in self.tags.values():
            if not tag.reference:
                tags[tag.id] = tag
        return tags
    
    def packetReady(self, event):
        """ When new measurements come in recalculate the position """
        # Log the message/command
        # Use a queue because otherwise measurements get missed while we're calculating!
        self.queue.put(event,False)
        
        
    def processEvent(self,event):
        if event.getPacket().getCommand()!=0x06:
            self.log(event.getPacket().pretty())
        self._logMeasurment(event)
        tag = None
        try: 
            # Decoding from a RFID tag
            tag = event.getPacket().getDecoding()['data']['tag']
            # If the event doesn't contain a getTagPacket cmd with a decoded value it will throw a KeyError
        except KeyError:
            return    
        
        if tag is not None:
            readerId = event.getPacket().getNodeId()
            
            # We found a new RFID tag, add it to the tracked list!
            if tag['id'] not in self.tags.keys():
                self.tags[tag['id']] = Tag(id=tag['id'],tracker=self)
                self.log("[INFO ] Tracking new tag with ID=%s!"%tag['id'])
                
            self.readers[readerId].addTag(tag['id'])
            
            # Add the measurement from the reader
            self.tags[tag['id']].addMeas(rssi=tag['rssi'],readerId=readerId)
            
            if (not self.tags[tag['id']].reference) and self.tags[tag['id']].isReady(): #and (self.tags[tag['id']].numMeas%2==0):
                self.estimateArea(tagId=tag['id'],rssi=tag['rssi'])
                
        
        
            
    def _logMeasurment(self,event):
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
                self.log(tag="Monitor",msg="[INFO ] Reader ID %s found new Tag with ID %s"%(event.getPacket().getNodeId(),tag['id']))
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
            self.log(tag="Monitor",msg="[DEBUG] UID=%s RSSI=%s "%(uid,tag['rssi']))
        except KeyError:
            pass

    def log(self,msg,tag="Tracker"):
        print "[%s][%s]%s"%(datetime.now().strftime("%H:%M:%S.%f"),tag,msg)

    
    def _waitForCalibration(self):
        """ Waits until all the readers get a full window of measurements on the reference tags"""
        pass
    
    
    def estimateArea(self,tagId,rssi):
        """ Output: estimated area of the object.
        This should be called every time a tag receives 
        a new measurement.
        
        2) Determine the basic detection area: Use the
        observations of the object as defined in (12). The basic
        detection area is determined by the three readers with
        the strongest average RSSI
        
        3) Calibration of the Propagation parameters: We
        use the method described in section 2.2.2 to calibrate
        the propagation parameters of the three readers in the
        basic detection area. Given [ RSSI't-w+1 , RSSI't-w+2 ,...RSSI 't ] 
        of the reference tags and the tracked objects, the propagation
        standard deviation can be calibrated by (11)
        
        4) According to the observation vector at time
        t, RSSI't , we can estimate the object's location using 
        Bayesian inference.
        
        The Bayesian inference is used in the following
        way: Whenever the sensors (i.e., the readers) provide a
        new observation at time t, RSSI't , the server first
        predicts state at time t according to the state at time t-1:
        
        (15)    Bel'(x^t) = integral of p(x^t|x^t-1)Bel(x^(t-1))dx^(t-1)
        
        where x^(t-1) represents state of the object at time t-1,
        Bel'(x^t) is called predictive belief, p(x^t|x^(t-1)) is the
        system dynamic model.
        
        Then the server corrects the predicted belief using 
        the observation at time t by:
        
        (16)    Bel(x^t) = B^t*p(RSSI't|x^t)Bel'(x^t)    
        
        where p(RSSI't|x^t) is the perceptual model which
        describes the likelihood of having observations
        RSSI't given that the object is at location x^t , B^t is a
        constant used to normalize the resulting Bel(x^t). 
        
        5) Determination of the resulting estimated area:
        After r times of Bayesian inference, the resulting
        estimated area is decided by the relative belief, Bel'(xt):
        
        (19)    Bel'(x^t) = Bel(x^t)/|max k=1..N of (Bel(x^t=C(gxk,gyk)))|
        
        where N is the number of the grid cells in the detection
        area. Given threshold P, the resulting estimated area
        comprises of the states with relative probability belief
        larger or equal to P.
        """
        
        # 2) does not apply since we're only using 3 readers...
        
        # 3) Calibration of the Propagation parameters
        self.calcDetectionAreaStd()
        for reader in self.readers.values():
            reader.recalibrate()
            
        #4) According to the observation vector at time t, RSSI 't , 
        # we can estimate the object's location using Bayesian inference.
        # Create a new prediction map
        predic = np.zeros((self.grid.x,self.grid.y))
        initialPredic = np.zeros((self.grid.x,self.grid.y))
        
        # Make a preception map for each reader for debugging purposes
        perceptualPredic = {'total':np.zeros((self.grid.x,self.grid.y))}
        for reader in self.readers.values():
            perceptualPredic[reader.id] = np.zeros((self.grid.x,self.grid.y))
            
        #if self.tags[tagId].getPositionBelief() is np.nan:
        #    self.tags[tagId].getPositionBelief()
        self.log("[INFO ] Tags last position is %s with belief %s"%(self.tags[tagId].getPosition(), self.tags[tagId].getPositionBelief()))
        
#        for cell in self.grid.getCells():
#            # The server first predicts state at time t according to the state at time t-1
#            initialBelief = self.getPredictiveBelief(cell,self.tags[tagId].getPosition(), self.tags[tagId].getPositionBelief())
#            initialPredic[cell] = initialBelief
#            
#            # Calculate the perceptual belief
#            perceptualBelief,pmap = self.getPerceptualBelief(rssi=rssi, x=cell,initialBelief=initialBelief)
#        
#            # Log the perception predictions
#            perceptualPredic['total'][cell] = perceptualBelief
#            for reader in self.readers.values():
#                perceptualPredic[reader.id][cell] = pmap[reader.id]
#                
#            # Then the server corrects the predicted belief using the observation at time t
#            predic[cell] = perceptualBelief * initialBelief
#                
#        """
#        5) Determination of the resulting estimated area:
#        After r times of Bayesian inference, the resulting
#        estimated area is decided by the relative belief,Bel'(x^t) :
#        
#        Bel'(x^t) = Bel(x^t)/|max(Bel(x^t=C(gxk,gyk)))|
#        """
#        
#        
#        # Force the maps to be in range... why aren't they???
#        np.clip(predic,0,1,out=predic)
#        np.clip(initialPredic,0,1,out=initialPredic)
#        for it in  perceptualPredic.keys():
#            np.clip(perceptualPredic[it],0,1,out=perceptualPredic[it])
#        
        
        for cell in self.grid.getCells():
            # The server first predicts state at time t according to the state at time t-1
            initialBelief = self.getPredictiveBelief(cell,self.tags[tagId].getPosition(), self.tags[tagId].getPositionBelief())
            initialPredic[cell] = initialBelief
            
            # Calculate the perceptual belief
            perceptualBelief,pmap = self.getPerceptualBelief(rssi=rssi, x=(cell[0],cell[1],self.tags[tagId].z),initialBelief=initialBelief)
        
            # Log the perception predictions
            for reader in self.readers.values():
                perceptualPredic[reader.id][cell] = pmap[reader.id]
        
        # Normalize the reader prediction maps so they have equal weight:        
        for reader in self.readers.values():
            perceptualPredic[reader.id] = perceptualPredic[reader.id]/perceptualPredic[reader.id].sum()
        
        # Determine the final prediction
        for cell in self.grid.getCells():
            perceptualPredic['total'][cell] = 1
            for reader in self.readers.values():
                perceptualPredic['total'][cell] *= perceptualPredic[reader.id][cell]
            
            # Then the server corrects the predicted belief using the observation at time t
            predic[cell] = initialPredic[cell]*perceptualPredic['total'][cell]
                
        """
        5) Determination of the resulting estimated area:
        After r times of Bayesian inference, the resulting
        estimated area is decided by the relative belief,Bel'(x^t) :
        
        Bel'(x^t) = Bel(x^t)/|max(Bel(x^t=C(gxk,gyk)))|
        """
        
        # Normalize the final prediction map according to (19):
        # Add the maps to the Tag object for debugging purposes
        self.tags[tagId].setPredictionMap(predic) # Note; this will update the Tag.getPosition()
        self.tags[tagId].setInitialPredicMap(initialPredic)
        self.tags[tagId].setPerceptualPredicMaps(perceptualPredic)
        self.log("[INFO ] Tag %s estimated at cell %s with %s belief"%(tagId,self.tags[tagId].getPosition(),self.tags[tagId].getPredictionMap().max()))
        
        # Update the position of the tag if it's above the threshold or better than the previous
        #if (self.tags[tagId].positionBelief is None) or  (belmax>self.tags[tagId].positionBelief):
        #    self.log("[INFO ] Updating Tag %s estimated position to %s"%(tagId,mostProbableCell))
        #    self.tags[tagId].setPosition(mostProbableCell,belmax)
        
        
    
    def calcDetectionAreaStd(self):
        """
        Determines the calibration parameters sigma.
        
        The standard deviation of detection area can be calculated by:
        sigma = sqrt(1/(Nr*No)*[for j in range(1,Nr): for i in range(1,No): std(RSSIj(i)]^2)
        where Nr and No denotes the number of the readers and the tags respectively.
        
        This doesn't work for the case when there is no deviation!
        
        """ 
#        totalStd = 0
#        for tag in self.tags.values(): # No
#            for row in tag.meas.values(): # Nr
#                totalStd +=row['std']**2 # std(RSSIj(i))**2
#        q =  np.sqrt(totalStd/(len(self.tags.keys())*len(self.readers)))
        data = None
        for tag in self.tags.values(): # No
            for row in tag.meas.values(): # Nr
                if data is None:
                    data = row['data']
                else:
                    data = np.concatenate((data,row['data']))
        q = data.std()
        self.stdOfDetectionArea = q#data.std()
        self.log(msg="[INFO ] Detection area σ=%s"%q)
        return q
        
    
    def getPerceptualBelief(self,rssi,x,initialBelief):
        '''
        Belief Bel(x^t) : At time t, a probability
        distribution over possible value of x t , called Belief,
        represents the uncertainty. Bel(x^t=C(gx,gy)) represents
        the probability that the object is located within grid cell
        C(gx,gy) at time t. This probability will be computed by
        using a series of observations.
        
        Initialization: The initial belief is given by:
        Bel(x0)={1/N ,1/N ,1/N,... }
        where N is the number of grid cells, and they all are equally probable.
        
        
        4c) Calculating belief Bel(x^t):
        We calculate the probability belief Bel(x^t) using
        (16). Bel'(x^t) is obtained by (17) and p(RSSI't|x^t) is
        obtain by (18), B^t is a constant for time-t which is used
        to normalize the resulting Bel(x^t) . In our algorithm, we
        use grid point (gxk,gyk) to calculate the
        belief Bel(x^t=C(gxk ,gyk))
        
        (16)    Bel(x^t) = B^t*p(RSSI't|x^t)Bel'(x^t)    
        
        where p(RSSI't|x^t) is the perceptual model which
        describes the likelihood of having observations
        RSSI't given that the object is at location x^t , B^t is a
        constant used to normalize the resulting Bel(x^t).
        
        '''
        B = 1 #what??? is B
        p,pmap = self.pdfPerceptualModel(rssi,x)
        bel = B*p
        #self.log("[DEBUG] Perceptual belief: p(%s|%s)=%s\tInitialBelief: %s\tBeta:%s\tBelief:%s"%(rssi,x,p,initialBelief,B,bel))
        return bel,pmap
    
    def getPredictiveBelief(self,x,lastCell,lastBelief):
        """
        Guess the belief of the current position based on the system dynamics model
        Bel'(x^t) is obtained by (17)
        So, according to (15), we have:
        (17)    Bel'(x^t) = integral of p(x^t|x^(t-1))Bel(x^(t-1))dx^(t-1) = Bel(x^(t-1))
        """
        #self.log("[WARN] Lastcell is %s with belief %s"%(lastCell,lastBelief))
        if lastCell is None:
            return 1/self.grid.N
        if lastBelief in [None,np.nan]:
            lastBelief = 1/self.grid.N
        p = self.pdfSystemDynamicsModel(x,lastCell)
        bel = p*lastBelief
        #self.log("[DEBUG] Predictive belief: p(%s|%s)=%s\tLastBelief: %s\tInitialBelief:%s"%(x,lastCell,p,lastBelief,bel))
        return bel
    
    def pdfSystemDynamicsModel(self,x,x0):
        """
        a) System Dynamics Model p(x^t|x^(t-1)) :
        For moving objects, especially fast moving object,
        this model depends on the object motion parameters,
        such as direction, speed, etc.
        
        For the static objects, ­ 
            (1)    p(x^t|x^(t-1)) = 1 if x^t == x^(t-1)
            (2)    p(x^t|x^(t-1)) = 0 if x^t != x^(t-1)
            
        TODO: For moving objects,
        I'm just using a normal distribution with variance of two cells
        which would be about 2m/s when this is as fast as it goes...
        """
        sigma = Config.systemDynamicsSigma
        dist = np.sqrt((x[0]-x0[0])**2+(x[1]-x0[1])**2)
        p =  np.exp(-0.5*(dist/sigma)**2)/(sigma*np.sqrt(2*np.pi))
        assert 0<=p<=1,"[FATAL] An invalid p(x^t|x^(t-1))=%s was calculated!"%p
        return p
#        if x==x0:
#            return 1
#        else:
#            return 0
    
    def pdfPerceptualModel(self,rssi,x):
        """
        4b) Perceptual Model p(RSSI't|x^t): 
        Euclidean distance between the object and the center of
        For a single reader, for example reader_1, we use the cell 
        (i.e., the grid point corresponding to the cell):
        the model described in Section 2.2.1. The PDF given 
        by (5) is used to calculate p1(RSSI1t|x^t) , reader_1's 
        perceptual model at time t. Combining the observations where 
        (gxk,gyk) are the coordinates of the grid point,
        from m readers to estimate the object's location, we use 
        the following equation: 
        
        (18)    p(RSSI't|x^t) = product of j=1..m of  p_j(RSSI_j_t|x^t)
        
        where p_j(RSSI_j_t|x^t) represents the perceptual model of
        a single reader, reader_j. For our algorithm, we use
        m=3.
        """
        p = 1
        pmap = {}
        for reader in self.readers.values():
            p0 = reader.calcProbOfRssi(self.stdOfDetectionArea,rssi,x)
            pmap[reader.id] = p0
            p *= p0
        assert 0<=p<=1,"[FATAL] An invalid p(RSSI't|x^t)=%s was calculated!"%p
        return p,pmap
