# file simTagStreamer.py simulates a robot in an arena

from sensorPlanTCP import SensorPlanTCP
from robotSim import DummyRobotSim, BigSlamSim
from joy import *
from joy.decl import *
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )


class Square(Plan):
  """Moves robot in square"""
  def __init__(self, app, robot):
    Plan.__init__(self,app)
    self.robot = robot

  def behavior(self):
    progress("square plan started")
    
    self.robot.moveForward(1)
    yield self.forDuration(4)
    self.robot.moveSide(1)
    yield self.forDuration(4)
    self.robot.moveForward(-1)
    yield self.forDuration(4)
    self.robot.moveSide(-1)
    yield self.forDuration(4)
    yield progress("square plan ending")

class FollowWaypoints(Plan):
  """Moves robot in square"""
  def __init__(self, app, robot, sensor):
    Plan.__init__(self,app)
    self.robot = robot
    self.sensor = sensor
    self.conversion = .033
    self.currentX = 0
    self.currentY = -115
    self.stepSize = .5
    self.leeway = self.stepSize / self.conversion / 2 + 1

  def behavior(self):
    progress("***STARTING WAYPOINT FOLLOW***")
    ts,w = self.sensor.lastWaypoints
    #progress(str(w[0]))

    for i in w:
      target_wp = i
      while abs(target_wp[0] - self.currentX) > self.leeway:
        direction = (target_wp[0] - self.currentX) / abs(target_wp[0] - self.currentX)
        self.robot.moveSide(direction * self.stepSize)
        self.currentX += direction * self.stepSize / self.conversion
        yield self.forDuration(4)

      
      while abs(target_wp[1] - self.currentY) > self.leeway:
        direction = (target_wp[1] - self.currentY) / abs(target_wp[1] - self.currentY)
        self.robot.moveForward(direction * self.stepSize)
        self.currentY += direction * self.stepSize / self.conversion
        yield self.forDuration(4)

    yield progress("***ENDING WAYPOINT FOLLOW***")
    

class RobotSimulatorApp( JoyApp ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the DummyRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.
     
     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """    
  def __init__(self,wphAddr=WAYPOINT_HOST,*arg,**kw):
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml", *arg, **kw
      ) 
    self.srvAddr = (wphAddr, APRIL_DATA_PORT)
    
  def onStart( self ):
    # Set up socket for emitting fake tag messages
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
    s.bind(("",0))
    self.sock = s
    # Set up the sensor receiver plan
    self.sensor = SensorPlanTCP(self,server=self.srvAddr[0])
    self.sensor.start()
    # Set up robot and its associated plans
    self.robSim = BigSlamSim(fn=None)
    self.square = Square(self, self.robSim)
    self.follow = FollowWaypoints(self, self.robSim, self.sensor)

    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/20.0)
    progress("Using %s:%d as the waypoint host" % self.srvAddr)
    self.T0 = self.now

  def showSensors( self ):
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
    else:
      progress( "Waypoints: << no reading >>" )
  
  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    # Send message to waypointServer "as if" we were tagStreamer
    self.sock.sendto(msg, self.srvAddr)
    
  def onEvent( self, evt ):
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus(): 
      self.showSensors()
      progress( self.robSim.logLaserValue(self.now) )
      # generate simulated laser readings
    elif self.timeForLaser():
      self.robSim.logLaserValue(self.now)
    # update the robot and simulate the tagStreamer
    if self.timeForFrame(): 
      self.emitTagMessage()

    if evt.type == KEYDOWN:
      if evt.key == K_UP:
        self.robSim.moveForward(0.5)
        return progress("(say) Move forward")
      elif evt.key == K_DOWN:
        self.robSim.moveForward(-0.5)
        return progress("(say) Move back")
      elif evt.key == K_LEFT:
        self.robSim.moveSide(-0.5)
        return progress("(say) Move left")
      elif evt.key == K_RIGHT:
        self.robSim.moveSide(0.5)
        return progress("(say) Move right")
      elif evt.key == K_s:
        self.square.start()
      elif evt.key == K_SPACE:
        self.follow.start()
        return
    # Use superclass to show any other events
      else:
        return JoyApp.onEvent(self,evt)
    return # ignoring non-KEYDOWN events

if __name__=="__main__":
  print """
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer. 
  """
  import sys
  if len(sys.argv)>1:
      app=RobotSimulatorApp(wphAddr=sys.argv[1], cfg={'windowSize' : [160,120]})
  else:
      #app=RobotSimulatorApp(wphAddr=WAYPOINT_HOST, cfg={'windowSize' : [160,120]})
      app=RobotSimulatorApp(wphAddr="127.0.0.1", cfg={'windowSize' : [160,120]})
  app.run()

