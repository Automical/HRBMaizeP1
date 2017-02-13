from joy import *
import ckbot.logical as L
import time
    
class ShaveNHaircutApp( JoyApp ):

  def __init__(self,shaveSpec,hairSpec,*arg,**kw):
    JoyApp.__init__(self, *arg,**kw)
    self.shaveSpec = shaveSpec
    self.hairSpec = hairSpec

  def onStart(self):
    self.c = L.Cluster(count=4)

    self.c.at.Nx4A.set_mode('Motor')
    self.c.at.Nx4F.set_mode('Motor')
    self.c.at.Nx4B.set_mode('Motor')
    self.c.at.Nx2C.set_mode('Motor')

  def onEvent(self,evt):
    if evt.type != KEYDOWN:
      return
    # assertion: must be a KEYDOWN event 
    if evt.key == K_w:
      self.c.at.Nx4A.set_torque(.1)
      self.c.at.Nx4F.set_torque(-.1)

      time.sleep(1)

      self.c.at.Nx4A.stop()
      self.c.at.Nx4F.stop()
    elif evt.key == K_s:
      self.c.at.Nx4A.set_torque(-.1)
      self.c.at.Nx4F.set_torque(.1)

      time.sleep(1)

      self.c.at.Nx4A.stop()
      self.c.at.Nx4F.stop()
    elif evt.key == K_a:
      self.c.at.Nx4B.set_torque(-.1)
      self.c.at.Nx2C.set_torque(.1)

      time.sleep(1)

      self.c.at.Nx4B.stop()
      self.c.at.Nx2C.stop()
    elif evt.key == K_d:
      self.c.at.Nx4B.set_torque(.1)
      self.c.at.Nx2C.set_torque(-.1)

      time.sleep(1)

      self.c.at.Nx4B.stop()
      self.c.at.Nx2C.stop()
    elif evt.key == K_ESCAPE:
        self.stop()

if __name__=="__main__":
  robot = None
  scr = None
  shaveSpec = "#shave "
  hairSpec = "#haircut "
  args = list(sys.argv[1:])
  while args:
    arg = args.pop(0)
    if arg=='--mod-count' or arg=='-c':
      N = int(args.pop(0))
      robot = dict(count=N)
    elif arg=='--shave' or arg=='-s':
      shaveSpec = args.pop(0)
      if shaveSpec[:1]==">": scr = {}
    elif arg=='--haircut' or arg=='-h':
      hairSpec = args.pop(0)
      if hairSpec[:1]==">": scr = {}
    elif arg=='--help' or arg=='-h':
      sys.stdout.write("""
  Usage: %s [options]
  
    'Shave and a Haircut' example of running Plan-s in parallel and in
    sequence. The example shows that plans can run in parallel, and that
    Plan behaviors (e.g. the ShaveNHaircutPlan defined here) can use other
    plans as sub-behaviors, thereby "calling them" sequentially.
    
    When running, the demo uses the keyboard. The keys are:
      's' -- start "Shave"
      'h' -- start "Haircut"
      'b' -- start "Both", calling "Shave" and "Haircut" in sequence
      'escape' -- exit program
      
    Options:      
      --mod-count <number> | -c <number>
        Search for specified number of modules at startup
      
      --shave <spec> | -s <spec>
      --haircut <spec> | -h <spec>
        Specify the output setter to use for 'shave' (resp. 'haircut')
        
        Typical <spec> values would be:
         '#shave ' -- to print messages to the terminal with '#shave ' as prefix
         '>x' -- send to Scratch sensor 'x'
         'Nx3C/@set_pos' -- send to position of CKBot servo module with ID 0x3C
        
        NOTE: to use robot modules you MUST also specify a -c option
        
    """ % sys.argv[0])
      sys.exit(1)
    # ENDS cmdline parsing loop
  
  app = ShaveNHaircutApp(shaveSpec,hairSpec,robot=robot,scr=scr)
  app.run()
