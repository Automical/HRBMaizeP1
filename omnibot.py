import time
import ckbot.logical as L
c = L.Cluster(count=2)

c.at.Nx4A.set_mode('Motor')
c.at.Nx4F.set_mode('Motor')
c.at.Nx4A.set_torque(.1)
c.at.Nx4F.set_torque(-.1)

time.sleep(2)

c.at.Nx4A.stop()
c.at.Nx4F.stop()