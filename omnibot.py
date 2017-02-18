import time
import ckbot.logical as L
c = L.Cluster(count=2)

SPEED = 0.5
DIRECTION = 1

c.at.Nx4A.set_mode('Motor')
c.at.Nx4F.set_mode('Motor')
c.at.Nx4A.set_torque(0.1 * SPEED * DIRECTION)
c.at.Nx4F.set_torque(-0.1 * SPEED * DIRECTION)

time.sleep(5)

c.at.Nx4A.stop()
c.at.Nx4F.stop()