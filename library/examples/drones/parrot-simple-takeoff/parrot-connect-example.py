from libstp_drones.parrot.Bebop import Bebop

bebop = Bebop()
bebop.connect(10)
bebop.safe_takeoff(10)
bebop.safe_land(10)
bebop.disconnect()
