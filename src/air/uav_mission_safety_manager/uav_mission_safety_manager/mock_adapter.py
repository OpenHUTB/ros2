class MockAdapter:
    """Offline adapter used before AirSim is reachable."""
    def __init__(self): self.commands = []
    def connect(self): self.commands.append('connect'); return True
    def arm(self): self.commands.append('arm'); return True
    def takeoff(self): self.commands.append('takeoff'); return True
    def goto(self, x, y, z): self.commands.append(('goto', x, y, z)); return True
    def pause(self): self.commands.append('pause')
    def resume(self): self.commands.append('resume')
    def return_home(self): self.commands.append('return_home')
    def land(self): self.commands.append('land')
