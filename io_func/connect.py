import configparser

def connection(config):
    self.config = config
    self.config = configparser.ConfigParser()
    self.config.read("./config.ini")

    self.cam = C.fetchClass(self.config['hardware']['camera'], self.config)
    self.robot = R.fetchClass(self.config['hardware']['robot'], self.config['robot'])
    self.turntable = None

    
