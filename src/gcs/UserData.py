import os

CONFIGURATION_FILE_NAME = 'minigcs.conf'

class UserData:

    def __init__(self):
        pass

    def loadGCSConfiguration(self):
        pass

    def saveGCSConfiguration(self):
        pass

    def defaultConfigurationFileDirectory(self):
        if os.name == 'nt':
            return os.getenv('APPDATA')
        if os.name == 'posix':
            return os.getenv('HOME')
        return None
