import os
import json

CONFIGURATION_FILE_NAME = 'minigcs.json'

class UserData:

    __singleton = None

    @staticmethod
    def getInstance():
        if UserData.__singleton == None:
            UserData()
        return UserData.__singleton

    def __init__(self):
        if UserData.__singleton == None:
            self.userData = None
            self.confDir = self.defaultConfigurationFileDirectory()
            UserData.__singleton = self
        else:
            raise Exception('Call UserData.getInstance() instead.')

    def loadGCSConfiguration(self):
        fullPath = os.path.join(self.confDir, CONFIGURATION_FILE_NAME)
        if os.path.isfile(fullPath):
            dataFile = open(fullPath, 'r')
            self.userData = json.load(dataFile)
            dataFile.close()
        else:
            # Load empty configuration
            self.userData = {}

    def saveGCSConfiguration(self):
        dataFile = open(os.path.join(self.confDir, CONFIGURATION_FILE_NAME), 'w')
        json.dump(self.userData, dataFile)
        dataFile.close()

    def defaultConfigurationFileDirectory(self):
        if os.name == 'nt':
            return os.getenv('APPDATA')
        if os.name == 'posix':
            return os.getenv('HOME')
        return None

    def setUserDataEntry(self, key, value):
        oldValue = None if key not in self.userData else self.userData[key]
        self.userData[key] = value
        return oldValue

    def getUserDataEntry(self, key, defaultValue = None):
        return defaultValue if key not in self.userData else self.userData[key]

    @staticmethod
    def getParameterValue(params, key, defaultValue = None):
        if key in params:
            return params[key]
        params[key] = defaultValue
        return defaultValue
