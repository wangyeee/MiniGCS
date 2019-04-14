import os
import json

CONFIGURATION_FILE_NAME = 'minigcs.json'

class UserData:

    __singleton = None

    userData = {}
    confDir = None

    @staticmethod
    def getInstance():
        if UserData.__singleton == None:
            UserData()
        return UserData.__singleton

    def __init__(self):
        if UserData.__singleton == None:
            self.confDir = self.defaultConfigurationFileDirectory()
            print('Conf Dir:', self.confDir)
            UserData.__singleton = self
        else:
            raise Exception('Call UserData.getInstance() instead.')

    def loadGCSConfiguration(self):
        dataFile = open(os.path.join(self.confDir, CONFIGURATION_FILE_NAME), 'r')
        self.userData = json.load(dataFile)
        dataFile.close()

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

    def getUserDataEntry(self, key):
        return None if key not in self.userData else self.userData[key]

# test only
def test_write():
    ud = UserData.getInstance()
    ud.userData['test'] = 'str'
    ud.userData['test_array'] = ['a', 'b', 'c']
    ud.userData['test_dict'] = {'name' : 'name_str', 'num' : 123}
    ud.saveGCSConfiguration()

def test_read():
    ud = UserData.getInstance()
    ud.loadGCSConfiguration()
    print(ud.userData)

if __name__ == '__main__':
    test_write()
    test_read()
