import sys

from arguments import parseArguments
from modules.Config import Config
from modules.Manager import Manager


def main(argv):
    data = parseArguments(argv)

    config = Config.Config(data)
    config.loadConfigFiles()
    config.loadDataFromRadarConfig()
    config.loadDataFromGlobalConfig()

    manager = Manager(config)
    manager.configureRadar()
    manager.configureCamera()

    manager.runner()


if __name__ == "__main__":
    main(sys.argv[1:])
