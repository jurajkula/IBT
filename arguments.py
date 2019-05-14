import getopt
import sys

from modules.errors import ERROR_ARGUMENTS


def parseArguments(argv):
    try:
        opts, args = getopt.getopt(argv, "hdm:", ["help=", "debug=", "mode="])
    except getopt.GetoptError:
        print('Wrong arguments')
        sys.exit(ERROR_ARGUMENTS)

    data = {
        'debug': False,
        'mode': 'run'
    }
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            print('HELP')
            sys.exit(0)
        elif opt in ("-d", "--debug"):
            data['debug'] = True
        elif opt in ("-m", "--mode"):
            data['mode'] = arg

    return data
