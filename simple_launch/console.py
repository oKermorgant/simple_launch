
def color(msg, color, newline = ''):
    return f'\033[{color}m{newline}[simple_launch]: {msg}\033[0m'


def info(msg):
    print(color(msg, 92))


def neutral(msg):
    print(color(msg, 0))


def warn(msg):
    print(color(msg, 93))


def error(msg):
    raise Exception(color(msg, 91, '\n'))
