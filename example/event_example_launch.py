from simple_launch import SimpleLauncher
from simple_launch.events import When, OnProcessStart, OnProcessExit, OnProcessIO, OnIncludeLaunchDescription


def generate_launch_description():

    sl = SimpleLauncher()
    sl.declare_arg('delay', 4., description = 'Delay to start the last node [s]')

    # small wrapper around a node execution
    def waiting_node(name, delay, what = ''):
        if isinstance(delay, int):
            delay = float(delay)
        return sl.node('simple_launch', 'example_waiting.py', name = name,
                       parameters = {'delay': delay,
                                     'what': what})

    node = waiting_node('main', 10)

    with sl.group(when = When(OnProcessStart, node, 1.)):
        second = waiting_node('second', 3,
                    '1 s after main node starts')

    with sl.group(when = When(OnProcessExit, second, sl.arg('delay'))):
        last = waiting_node('last', 2,
                    'after second exits and should terminate with main if delay = 4')

    # TODO OnProcessIO crashes somehow
    #with sl.group(when = When(OnProcessIO, node, io='stdout')):
        #waiting_node('io', 2,
                    #'as soon as first node prints anything')

    return sl.launch_description()
