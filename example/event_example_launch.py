from simple_launch import SimpleLauncher
from simple_launch.events import When, OnProcessStart, OnProcessExit


def generate_launch_description():

    sl = SimpleLauncher()
    sl.declare_arg('delay', 4., description = 'Delay to start the last node [s]')

    # small wrapper around a node execution
    # this node will wait for some delay before exit
    def waiting_node(name, delay, what = ''):
        return sl.node('simple_launch', 'example_waiting.py', name = name,
                       parameters = {'delay': float(delay) if isinstance(delay, int) else delay,
                                     'what': what})

    node = waiting_node('main', 10)

    with sl.group(when = When(node, OnProcessStart, 1.)):
        other_node = waiting_node('other_node', 3,
                    '1 s When main node starts')

    with sl.group(ns = 'last_ns', when = When(other_node, OnProcessExit, sl.arg('delay'))):
        last = waiting_node('last', 2,
                    'When other_node exits and should terminate with main if delay = 4')

    # TODO other events depending on e.g. last

    return sl.launch_description()
