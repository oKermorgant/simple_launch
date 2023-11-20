# This file does the same as https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Event-Handlers.html#event-handlers-example-launch-file


from launch.actions import (EmitEvent, LogInfo)
from launch.event_handlers import (OnExecutionComplete, OnProcessIO)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, LocalSubstitution)
from simple_launch import SimpleLauncher
from simple_launch.events import After, OnProcessExit, OnProcessStart, OnShutdown


def generate_launch_description():

    sl = SimpleLauncher()
    verbose = sl.declare_arg('verbose', True)

    turtlesim_ns = sl.declare_arg('turtlesim_ns', 'turtlesim1')
    sl.declare_arg('use_provided_red', False)
    sl.declare_arg('new_background_r', 200)

    with sl.group(ns = turtlesim_ns):

        sim = sl.node('turtlesim', 'turtlesim_node', name='sim')

        with sl.group(when = After(sim, OnProcessStart)):
            sl.log_info('Turtlesim started, spawning turtle')
            spawn_turtle = sl.call_service('spawn', {'x': 2., 'y': 2., 'theta': 0.2},
                                           verbose = sl.arg('verbose'))

        with sl.group(when = After(spawn_turtle, OnExecutionComplete)):
            sl.log_info('Spawn finished')
            sl.set_parameters('sim', {'background_r': 120})

            with sl.group(if_arg = 'use_provided_red', when = After(delay=2.)):
                sl.set_parameters('sim', {'background_r': sl.arg('new_background_r')})

        with sl.group(when = After(sim, OnProcessExit)):
            sl.log_info([EnvironmentVariable(name='USER'),' closed the turtlesim window']),
            sl.add_action(EmitEvent(event=Shutdown(reason='Window closed')))

        # this triggers will activate only if verbose
        with sl.group(when = After(spawn_turtle, OnProcessIO, io='stdout')):
            sl.add_action(lambda event: sl.log_info('Spawn request says "{}"'.format(
                        event.text.decode().strip())))
            sl.add_action(lambda event: LogInfo(
                    msg='Spawn request really says "{}"'.format(
                        event.text.decode().strip())))

        with sl.group(when = After(event = OnShutdown)):
            sl.log_info(['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')])

    return sl.launch_description()
