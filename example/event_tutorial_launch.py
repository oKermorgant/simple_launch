# This file does the same as https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Event-Handlers.html#event-handlers-example-launch-file

from launch.actions import (EmitEvent, LogInfo)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, LocalSubstitution)
from simple_launch import SimpleLauncher
from simple_launch.events import When, OnProcessExit, OnProcessStart, OnShutdown, OnExecutionComplete, OnProcessIO


def generate_launch_description():

    sl = SimpleLauncher()
    sl.declare_arg('verbosity', 'reqres')

    turtlesim_ns = sl.declare_arg('turtlesim_ns', 'turtlesim1')
    new_background_r = sl.declare_arg('new_background_r', -1)

    # use provided background if valid
    provided_red_valid = sl.py_eval('0 <= ', new_background_r, ' <= 255')

    with sl.group(ns = turtlesim_ns):

        sim = sl.node('turtlesim', 'turtlesim_node', name='sim')

        with sl.group(when = When(sim, OnProcessStart)):
            sl.log_info('Turtlesim started, spawning turtle')
            spawn_turtle = sl.call_service('spawn', {'x': 2., 'y': 2., 'theta': 0.2},
                                           verbosity = sl.arg('verbosity'))

        with sl.group(when = When(spawn_turtle, OnExecutionComplete)):
            sl.log_info('Spawn finished')
            sl.set_parameters('sim', {'background_r': 120})

            with sl.group(if_condition = provided_red_valid, when = When(delay=2.)):
                sl.set_parameters('sim', {'background_r': new_background_r})

        with sl.group(when = When(sim, OnProcessExit)):
            sl.log_info([EnvironmentVariable(name='USER'),' closed the turtlesim window']),
            sl.add_action(EmitEvent(event=Shutdown(reason='Window closed')))

        # these actions will activate only if verbose, otherwise the spawn is silent
        with sl.group(when = When(spawn_turtle, OnProcessIO, io='stdout')):
            sl.add_action(lambda event: sl.log_info('Spawn request says "{}"'.format(
                        event.text.decode().strip())))
            sl.add_action(lambda event: LogInfo(
                    msg='Once again, spawn request says "{}"'.format(
                        event.text.decode().strip())))

        with sl.group(when = When(event = OnShutdown)):
            sl.log_info(['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')])

    return sl.launch_description()
