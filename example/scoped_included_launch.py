from simple_launch import SimpleLauncher
import os

root = os.path.abspath(os.path.dirname(__file__))

sl = SimpleLauncher()
sl.declare_arg('which', '')
sl.declare_arg('prefix', '')


def launch_setup():

    which = sl.arg('which')

    print(f'{which}: prefix = {sl.arg("prefix")}')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
