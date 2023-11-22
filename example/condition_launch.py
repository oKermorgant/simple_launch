from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    cond1 = sl.declare_arg('cond1', True)
    cond2 = sl.declare_arg('cond2', False)

    for logic in ('and', 'or'):

        combined = sl.py_eval(cond1, f' {logic} ', cond2)

        sl.log_info([f'{logic} condition is ', combined])

        with sl.group(if_condition = combined):
            sl.node('demo_nodes_cpp', 'talker', name = f'talker_{logic}')

    return sl.launch_description()
