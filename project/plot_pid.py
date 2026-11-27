"""Plot the error and control command logged by one run of the PID controller.

Usage, from the project/ directory:

    python3 plot_pid.py                      # show the plots on screen
    python3 plot_pid.py --save-dir runs/x    # also save them as PNG files
    python3 plot_pid.py --save-dir runs/x --no-show   # save only (no display needed)
    python3 plot_pid.py --data-dir runs/x    # plot an archived run
"""
import argparse
import os
import sys

STEER_FILE = 'steer_pid_data.txt'
THROTTLE_FILE = 'throttle_pid_data.txt'
STEER_PLOT = 'steer_plot.png'
THROTTLE_PLOT = 'throttle_plot.png'
PLOT_DPI = 150


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--data-dir', default='.',
                        help='directory holding the two *_pid_data.txt files (default: current directory)')
    parser.add_argument('--save-dir', default=None,
                        help='if given, save steer_plot.png and throttle_plot.png into this directory')
    parser.add_argument('--no-show', action='store_true',
                        help='do not open plot windows (use with --save-dir on a machine without a display)')
    parser.add_argument('--rows', type=int, default=-1,
                        help='plot only the first N rows (default: all)')
    return parser.parse_args(argv)


def read_steer_data(data_dir):
    import pandas as pd
    steer_file = os.path.join(data_dir, STEER_FILE)
    steer_df = pd.read_csv(steer_file, sep=r'\s+', header=None, usecols=[0, 1, 2])
    steer_df.columns = ['Iteration', 'Error Steering', 'Steering Output']
    print(f'Steer data:\n{steer_df.head()}\n')
    return steer_df


def read_throttle_data(data_dir):
    import pandas as pd
    throttle_file = os.path.join(data_dir, THROTTLE_FILE)
    throttle_df = pd.read_csv(throttle_file, sep=r'\s+', header=None, usecols=[0, 1, 2, 3])
    throttle_df.columns = ['Iteration', 'Error Throttle', 'Brake Output', 'Throttle Output']
    print(f'Throttle data:\n{throttle_df.head()}\n')
    return throttle_df


def plot_data(df, n_rows, title, save_path):
    """Plot every column after the first against the iteration number."""
    df2 = df if n_rows < 0 else df[:n_rows]
    axes = df2.plot(x=df.columns[0], y=list(df.columns[1:]), kind='line', title=title)
    axes.grid(True, alpha=0.3)
    if save_path is not None:
        axes.get_figure().savefig(save_path, dpi=PLOT_DPI, bbox_inches='tight')
        print(f'Saved {save_path}')


def main(argv=None):
    args = parse_args(argv)

    import matplotlib
    if args.no_show:
        # A backend that needs no display, so this also works over SSH.
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    try:
        steer_df = read_steer_data(args.data_dir)
        throttle_df = read_throttle_data(args.data_dir)
    except (OSError, ValueError) as error:
        # ValueError covers an empty or malformed data file.
        print(f'Error: could not read the run data in "{args.data_dir}": {error}', file=sys.stderr)
        return 1

    steer_path = None
    throttle_path = None
    if args.save_dir is not None:
        os.makedirs(args.save_dir, exist_ok=True)
        steer_path = os.path.join(args.save_dir, STEER_PLOT)
        throttle_path = os.path.join(args.save_dir, THROTTLE_PLOT)

    plot_data(steer_df, args.rows, 'Steering: heading error and steer command', steer_path)
    plot_data(throttle_df, args.rows, 'Throttle: speed error, brake and throttle commands', throttle_path)

    if not args.no_show:
        plt.show()
    return 0


if __name__ == '__main__':
    sys.exit(main())
