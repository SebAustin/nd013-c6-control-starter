"""Overlay several archived tuning runs on one plot.

Each run is a directory made by archive_run.sh. The plot has two panels: the
error of the chosen controller, and its control command. Use it to show the
effect of changing one gain while the others stay fixed.

Usage, from the project/ directory:

    python3 plot_compare.py --controller throttle --out runs/compare_throttle_kp.png \\
        runs/<run A> runs/<run B> runs/<run C>
"""
import argparse
import math
import os
import sys

# controller name -> (data file, index of the error column, index of the command column)
DATA_LAYOUT = {
    'steer': ('steer_pid_data.txt', 1, 2),
    # The throttle file holds: iteration, error, brake, throttle.
    'throttle': ('throttle_pid_data.txt', 1, 3),
}
BRAKE_COLUMN = 2
GAINS_FILE = 'pid_gains.txt'
PLOT_DPI = 150
FIGURE_SIZE = (10, 7)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--controller', required=True, choices=sorted(DATA_LAYOUT))
    parser.add_argument('--out', required=True, help='path of the PNG file to write')
    parser.add_argument('--title', default=None, help='plot title (default: derived from the controller)')
    parser.add_argument('runs', nargs='+', help='run directories to compare')
    return parser.parse_args(argv)


def read_rows(path):
    """Return the rows of a data file as lists of floats, skipping blank lines."""
    rows = []
    with open(path) as data_file:
        for line_number, line in enumerate(data_file, start=1):
            fields = line.split()
            if not fields:
                continue
            try:
                row = [float(field) for field in fields]
            except ValueError:
                raise ValueError(f'{path} line {line_number}: not a row of numbers: {line.strip()!r}')
            if not all(math.isfinite(value) for value in row):
                raise ValueError(f'{path} line {line_number}: contains a non-finite number')
            rows.append(row)
    if not rows:
        raise ValueError(f'{path}: no data rows')
    return rows


def read_series(run_dir, controller):
    """Return (iterations, errors, commands) for one run.

    For the throttle controller the command is throttle minus brake, which
    restores the single signed command the controller produced.
    """
    file_name, error_column, command_column = DATA_LAYOUT[controller]
    rows = read_rows(os.path.join(run_dir, file_name))
    min_columns = command_column + 1
    if any(len(row) < min_columns for row in rows):
        raise ValueError(f'{run_dir}/{file_name}: expected at least {min_columns} columns per row')

    iterations = [row[0] for row in rows]
    errors = [row[error_column] for row in rows]
    if controller == 'throttle':
        commands = [row[command_column] - row[BRAKE_COLUMN] for row in rows]
    else:
        commands = [row[command_column] for row in rows]
    return iterations, errors, commands


def read_gains_label(run_dir, controller):
    """Return a legend label such as 'Kp=0.3 Ki=0 Kd=0.1' for one run."""
    gains_path = os.path.join(run_dir, GAINS_FILE)
    try:
        with open(gains_path) as gains_file:
            for line in gains_file:
                fields = line.split('#', 1)[0].split()
                if len(fields) == 4 and fields[0] == controller:
                    return f'Kp={fields[1]} Ki={fields[2]} Kd={fields[3]}'
    except OSError:
        pass
    # No usable gains file: fall back to the directory name.
    return os.path.basename(os.path.normpath(run_dir))


def main(argv=None):
    args = parse_args(argv)

    try:
        runs = [(read_gains_label(run_dir, args.controller),) + read_series(run_dir, args.controller)
                for run_dir in args.runs]
    except (OSError, ValueError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1

    import matplotlib
    matplotlib.use('Agg')  # Writes a file; needs no display.
    import matplotlib.pyplot as plt

    error_name = 'Heading error (rad)' if args.controller == 'steer' else 'Speed error (m/s)'
    command_name = 'Steer command' if args.controller == 'steer' else 'Throttle command (negative = brake)'
    figure, (error_axes, command_axes) = plt.subplots(2, 1, sharex=True, figsize=FIGURE_SIZE)
    for label, iterations, errors, commands in runs:
        error_axes.plot(iterations, errors, label=label)
        command_axes.plot(iterations, commands, label=label)

    error_axes.set_ylabel(error_name)
    error_axes.axhline(0.0, color='black', linewidth=0.5)
    error_axes.grid(True, alpha=0.3)
    error_axes.legend()
    command_axes.set_ylabel(command_name)
    command_axes.set_xlabel('Iteration')
    command_axes.axhline(0.0, color='black', linewidth=0.5)
    command_axes.grid(True, alpha=0.3)
    figure.suptitle(args.title or f'{args.controller.capitalize()} controller: comparison of tuning runs')

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    figure.savefig(args.out, dpi=PLOT_DPI, bbox_inches='tight')
    print(f'Saved {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
