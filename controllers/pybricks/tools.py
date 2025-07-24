# SPDX-License-Identifier: MIT
# Copyright (c) 2018-2020 The Pybricks Authors
from pybricks.robot import RobotSingleton as _Robot
import time 

"""Common tools for timing and data logging."""


def wait(time):
    """Pauses the user program for a specified amount of time.

    Arguments:
        time (:ref:`time`): How long to wait.

    """
    robot = _Robot().get_robot()
    start_time = robot.getTime() * 1000.0
    time_step = _Robot().get_time_step()
    while robot.getTime() * 1000.0 - start_time <= time:
        robot.step(time_step)

class StopWatch:
    """A stopwatch to measure time intervals. Similar to the stopwatch
    feature on your phone."""

    def __init__(self):
        self.robot = _Robot().get_robot()
        self.previous_time = self.robot.getTime() * 1000.0
        self.paused = False
        self.paused_time = 0.0
        self.total_paused_time = 0.0
        self.reset_after_pause = False
        self.current_time = 0.0
        self.elapsed_time = 0.0

    def time(self):
        """Gets the current time of the stopwatch.

        Returns:
            :ref:`time`: Elapsed time.
        """
        if not self.paused:
            self.current_time = self.robot.getTime() * 1000.0 
            self.elapsed_time += (self.robot.getTime() * 1000.0 - self.previous_time)
            self.previous_time = self.current_time
        
        return round(self.elapsed_time)

    def pause(self):
        """Pauses the stopwatch."""
        self.paused = True
        
    def resume(self):
        """Resumes the stopwatch."""
        self.paused = False
        self.previous_time = self.robot.getTime() * 1000.0

    def reset(self):
        """Resets the stopwatch time to 0.

        The run state is unaffected:

        * If it was paused, it stays paused (but now at 0).
        * If it was running, it stays running (but starting again from 0).
        """
        self.elapsed_time = 0.0

class DataLog:
    """Create a file and log data."""
 
    def __init__(self, *headers, name='log', timestamp=True, extension='csv', append=False):
        """
        Arguments:
            headers (`col1`, `col2`, `...`): Column headers. These are the
                names of the data columns. For example, choose ``'time'`` and
                ``'angle'``.
            name (str): Name of the file.
            timestamp (bool): Choose ``True`` to add the date and time to the
                file name. This way, your file has a unique name.
                Choose ``False`` to omit the timestamp.
            extension (str): File extension.
            append (bool): Choose ``True`` to reopen an existing data log file
                and append data to it. Choose ``False`` to clear existing
                data. If the file does not exist yet, an empty file will be
                created either way.
        """
        pass 
        # Untested
        
        # self.filename = name
        # if timestamp:
        #     import datetime
        #     timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        #     self.filename += f"_{timestamp_str}"
        # self.filename += f".{extension}"
        
        # self.headers = headers
        # self.append = append

        # if not self.append:
        #     with open(self.filename, 'w') as file:
        #         file.write(','.join(headers) + '\n')

    def log(self, *values):
        """Saves one or more values on a new line in the file.

        Arguments:
            values (object, object, `...`): One or more objects or values.
        """
        pass
        # Untested 

        # with open(self.filename, 'a') as file:
        #     file.write(','.join(str(v) for v in values) + '\n')

