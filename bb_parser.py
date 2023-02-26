"""A script to programatically check the log messages of black box log(s) for anomalies.
The script passes the file(s) through the simulator preprocessing logic in order to generate the 
LogMsgs.txt files and then the text files are parsed.

Usage:

.. code-block::

   python3 bb_parser.py -d <directory containing black_box files>
   python3 bb_parser.py -f <black_box file>

   Example:
   python3 bb_parser.py -f ~/Desktop/black_box/CBNLog_0000000015.nz.bin
   python3 bb_parser.py -d ~/Desktop/black_box/

"""
from Data_Input import JSON_INPUT
import argparse
import csv
import os
from re import L
import numpy as np
from datetime import datetime
from collections import defaultdict
import matplotlib.pyplot as plt
from os import path
from PIL import Image
import sys
from operator import contains
import plotly.express as px
import plotly.io as pio
import calendar


class BB_Parser(JSON_INPUT):

    def __init__(self,json_file_path):

        super().__init__(json_file_path)
        # Valid file suffix and length for each log file
        self.VALID_FILE_EXTENSION = '.nz.bin'
        self.MIN_LOG_FILE_LENGTH = len(self.VALID_FILE_EXTENSION)
        self.UNCOMPRESSED_FILE_EXTENSION = '.bin'

        # File path of the simulator binary 
        self.ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        self.SIMULATOR_BINARY = './simulator'

        # File path of the header file containing the drop threshold
        self.DROP_THRESH_HEADER = '{}/include/robot/services/svc_prox_sensor_cache.h'.format(self.ROOT_DIR)

        # Time between any two logged data points
        self.LOGGING_FREQUENCY = 10 # ms

        # Filename for the summary txt file
        self.SUMMARY_TXT = 'bb_parser_summary.txt'

        # Filename for the summary csv file 
        self.SUMMARY_CSV = 'bb_parser_summary.csv'

        # Filename for the autonomy summary csv file 
        self.AUTONOMY_CSV = 'autonomy_summary.csv'

        # Filename for the autonomy transition dropoffs csv file 
        self.TRANSITIONS_CSV = 'transitions_summary.csv'

        # Filename for the global maps
        self.GLOBAL_RANK_MAP_NAME = 'global_rank_map.pgm'
        self.GLOBAL_TRACK_REAL_MAP_NAME = 'global_track_real_map.pgm'
        self.GLOBAL_TRACK_VIRTUAL_MAP_NAME = 'global_track_virtual_map.pgm'

        # Location of the log txt file after being run through the simulator
        self.LOG_MSGS_START_INDICATOR = 'Start MAPPING'

        # Set the column name of the time stamp field that is used in multiple errors
        self.PROX_READING_TIMESTAMP_FIELD = 'TimeInMS'

        # Define the maximum time to display logging gap error times on the histogram
        self.MAX_DISPLAY_LOGGING_GAP_TIME = 500 # ms

        # Define the time threshold before a gap in the readings is determined to be an error
        self.LOGGING_GAP_ERROR_THRESHOLD = 1.5 * self.LOGGING_FREQUENCY

        # The bin size used when displaying the timing gaps between two prox readings
        self.PROX_ERROR_BIN_SIZE = 20 # milliseconds

        # Define the time threshold required before a tof reading is considered stale
        self.STALE_TOF_READING_THRESHOLD = 100 * self.LOGGING_FREQUENCY

        # Define areas of interest surrounding NAVIGATION_FALLING
        self.NAV_FALLING_TIME_RADIUS = 5000 # the time in ms to plot data
        self.NAV_FALLING_MAX_DROP_VAL = 255 # the value in mm to set maximum drop value and scale drop state

        # Location of the robot pose data after being run through the simulator
        self.POSE_DATA_FILE = '{}/PoseData.csv'.format(self.LOG_DATA_DIR)

        # Calculate the sampling rate we want to check the robot position for any movement
        self.READING_SUBSAMPLE_TIME = 500 # ms
        self.READING_SUBSAMPLE_ITERS = int(self.READING_SUBSAMPLE_TIME / self.LOGGING_FREQUENCY)

        # Define the threshold at which we would consider a robot as stationary 
        # The robot would need to have moved at least this distance between the last two
        # sampled readings otherwise the robot is considered stationary for that amount of time
        self.STATIONARY_POSE_THRESHOLD = 30 # mm

        # Define the minimum amount of time a robot stays within the stationary pose threshold
        # until an error is raised
        self.STATIONARY_TIME_THRESHOLD = 5000 # ms
        self.STATIONARY_TIME_ITERS_THRESHOLD = int(self.STATIONARY_TIME_THRESHOLD / self.READING_SUBSAMPLE_TIME)

        # Define the maximum time to display on the histogram
        self.MAX_DISPLAY_STATIONARY_TIME = 120

        # Number of milliseconds in second
        self.MS_TO_S = 1000

        # The bin size used when displaying the times the robot was stationary in a histogram
        self.STATIONARY_ERROR_BIN_SIZE = 5

        # Location of the version data file after being run through the simulator
        self.VERSION_DATA_FILE = '{}/VersionData.txt'.format(self.LOG_DATA_DIR)

        # This string indicates the build version in the file
        self.ROBOT_BUILD_VERSION_STRING = 'Robot SW Build Version'

        # Unknown pixel values from the rank map
        self.RV_UNKNOWN = 128
        self.IMAGE_BUFFER = 20

        self.Any_Messages = 'bb_parser_typeError.txt'

        self.Event_txt = 'bb_Event_Error.txt'

        self.MIN_THRESH = 200  # threshold for 8-bit
        self.AREA_PER_CELL = 0.0016  # m^2


    def get_drop_threshold(self):
        """Retrieve the drop threshold (kDropTriggerDistanceMM) from its header file.

        Returns
        ----------
        drop_thresh : int
            The minimum value (in mm) at which drop readings (after correction)
            will fill the sensor buffer with ES_OBSTACLE

        """
        drop_thresh = self.NAV_FALLING_MAX_DROP_VAL
        with open(self.DROP_THRESH_HEADER) as f:
            header_content = f.readlines()
            for line in header_content:
                if 'kDropTriggerDistanceMM' in line:
                    try:
                        # line expected in form: 'constexpr uint32_t kDropTriggerDistanceMM = 30;  // in mm'
                        drop_thresh = int(line.split('=')[1].split(';')[0]) # value between '=' and ';'
                        break
                    except:
                        print("Cannot parse drop threshold from svc_prox_sensor_cache.h. Defaulting to {} mm.".format(self.NAV_FALLING_MAX_DROP_VAL))
                        break
        return drop_thresh

    def get_files_from_user(self, user_input):
        """Perform error checks on user inputs and get the valid file(s)
        from the user.

        Parameters
        ----------
        user_input : args object
            The user command line input

        Returns
        ----------
        bb_files : list(str)
            The list of valid black box file paths

        root_dir : str
            The root directory containing the file(s) in bb_files

        """

        # Keep track of all the files that need to be parsed
        bb_files = []
    
        # Get the root dir
        root_dir = None

        if (user_input.file):
            # Error check for the input to make sure its a valid file
            if (not path.isfile(user_input.file)):
                raise Exception("Inputted file does not exist.")
            bb_files.append(user_input.file)
            root_dir = os.path.dirname(user_input.file)
        elif (user_input.dir):
            # Error check for the input to make sure its a valid directory
            if (not path.isdir(user_input.dir)):
                raise Exception("Inputted directory does not exist.")

            # Set the root directory as the top level directory 
            root_dir = user_input.dir

            # Loop through the files in the directory and its subdirectories and only grab the valid *.bin files
            for root, _, log_files in os.walk(user_input.dir):
                for log_file in log_files:
                    if len(log_file) > self.MIN_LOG_FILE_LENGTH and log_file[-self.MIN_LOG_FILE_LENGTH:] == self.VALID_FILE_EXTENSION:
                        bb_files.append(os.path.join(root, log_file))

            # If no files were added, then the directory did not contain any valid *.bin files
            if (len(bb_files) == 0):
                raise Exception("Inputted directory does not contain any valid *.bin files.")
        else: 
            raise Exception("A valid input file or directory must be passed to the script.")

        return bb_files, root_dir


    def search_for_log_msg_errors(self, error_count, file_being_processed, summary_txt):
        """Traverses through the log file line-by-line to search for errors and 
        keeps count of how many times they appear. The function also keeps track 
        of which robot critical errors were set in the log file.

        Parameters
        ----------
        error_count : dict(str: int)
            The errors found and the count of how many times they appeared in the log file

        Returns
        ----------
        raised_errors : dict{str : int}
            A dictionary of errors set in the log file and the number of times they were raised

        nav_falling_timestamps : list[int]
            A list of the timestamps at which NAVIGATION_FALLING were triggered

        """
    
        # Keep a list of critical errors that appeared in the log
        raised_errors = defaultdict(int)

        # Keep track of the total cleaning time of the robot
        total_cleaning_time = 0

        # Keep track of whether or not a cleaning was ever started on this log
        cleaning_started = False

        # Keep a list of time stamps when NAVIGATION_FALLING was triggered
        nav_falling_timestamps = []
    
        # Timestamp at which robot detects connection with the base  
        base_reconnected_timestamp = 0

        last_robot_state_transition = 'DEFAULT_STATE'

        # Open the generated log txt file and count every instance an error on the list occurs
        # NOTE: This file is opened as a binary since there can sometimes be non-unicode characters stored in the log file
        with open(self.Log_TXT, 'rb') as fp:
            # Read each binary line
            for bline in fp:
                try:
                    # Try to decode the line. All invalid bytes will be caught and ignored
                    line = bline.decode()

                    # See if the cartridge was ever enabled on the robot to indicate a cleaning had started
                    if self.LOG_MSGS_START_INDICATOR in line:
                        cleaning_started = True

                    # Search for and log errors
                    for error_code, (error_desc, _) in self.LOG_MSGS_ERRORS.items():
                        if ("new state = " in line):
                            last_robot_state_transition = line.replace('=', ',')
                        if error_code == 'CANCEL_CLEANING_RUN':
                            # Since there are multiple ways to that signify a cancelled cleaning run, search for all possibilities
                            for desc in error_desc:
                                if desc in line:
                                    error_count[error_code] += 1
                        elif error_desc in line:
                            error_count[error_code] += 1

                            if error_code == 'SUSPENDED_AUTONOMY_STATES_RESET':
                                for state in self.AUTONOMY_SUSPENDED_STATES:
                                    error_count[state] = 0
                            elif (error_code == 'ERROR_RAISED'):
                                # The actual error being set is at the end of the line along with its severity if the logs are later than build 1.2.0-296
                                raised_error = line[line.rindex(error_desc) + len(error_desc):].strip()
                                raised_errors[raised_error] += 1
                            elif (error_code == 'TOTAL_CLEANING_TIME'):
                                # Set the total cleaning time to be the last reported total cleaning time since this is
                                # also reported for every suspended charging
                                error_count[error_code] = float(line.split(' ')[-1].strip())

                                # If there was a recorded cleaning time, then also mark the run as cleaning started
                                cleaning_started = True
                            elif (error_code == 'AUTONOMOUS_DOCK'):
                                # If the robot docked autonomously
                                error_count[error_code] = bool(line.split(' ')[-1].strip()=='true')    
                            elif (error_code == 'TOTAL_PAUSED_TIME'):
                                # Set the total paused time to be the last reported total paused time since this is
                                # reported for every instance the robot is paused
                                error_count[error_code] = float(line.split(' ')[-1].strip())
                            elif (error_code == 'NAVIGATION_FALLING'):
                                # Record the timestamp for future plotting of drop data and wheel extension data in +/-5 seconds
                                nav_falling_timestamps.append(int(line.split(' ')[4].replace('(','').replace(')','')))
                            elif (error_code == 'AVG_EXT_VOLTAGE_AFTER_RECONNECT'):
                                # Record the time stamp at which robot connected to the base
                                base_reconnected_timestamp = int(line.split(' ')[4].replace('(','').replace(')',''))
                            
                            
                except UnicodeDecodeError as e:
                    print("There was a bad line in this log file with this error: {}".format(e))

        if not cleaning_started:
            error_count['INVALID'] = 1
        else:
            if 'COMPLETE_CLEANING_RUN' in error_count and 'CANCEL_CLEANING_RUN' not in error_count:
                # If the robot returned to its start location without the run being cancelled, then we consider this a pass
                error_count['PASS'] = 1
            else:
                # If the robot was cancelled or never gave the complete cleaning run event then its a fail
                error_count['FAIL'] = 1

        # Subtract the total paused time, if available, from the total cleaning time
        total_paused_time = 0 if 'TOTAL_PAUSED_TIME' not in error_count else error_count.pop("TOTAL_PAUSED_TIME")
        error_count['TOTAL_CLEANING_TIME'] -= total_paused_time

        self.write_to_file(summary_txt, f'{file_being_processed}, last robot state = {last_robot_state_transition}')

        return raised_errors, nav_falling_timestamps, base_reconnected_timestamp

    def event_search(self):

        appending = False
        returnList = []
        with open(self.Log_TXT, "rb") as fp:
            for bline in fp:
                try:
                    line = bline.decode()
                    Split_List = line.split("\n")
                    for i in range(len(self.start_string)):
                        for index in range(len(Split_List)):
                            if self.start_string[i] in Split_List[index]:
                                appending = True
                            if self.endstring[i] in Split_List[index]:
                                if appending:
                                    returnList.append(Split_List[index])
                                appending = False
                            if appending:
                                returnList.append(Split_List[index])
                except UnicodeDecodeError as e:
                    print("There was a bad line in this log file with this error: {}".format(e))
                    
        return returnList

    def display_for_Event_alerts(self,Error_Event_alerts, summary_txt):

        if Error_Event_alerts:
            label = "\t\t\n These Event messages appeared in this log:\n"
            print(label)
            self.write_to_file(summary_txt, label)
            for message in Error_Event_alerts:
                msg = "\t\t\t* {}\n".format(message)
                print(msg)
                self.write_to_file(summary_txt, msg)


    def write_to_file(self, summary_txt, statement):
        """Writes the particular statement to the given file 
        if the file object exists

        Parameters
        ----------
        summary_txt : file obj or None
            File object to write the output of the parser or None if the user prefers console only

        statement: str
            The statement to write to the file
        """
    
        if summary_txt:
            summary_txt.write(statement)


    def display_log_msg_errors(self, raised_errors, summary_txt):
        """Print out the specific errors that appeared in the log messages

        Parameters
        ----------
        raised_errors : dict{str: int}
            A dictionary containing the errors raised in the log file as well as the number of times it occurred

        summary_txt : file obj or None
            File object to write the output of the parser or None if the user prefers console only

        """

        # Only display the log error messages if they exist
        if raised_errors:
            error_statement = "\t\t These critical errors appeared this many times:\n"
            print(error_statement)
            self.write_to_file(summary_txt, error_statement)
            for raised_error, count in raised_errors.items():
                error_statement = "\t\t\t* {} {}\n".format(raised_error, count)
                print(error_statement)
                self.write_to_file(summary_txt, error_statement)


    def display_all_errors_and_counts(self, log, error_count, summary_txt):
        """Print out the relevant errors found in the particular log file

        Parameters
        ----------
        log : str
            The filepath of the current log file being processed

        error_count : dict(str: int)
            The errors found and the count of how many times they appeared in the log file

        summary_txt : file obj or None
            File object to write the output of the parser or None if the user prefers console only

        """

        # Display the results for the particular file
        log_path = "{}: \n".format(log)
        print(log_path)
        self.write_to_file(summary_txt, log_path)

        # If the robot returned to its start location without the run being cancelled, then we consider this a pass
        if 'PASS' in error_count:
            pass_fail_statement = "\t\t ***PASS*** The robot successfully finished cleaning in this run. \n"
        elif 'FAIL' in error_count:
            pass_fail_statement = "\t\t ***FAIL*** The robot did not successfully finish this cleaning run \n"
        else:
            pass_fail_statement = "\t\t This log did not contain a valid cleaning run. \n"
        print(pass_fail_statement)
        self.write_to_file(summary_txt, pass_fail_statement)

        for error_code, count in error_count.items():
            error_statement = None
            if error_code == 'COMPLETE_CLEANING_RUN':
                error_statement = "\t\t {} The robot was able to return to its starting location in this run.\n".format(error_code)
            elif error_code == 'CANCEL_CLEANING_RUN':
                error_statement = "\t\t {} The user cancelled the cleaning in this run.\n".format(error_code)
            elif error_code == 'TOTAL_CLEANING_TIME':
                error_statement = "\t\t {} for this run was {} seconds.\n".format(error_code, count)
            elif error_code == 'AUTONOMOUS_DOCK':
                error_statement = "\t\t {}: {}.\n".format(error_code, count)
            elif error_code == 'AVG_EXT_VOLTAGE_AFTER_RECONNECT':
                error_statement = "\t\t {}: Avg Ext Voltage of 10 samples after robot is reconnected = {} mV.\n".format(error_code, count)
            elif error_code != 'PASS' and error_code != 'FAIL':
                error_statement = "\t\t {} error appeared {} time{}.\n".format(error_code, count, "" if count == 1 else "s")

            if error_statement:
                print(error_statement)
                self.write_to_file(summary_txt, error_statement)

    def search_for_any_errors_alerts(self):

        """Read through the log file and extract information about specific message"""
        am = []
        #sorted_am = None 
        with open(self.Log_TXT, "rb") as fp:
            for bline in fp:
                try:
                    line = bline.decode()
                    Split_List = line.split("\n")
                    for error_name in self.Custom_Error:
                        for index in range(len(Split_List)):
                            if error_name in Split_List[index]:
                                am.append(Split_List[index])
                                #sorted_am = sorted(am, key=lambda s: s[43:])
                except UnicodeDecodeError as e:
                    print("There was a bad line in this log file with this error: {}".format(e))
        return am

    def display_for_any_errors_alerts(self,Error_name_alerts, summary_txt):

        if Error_name_alerts:
            label = "\t\t\n These Specific messages appeared in this log:\n"
            print(label)
            self.write_to_file(summary_txt, label)
            for message in Error_name_alerts:
                msg = "\t\t\t* {}\n".format(message)
                print(msg)
                self.write_to_file(summary_txt, msg)

    def count_avoided_cells(self,tof_map_file):

        """Loop through map file and count obstacle cells"""

        # convert from binary to text
        temp_tof_map = "/tmp/global_tof_map.pgm"
        os.system(f"convert {tof_map_file} -compress none {temp_tof_map}")

        # initialize tof object cell count
        tof_cells_count = 0

        # open text file
        with open(temp_tof_map, "r") as pgmf:

            # assert it is a valid P2 file
            assert pgmf.readline() == "P2\n"

            # skip lines containing metadata
            for _ in range(3):
                next(pgmf)

            # scale threshold as necessary
            max_value = int(next(pgmf))
            min_thresh = self.MIN_THRESH * max_value / 256

            # loop through each value
            for line in pgmf:
                for value in line.split():

                    # if past threshold, count it
                    if int(value) >= min_thresh:
                        tof_cells_count += 1

        return tof_cells_count


    def calc_area_from_cells(self,num_cells):
        """Convert number of cells to area in square meters"""

        return num_cells * self.AREA_PER_CELL



    def search_for_prox_reading_errors(self, error_count, base_reconnected_timestamp):
        """Traverses through the prox readings file of the robot log to 
        identify instances in which sensor readings fail. 

        Parameters
        ----------
        error_count : dict(str: int)
            The errors found and the count of how many times they appeared in the log file

        Returns
        ----------
        logging_gap_errors : list(int)
            The length of the gaps in the prox readings log file

        i2c_bus_errors : dict{int: (str, int)}
            The instances of I2C bus errors where the key is the timestamp at which the error
            started and the value is the specific i2c error and the duration (in ms) that the error occurred

        stale_tof_readings : dict{(int, str): (int, int)}
            The instances of stale tof readings where the key is the pair of the tof sensor and the timestamp
            at which the error occured. The value is the stale value of the tof sensor and the duration
            in ms that the value was held.

        """

        # Keep track of the previous time stamp
        prev_timestamp = -1

        # Used to track the duration of the error and the last error
        i2c_bus_error_start_time = -1
        last_i2c_bus_error = None

        # Keep track of the timestamp, the type of error and the duration (in ms) of the I2C bus error
        i2c_bus_errors = {}

        # Used to track the duration of a tof reading and the last set of readings
        tof_reading_start_time = {field: -1 for field in self.PROX_ERROR_FIELDS_OF_INTEREST['STALE_TOF_READINGS']}
        last_tof_reading = {field: -1 for field in self.PROX_ERROR_FIELDS_OF_INTEREST['STALE_TOF_READINGS']}

        # Keep track of the timestamp, the prox field and the duration (in ms) of the stale reading
        stale_tof_readings = {}

        # Keep track of the instances in which there are gaps in the log and how long they occur
        logging_gap_errors = []
    
        ext_voltage_counter = 0
        avg_ext_voltage = 0

        # Iterate through the prox readings line by line to find the errors
        with open(self.CSV_File) as csvfile:
            # First open the file to get the number of rows so that we know when the last reading occurred
            reader = csv.DictReader(csvfile)
            num_rows = sum(1 for row in reader)

            # Reset the file reader and open it again to actually iterate through each row
            csvfile.seek(0)
            reader = csv.DictReader(csvfile)
            for i, row in enumerate(reader):
                cur_timestamp = int(row[self.PROX_READING_TIMESTAMP_FIELD])
                # For every error, find the relevant rows of the log file to check
                for error_code, fields in self.PROX_ERROR_FIELDS_OF_INTEREST.items():

                    if (error_code == 'EXTERNAL_VOLTAGE' and cur_timestamp > base_reconnected_timestamp):
                        ext_voltage_counter += 1
                        avg_ext_voltage += int(row[fields[0]])/10
                        if (ext_voltage_counter == 10):
                            error_count['AVG_EXT_VOLTAGE_AFTER_RECONNECT'] = int(avg_ext_voltage)    
                                                                            
                    if error_code == 'TOF_SENSOR_ERRORS':
                        cur_tof_error_reading = None
                        for field in fields:
                            # Filter out the error code from the sensor reading by taking only the 8 MSB
                        
                            tof_sensor_code = 0xFF00 & int(row[field])
                            # tof_sensor_code = (tof_sensor_code)
                            tof_sensor_code = hex(tof_sensor_code)
                            # Get the error if one is present
                            if tof_sensor_code in self.TOF_ERROR_CODES:
                                cur_tof_error_reading = self.TOF_ERROR_CODES[tof_sensor_code][0]
                        
                    
                        if last_i2c_bus_error != cur_tof_error_reading or i == (num_rows-1):
                            # If we're transitioning out of an error state, calculate the duration and save it
                            if i2c_bus_error_start_time > 0:
                                i2c_bus_errors[i2c_bus_error_start_time] = (last_i2c_bus_error, cur_timestamp - i2c_bus_error_start_time)
                                last_i2c_bus_error = cur_tof_error_reading

                                # Reset the duration timer for the I2C bus error
                                i2c_bus_error_start_time = -1

                            # Only increment the error count for every new instance of the error
                            if i2c_bus_error_start_time < 0 and cur_tof_error_reading is not None:
                                error_count[error_code] += 1
                                i2c_bus_error_start_time = cur_timestamp
                                last_i2c_bus_error = cur_tof_error_reading

                    if error_code == 'STALE_TOF_READINGS':
                        for field in fields:
                            tof_reading = int(row[field])

                            # If the reading is an error or if the reading has changed, update the previous values
                            if tof_reading != last_tof_reading[field] or tof_reading > 255 or i == (num_rows-1):
                                tof_reading_duration = cur_timestamp - tof_reading_start_time[field]

                                # If the reading was stale, keep track of it
                                if tof_reading_duration > self.STALE_TOF_READING_THRESHOLD and tof_reading_start_time[field] > 0:
                                    stale_tof_readings[(tof_reading_start_time[field], field)] = (last_tof_reading[field], tof_reading_duration)
                            
                                # Change the last tof reading and update the start timestamp
                                last_tof_reading[field] = tof_reading
                                tof_reading_start_time[field] = cur_timestamp

                    elif error_code == 'PROX_READINGS_LOGGING_GAP':
                        # NOTE: No need to use the field variable for this error since that is 
                        # taken care of with the timestamp

                        if (prev_timestamp > 0):
                            # Calculate the gap between the current and previous prox readings
                            logging_gap = cur_timestamp - prev_timestamp

                            # If the gap is larger than the LOGGING_GAP_ERROR_THRESHOLD
                            if (logging_gap > self.LOGGING_GAP_ERROR_THRESHOLD):
                                error_count[error_code] += 1
                                logging_gap_errors.append(logging_gap)
            
                # Change the previous time stamp
                prev_timestamp = cur_timestamp

        return logging_gap_errors, i2c_bus_errors, stale_tof_readings

    def new_custom_prox(self):

        Data_values = []
        keyNames = self.CUSTOM_PROX_DATA

        with open(self.CSV_File) as csvfile:
            # First open the file to get the number of rows so that we know when the last reading occurred
            reader = csv.DictReader(csvfile)
            num_rows = sum(1 for row in reader)
            # Reset the file reader and open it again to actually iterate through each row
            csvfile.seek(0)
            reader = csv.DictReader(csvfile)
            for i, row in enumerate(reader):
                prox_dict = {}
                for custom in self.CUSTOM_PROX_DATA:
                    prox_dict[custom] = row[custom]
                Data_values.append(prox_dict)
                
            #plotting
            times = []
            Columns_Name = self.CUSTOM_PROX_DATA[1:]
            proxData = []
            Condition_met = False
            # put the right amount of list in the prox data
            for i in range(len(Columns_Name)):
                proxData.append([])
            #appends data to the times and proxData list at an index corresponding to the index of the column_name
            for timeListIndex in range(len(self.startTime)):
                for i in range(len(Data_values)):
                    if int(Data_values[i]["TimeInMS"]) == self.startTime[timeListIndex]:
                        Condition_met = True
                        while int(Data_values[i]["TimeInMS"]) <= self.endTime[timeListIndex]:
                            times.append(int(Data_values[i]["TimeInMS"]))
                            for j in range(len(Columns_Name)):
                                proxData[j].append(int(Data_values[i][Columns_Name[j]]))
                            i += 1
            if Condition_met:
                plt.title("Test title)")
                plt.xlabel("TimeInMS")
                plt.ylabel("Data_values")
                for j in range(len(Columns_Name)):
                    plt.plot(times, proxData[j], label = Columns_Name[j])
                plt.legend()
                plt.autoscale(enable=True, axis='both')
                plt.savefig('{}_plot_data.png'.format(self.log))
                plt.close()

        return Data_values, keyNames

    def csv_spec_data(self):
        
        error_count_list = []
        final_list = []
        column_name = self.CSV_specfic_data
        with open(self.CSV_File) as csvfile:
            # First open the file to get the number of rows so that we know when the last reading occurred
            reader = csv.DictReader(csvfile)
            num_rows = sum(1 for row in reader)
            # Reset the file reader and open it again to actually iterate through each row
            csvfile.seek(0)
            reader = csv.DictReader(csvfile)
            #readerList = list(reader)
            
            for i, row in enumerate(reader):
                prox_dict = {}
                for custom in self.CSV_specfic_data:
                    prox_dict[custom] = row[custom]
                error_count_list.append(prox_dict)
               
            final_list.append(column_name)

            for i, val_dict in enumerate(error_count_list):
                for findVal in self.Input_values:
                    for header in column_name[1:]:
                        if int(findVal) == int(val_dict[header]):
                            temp_list = []
                            temp_list.append(error_count_list[i]["TimeInMS"]) #timeinms value
                            temp_list.append(findVal)
                            final_list.append(temp_list)

            return final_list

    def csv_error_count(self):

        error_count_list = []
        column_name = self.CSV_count
        with open(self.CSV_File) as csvfile:
            # First open the file to get the number of rows so that we know when the last reading occurred
            reader = csv.DictReader(csvfile)
            num_rows = sum(1 for row in reader)
            # Reset the file reader and open it again to actually iterate through each row
            csvfile.seek(0)
            reader = csv.DictReader(csvfile)
            
            for value in self.values:
                temp_list = [value]
                for i in range(len(column_name)-1):
                    temp_list.append(0)
                error_count_list.append(temp_list)

            for i, row in enumerate(reader):
                for j, value in enumerate(self.values):
                    for k, custom in enumerate(self.CSV_count):
                        if custom == "Value":
                            continue
                        if int(row[custom]) == int(value):
                            error_count_list[j][k] += 1

        return error_count_list, column_name

    
    def Find_State(self):

        def convertTimeString(sTime):
    
            temp = list(sTime)
            temp[4] = '-'
            temp[8] = '-'
            sTime = "".join(temp)
            sMonth = line[5:8]
            nMonth = list(calendar.month_abbr).index(sMonth)
            sMonth = str(nMonth)
            sMonth = sMonth.zfill(2)
            sTime = line[0:4] + "-" + sMonth + "-" + line[9:24]
            formatTime = datetime.strptime(sTime, '%Y-%m-%d %H:%M:%S.%f')
            return formatTime
            
        state_starts = []
        
        with open(self.Log_TXT) as logfile:
            for line in logfile:
                if(line.startswith("2022")):
                    sStartTime = convertTimeString(line[0:24])
                    break

        # Go through entire log file and extract state changes 
        with open(self.Log_TXT) as logfile:
            for line in logfile:
                # Check pattern for state changes
                if contains(line, "old state =") and contains(line, "event ="):
                    # Get converted time stamp
                    sTime = convertTimeString(line[0:24])
            
                    # Extract old state, event and new state
                    sSplit = line.split('=')
                    #sOldState = sSplit[1]
                    #sOldState = sOldState.replace(' ','')
                    #sOldState = sOldState.replace('ST_','')
                    sEvent = sSplit[3]
                    sEvent = sEvent.split(',')
                    sEvent = sEvent[0]
                    sEvent = sEvent.replace(' ','')
                    sEvent = sEvent.replace('EVENT_','')
                    sNewState = sSplit[4]
                    sNewState = sNewState.replace(' ','')
                    sNewState = sNewState.replace('ST_','')
            
                    # Save information (s)
                    state_starts.append(
                        {
                            'time': sTime,
                            'newstate': sNewState,
                            'event': sEvent
                        }
                    )

        nEvents = len(state_starts)

        # Create entry for first state in list
        # Starts at time of first log entry and ends
        # at first state change
        sFrom = sStartTime
        sTo = state_starts[0]['time']
        sEvent = state_starts[0]['event']
        sState = state_starts[0]['newstate']
        events = [
            {
                'state': sState, 
                'start': sFrom, 
                'end': sTo, 
                'event': sEvent, 
            }
        ]

        # Add a dictionary for each ne state
        # Last state might be missing
        for i in range(1,len(state_starts)):
            sFrom = state_starts[i-1]['time']
            sTo = state_starts[i]['time']
            sEvent = state_starts[i]['event']
            sState = state_starts[i]['newstate']
    
            events.append(  
            {
                'state': sState, 
                'start': sFrom, 
                'end': sTo, 
                'event': sEvent, 
            }
        )

        # Sort the list of dicts, not sure how it sorts so
        category_order = sorted(list(set([e['state'] for e in events])))

        # Create the plot with plotly express functionality
        fig = px.timeline(
            events, x_start="start", x_end="end", y="state", 
            hover_data=['event'], 
            #color='tenant', 
            height=400, width=1900, 
            category_orders={'state': category_order}
        )

        # Add a rangeslider to the plot
        fig.update_layout(xaxis=dict(rangeslider=dict(visible=True)))

        
        pio.write_html(fig, '{}_state.html'.format(self.log))

        return None
        

    def Service_state(self):

        def convertTimeString(sTime):
    
            temp = list(sTime)
            temp[4] = '-'
            temp[8] = '-'
            sTime = "".join(temp)
            sMonth = line[5:8]
            nMonth = list(calendar.month_abbr).index(sMonth)
            sMonth = str(nMonth)
            sMonth = sMonth.zfill(2)
            sTime = line[0:4] + "-" + sMonth + "-" + line[9:24]
            formatTime = datetime.strptime(sTime, '%Y-%m-%d %H:%M:%S.%f')
            return formatTime
            
        state_starts = []
        
        with open(self.Log_TXT) as logfile:
            for line in logfile:
                if(line.startswith("2022")):
                    sStartTime = convertTimeString(line[0:24])
                    break

        # Go through entire log file and extract state changes 
        with open(self.Log_TXT) as logfile:
            hasRun = False
            for line in logfile:
                # Check pattern for state changes
                if contains(line, " ===== ") and (contains(line, "Start")):
                    # Get converted time stamp
                    sTime = convertTimeString(line[0:24])

                    
            
                   
                    sSplit = line.split(' ===== ')
                    rightSplit = sSplit[1].split(":")
                    sNewState = rightSplit[0]
                    if (contains(line, "Start")):
                        startE = rightSplit[1]

                    with open(self.Log_TXT) as logfile2:
                        for line2 in logfile2:
                            newTime = convertTimeString(line2[0:24])
                            if (contains(line2, "Stop")) and contains(line2, sNewState):
                                if hasRun: 
                                    condition = False
                                    for i in range(len(state_starts)):
                                        if newTime == state_starts[i]['time']:
                                            pass
                                        else: 
                                            stopSplit = line2.split(' ===== ')
                                            rightStopSplit = stopSplit[1].split(":")
                                            stopE = rightStopSplit[1]
                                            condition = True
                                    if condition:
                                        break
                                else:
                                    stopSplit = line2.split(' ===== ')
                                    rightStopSplit = stopSplit[1].split(":")
                                    stopE = rightStopSplit[1]
                                    break
                    #elif (contains(line, "Stop")):
                        #stopE = rightSplit[1]
              
            
                    # Save information (s)
                    state_starts.append(
                        {
                            'time': sTime,
                            'newstate': sNewState,
                            'startEvent': startE,  #sEvent
                            'stopEvent': stopE
                        }
                    )
                    hasRun = True

        nEvents = len(state_starts)
      

        # Create entry for first state in list
        # Starts at time of first log entry and ends
        # at first state change
        sFrom = sStartTime
        sTo = state_starts[0]['time']
        #sEvent = state_starts[0]['event']
        stopEvent = state_starts[0]['stopEvent']
        startEvent = state_starts[0]['startEvent']
        
        
        sState = state_starts[0]['newstate']
        events = [
            {
                'state': sState, 
                'start': sFrom, 
                'end': sTo, 
                'startEvent': startEvent,
                'stopEvent': stopEvent,
                #'event': sEvent, 
            }
        ]
        
        # Add a dictionary for each ne state
        # Last state might be missing
        for i in range(1,len(state_starts)):
            sFrom = state_starts[i-1]['time']
            sTo = state_starts[i]['time']
            #sEvent = state_starts[i]['event']
            sState = state_starts[i]['newstate']
            startEvent = state_starts[i]['startEvent']
            stopEvent = state_starts[i]['stopEvent']
            
            #print(startEvent)
            #print(stopEvent)
            events.append(  
            {
                'state': sState, 
                'start': sFrom, 
                'end': sTo, 
                'startEvent': startEvent,
                'stopEvent': stopEvent,
                #'event': sEvent, 
            }
        )
        
  
        # Sort the list of dicts, not sure how it sorts so
        category_order = sorted(list(set([e['state'] for e in events])))
       
        # Create the plot with plotly express functionality
        fig = px.timeline(
            events, x_start="start", x_end="end", y="state", 
            #hover_data=events[0]['event'], 
            hover_data= ['startEvent','stopEvent'],
            #color='tenant', 
            height=1000, width=1500, 
            category_orders={'state': category_order}
        )

        # Add a rangeslider to the plot
        fig.update_layout(xaxis=dict(rangeslider=dict(visible=True)))

        # Starts local http-server and displays plot on local website
        #fig.show()
        #plotly.offline.plot(fig, f'{"state.html"}/{self.log}', auto_open=False)
        #pathway = self.log.strip('CBNL')
        #print(pathway)
        #location = (str(pathway[0]+r"\service_folder"))
        #if not os.path.exists(location):
            #os.makedirs(location )
        pio.write_html(fig, '{}service_state.html'.format(self.log))
        #pio.write_image(fig, '{}_state.png'.format(self.log))

        return None
                    
                    


    def display_logging_gap_errors(self, log_name, logging_gap_errors, summary_txt):
        """Display the lengths of times of the gaps in the prox readings log file.

        Parameters
        ----------
        log_name : str
            The name of the log file we're analyzing

        logging_gap_errors : list(int)
            The lengths of time of each gap in the log file

        summary_txt : file obj or None
            File object to write the output of the parser or None if the user prefers console only

        """
        

        # Create the list of bins for the histogram 
        bins_list = list(range(-self.PROX_ERROR_BIN_SIZE, self.MAX_DISPLAY_LOGGING_GAP_TIME + self.PROX_ERROR_BIN_SIZE, self.PROX_ERROR_BIN_SIZE))

        # Only display the log error messages if they exist
        if len(logging_gap_errors) > 0:
            # Print errors as list instead of numpy array for cleaner print statement
            prox_readings_statement = "\t\t These are the lengths of times there were gaps in the prox readings logs (in milliseconds): {}\n".format(logging_gap_errors)
            print(prox_readings_statement)
            self.write_to_file(summary_txt, prox_readings_statement)

            # Clip all times greater than MAX_DISPLAY_STATIONARY_TIME for histogram visualizations
            logging_gap_errors = np.clip(logging_gap_errors, 0, self.MAX_DISPLAY_LOGGING_GAP_TIME)

            # Calculate weights to normalize the histogram data
            weights = np.ones_like(logging_gap_errors) / float(len(logging_gap_errors))

            # Generate the plot
            plt.hist(logging_gap_errors, bins_list, weights=weights)

            plt.title(log_name)
            plt.xlabel('Time of Gap in Prox Readings in milliseconds')
            plt.ylabel('Normalized Frequency of Occurrence')

            # Save the plot in the log directory
            plt.savefig('{}/{}_logging_gap.png'.format(self.LOG_DATA_DIR, log_name))
            plt.close()


    def display_navigation_falling(self, root_dir, log_name, nav_falling_timestamps, drop_thresh, no_left=False, no_right=False):
        """Display the drop values, drop states, and wheel extensions surrounding NAVIGATION_FALLING.

        Parameters
        ----------
        root_dir : str
            The root directory that contains all of the bb_files

        log_name : str
            The name of the log file we're analyzing

        nav_falling_timestamps : list[int]
            A list of the timestamps at which NAVIGATION_FALLING were triggered

        drop_thresh : int
            The threshold at which the drop values would trigger NAVIGATION_FALLING

        no_left : boolean
            Toggle on whether to plot left drop data and wheel extension

        no_right : boolean
            Toggle on whether to plot right drop data and wheel extension

        """
        

        if len(nav_falling_timestamps) < 1:
            return
    
        if not drop_thresh:
            drop_thresh = self.get_drop_threshold()

        if no_left and no_right:
            print("\ndisplay_navigation_falling : no_left and no_right flags both provided, nothing to plot.")
            return
        elif no_left or no_right:
            num_subplots = 2
        else:
            num_subplots = 4

        timestamps_all = [ [] for timestamp in nav_falling_timestamps]
        timestamps_left_under = [ [] for timestamp in nav_falling_timestamps]
        timestamps_left_over = [ [] for timestamp in nav_falling_timestamps]
        timestamps_right_under = [ [] for timestamp in nav_falling_timestamps]
        timestamps_right_over = [ [] for timestamp in nav_falling_timestamps]

        left_drop_filtered_under = [ [] for timestamp in nav_falling_timestamps]
        right_drop_filtered_under = [ [] for timestamp in nav_falling_timestamps]
        left_drop_filtered_over = [ [] for timestamp in nav_falling_timestamps]
        right_drop_filtered_over = [ [] for timestamp in nav_falling_timestamps]

        left_drop_state = [ [] for timestamp in nav_falling_timestamps]
        right_drop_state = [ [] for timestamp in nav_falling_timestamps]

        left_wheel_extend = [ [] for timestamp in nav_falling_timestamps]
        right_wheel_extend = [ [] for timestamp in nav_falling_timestamps]

        # Record the readings within +/-NAV_FALLING_TIME_RADIUS of each occurance
        with open(self.CSV_File) as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                cur_timestamp = int(row[self.PROX_READING_TIMESTAMP_FIELD])
                for time_index, timestamp in enumerate(nav_falling_timestamps):
                    min_time = timestamp - self.NAV_FALLING_TIME_RADIUS
                    max_time = timestamp + self.NAV_FALLING_TIME_RADIUS
                    if min_time < cur_timestamp < max_time:
                        # Save the drop states and wheel extend
                        timestamps_all[time_index].append(cur_timestamp)
                        left_drop_state[time_index].append(int(row['left_drop_state'])*self.NAV_FALLING_MAX_DROP_VAL)
                        right_drop_state[time_index].append(int(row['right_drop_state'])*self.NAV_FALLING_MAX_DROP_VAL)
                        left_wheel_extend[time_index].append(int(row['left_wheel_extend']))
                        right_wheel_extend[time_index].append(int(row['right_wheel_extend']))
                        # Save left drop, separated by drop threshold for color coding
                        if int(row['left_drop_filtered']) > drop_thresh:
                            timestamps_left_over[time_index].append(cur_timestamp)
                            if int(row['left_drop_filtered']) > self.NAV_FALLING_MAX_DROP_VAL:
                                left_drop_filtered_over[time_index].append(int(self.NAV_FALLING_MAX_DROP_VAL))
                            else:
                                left_drop_filtered_over[time_index].append(int(row['left_drop_filtered']))
                        else:
                            timestamps_left_under[time_index].append(cur_timestamp)
                            left_drop_filtered_under[time_index].append(int(row['left_drop_filtered']))
                        # Save right drop, separated by drop threshold for color coding
                        if int(row['right_drop_filtered']) > drop_thresh:
                            timestamps_right_over[time_index].append(cur_timestamp)
                            if int(row['right_drop_filtered']) > self.NAV_FALLING_MAX_DROP_VAL:
                                right_drop_filtered_over[time_index].append(int(self.NAV_FALLING_MAX_DROP_VAL))
                            else:
                                right_drop_filtered_over[time_index].append(int(row['right_drop_filtered']))
                        else:
                            timestamps_right_under[time_index].append(cur_timestamp)
                            right_drop_filtered_under[time_index].append(int(row['right_drop_filtered']))

        # LEGO-4203: Evaluate the impact of adding time threshold for wheel extensions
        botvac_timestamps = []
        # loop through timestamps starting with actual trigger, watch wheel extension data
        for time_index, timestamp in enumerate(nav_falling_timestamps):
            nav_falling = True
            reset_timestamp = 0
            for row_index in list(range(len(timestamps_all[time_index]))):
                if timestamps_all[time_index][row_index] <= timestamp: # before timestamp
                    continue
                else:
                    if nav_falling:
                        if (not left_wheel_extend[time_index][row_index]) and (not right_wheel_extend[time_index][row_index]):
                            nav_falling = False
                            reset_timestamp = timestamps_all[time_index][row_index]
            botvac_timestamps.append(reset_timestamp)

        # Generate the plot(s)

        for time_index, timestamp in enumerate(nav_falling_timestamps):
            min_time = timestamp - self.NAV_FALLING_TIME_RADIUS
            max_time = timestamp + self.NAV_FALLING_TIME_RADIUS

            subplot_counter = 0
            if not no_left:
                subplot_counter += 1
                plt.subplot(num_subplots, 1, subplot_counter)
                if subplot_counter == 1:
                    plt.title('{} Nav Falling at {}'.format(log_name,timestamp))
                plt.scatter(timestamps_all[time_index], left_drop_state[time_index],s=4)
                plt.scatter(timestamps_left_under[time_index], left_drop_filtered_under[time_index],s=2, c='orange')
                plt.scatter(timestamps_left_over[time_index], left_drop_filtered_over[time_index],s=2, c='red')
                # Display maximum drop value on the right
                if len(left_drop_filtered_over[time_index]) > 0:
                    plt.text(max_time, max(left_drop_filtered_over[time_index]), '..{}mm'.format(max(left_drop_filtered_over[time_index])))
                else:
                    plt.text(max_time, max(left_drop_filtered_under[time_index]), '..{}mm'.format(max(left_drop_filtered_under[time_index])))
                plt.xlim(min_time,max_time)
                plt.ylim(0,self.NAV_FALLING_MAX_DROP_VAL)
                if subplot_counter < num_subplots:
                    plt.gca().get_xaxis().set_visible(False)
                plt.ylabel('left drop')

            if not no_right:
                subplot_counter += 1
                plt.subplot(num_subplots, 1, subplot_counter)
                if subplot_counter == 1:
                    plt.title('{} Nav Falling at {}'.format(log_name,timestamp))
                plt.scatter(timestamps_all[time_index], right_drop_state[time_index],s=4)
                plt.scatter(timestamps_right_under[time_index], right_drop_filtered_under[time_index],s=2, c='orange')
                plt.scatter(timestamps_right_over[time_index], right_drop_filtered_over[time_index],s=2, c='red')
                # Display maximum drop value on the right
                if len(right_drop_filtered_over[time_index]) > 0:
                    plt.text(max_time, max(right_drop_filtered_over[time_index]), '..{}mm'.format(max(right_drop_filtered_over[time_index])))
                else:
                    plt.text(max_time, max(right_drop_filtered_under[time_index]), '..{}mm'.format(max(right_drop_filtered_under[time_index])))
                plt.xlim(min_time,max_time)
                plt.ylim(0,self.NAV_FALLING_MAX_DROP_VAL)
                if subplot_counter < num_subplots:
                    plt.gca().get_xaxis().set_visible(False)
                plt.ylabel('right drop')

            if not no_left:
                subplot_counter += 1
                plt.subplot(num_subplots, 1, subplot_counter)
                plt.scatter(timestamps_all[time_index], left_wheel_extend[time_index],s=4)
                plt.xlim(min_time,max_time)
                plt.ylim(0,1)
                if subplot_counter < num_subplots:
                    plt.gca().get_xaxis().set_visible(False)
                # LEGO-4203: Plot time at which both wheel extensions are 0 (if any) to evaluate impact of time threshold per botvac logic
                plt.vlines(x=timestamp,ymin=0,ymax=1,linestyles='dotted',color='red')
                if botvac_timestamps[time_index]:
                    plt.vlines(x=botvac_timestamps[time_index],ymin=0,ymax=1,linestyles='dotted',color='green')
                    plt.text(botvac_timestamps[time_index], 0.5,' {}ms delay'.format(botvac_timestamps[time_index]-timestamp))
                plt.ylabel('left wheel')

            if not no_right:
                subplot_counter += 1
                plt.subplot(num_subplots, 1, subplot_counter)
                plt.scatter(timestamps_all[time_index], right_wheel_extend[time_index],s=4)
                plt.xlim(min_time,max_time)
                plt.ylim(0,1)
                if subplot_counter < num_subplots:
                    plt.gca().get_xaxis().set_visible(False)
                # LEGO-4203: Plot time at which both wheel extensions are 0 (if any) to evaluate impact of time threshold per botvac logic
                plt.vlines(x=timestamp,ymin=0,ymax=1,linestyles='dotted',color='red')
                if botvac_timestamps[time_index]:
                    plt.vlines(x=botvac_timestamps[time_index],ymin=0,ymax=1,linestyles='dotted',color='green')
                    plt.text(botvac_timestamps[time_index], 0.5,' {}ms delay'.format(botvac_timestamps[time_index]-timestamp))
                plt.ylabel('right wheel')

            # Save the plot in the log directory
            plt.savefig('{}/{}_navigation_falling_{}.png'.format(root_dir, log_name, time_index))
            plt.close()


    def display_i2c_bus_errors(self, i2c_bus_errors, summary_txt):
        """Print out the timestamp as well as the duration of the 
        I2C bus errors.

        Parameters
        ----------
        i2c_bus_errors : dict{int: (str, int)}
            The instances of I2C bus errors where the key is the timestamp at which the error
            started and the value is the specific i2c error and the duration (in ms) that the error occurred

        summary_txt : file obj or None
            File object to write the output of the parser or None if the user prefers console only

        """

        # Only display the error messages if they exist
        if i2c_bus_errors:
        
            i2c_bus_error_statement = "\t\t These I2C bus errors occured at these time stamps and for these durations: \n"
            print(i2c_bus_error_statement)
            self.write_to_file(summary_txt, i2c_bus_error_statement)
            for timestamp, (error_code, duration) in i2c_bus_errors.items():
                converted_timestamp = datetime.fromtimestamp(timestamp / self.MS_TO_S)
                i2c_bus_error_statement = "\t\t\t* {} ({}): {} ({} ms)\n".format(converted_timestamp, timestamp, error_code, duration) 
                print(i2c_bus_error_statement)
                self.write_to_file(summary_txt, i2c_bus_error_statement)


    def display_stale_tof_readings(self, stale_tof_readings, summary_txt):
        """Print out the timestamp, duration, which tof sensor, and the value
        of the stale tof sensor readings.

        Parameters
        ----------
        stale_tof_readings : dict{(int, str): (int, int)}
            The instances of stale tof readings where the key is the pair of the tof sensor and the timestamp
            at which the error occured. The value is the stale value of the tof sensor and the duration
            in ms that the value was held.

        summary_txt : file obj or None
            File object to write the output of the parser or None if the user prefers console only

        """

        # Only display the error messages if they exist
        if stale_tof_readings:
            stale_tof_error_statement = "\t\t These stale tof readings occured for these sensors at these timestamps. The stale value and the duration are also shown: \n"
            print(stale_tof_error_statement)
            self.write_to_file(summary_txt, stale_tof_error_statement)
            for (timestamp, field), (tof_reading, duration) in sorted(stale_tof_readings.items()):
                converted_timestamp = datetime.fromtimestamp(timestamp / self.MS_TO_S)
                stale_tof_error_statement = "\t\t\t* {} ({}): {} - {} ({} ms)\n".format(converted_timestamp, timestamp, field, tof_reading, duration) 
                print(stale_tof_error_statement)
                self.write_to_file(summary_txt, stale_tof_error_statement)

    def display_autonomy_percentages(self, plot_events, plot_percentages, plot_bar_texts, total_count, started_where, build):
        """Display the percentage of logs that triggered each autonomy state transition"

        Parameters
        ----------
    
        plot_events: list
            list of autonomy events for which we want to track the percentages
        plot_percentages:list
            percentages of logs which triggered a specific event
        started_where: string
            string saying if robot started "OFF_BASE" or "ON_BASE"
        build: string
            what build is the plot for

        """
        fig = plt.figure(figsize = (10, 5))
        bars = plt.bar([event.partition("(")[0] for event in plot_events], plot_percentages)
        plt.title(f'Build {build}, {started_where}, Log count: {total_count}')
        plt.ylabel('%')

        for i, rect in enumerate(bars):
            height = rect.get_height()
            plt.text(rect.get_x() + rect.get_width() / 2.0, height, f'{plot_bar_texts[i]}', ha='center', va='bottom')

        # Save the plot in the log directory
        plt.savefig(f'{root_dir}/{build}_{started_where}_percentages.png')

    def search_for_stationary_errors(self, error_count):
        """Traverses through the pose data to identify times in the cleaning 
        run where the robot remained stationary for a time longer than STATIONARY_TIME_THRESHOLD

        Parameters
        ----------
        error_count : dict(str: int)
            The errors found and the count of how many times they appeared in the log file

        Returns
        ----------
        stationary_errors : list(int)
            The list containing the length of times the robot was stationary longer than the threshold

        """

        # Keep track of the instances at which the robot was stationary
        stationary_errors = []

        # Initialize the current pose at the origin
        last_pose = np.array([0.0, 0.0])

        # Keep a count of the number of iterations the robot has been stationary
        stationary_count = 0

        # Set the error code for the stationary error
        error_code = 'ROBOT_STATIONARY_ERROR'

        # Iterate through the prox readings line by line to find the errors
        with open(self.POSE_DATA_FILE) as csvfile:
            reader = csv.DictReader(csvfile)
            for cnt, row in enumerate(reader):
                if (cnt % self.READING_SUBSAMPLE_ITERS == 0):
                    # Get the current position of the robot
                    cur_pose = np.array([float(row['smooth_x(mm)']), float(row['smooth_y(mm)'])])

                    # Calculate the distance traveled between the last two sampled points
                    distance_traveled = np.linalg.norm(cur_pose - last_pose)

                    # Increment the count every time the robot is stationary, otherwise reset the count
                    if (distance_traveled < self.STATIONARY_POSE_THRESHOLD):
                        stationary_count += 1
                    else:
                        # Before resetting the count, take note of all occurrences of the robot being stationary 
                        # for longer than the threshold
                        if (stationary_count > self.STATIONARY_TIME_ITERS_THRESHOLD):
                            # Convert the number of iters to time in ms before appending to the list
                            stationary_errors.append(stationary_count * self.READING_SUBSAMPLE_TIME)
                            error_count[error_code] += 1

                        # Reset the stationary count
                        stationary_count = 0

                        # Only change the last pose if the robot has moved far enough away from its last position
                        last_pose = cur_pose

        # Check if the robot was still stationary at the end of the run
        if (stationary_count > self.STATIONARY_TIME_ITERS_THRESHOLD):
            # Convert the number of iters to time in ms before appending to the list
            stationary_errors.append(stationary_count * self.READING_SUBSAMPLE_TIME)
            error_count[error_code] += 1

        return stationary_errors


    def display_stationary_errors(self, log_name, stationary_errors, summary_txt):
        """Display the lengths of times that the robot remained stationary 
        and generate a normalized histogram of those plots. Save the plots 
        in the Log

        Parameters
        ----------
        log_name : str
            The name of the log file we're analyzing

        stationary_errors : list(int)
            The lengths of time the robot remained stationary over the minimum threshold time

        summary_txt : file obj or None
            File object to write the output of the parser or None if the user prefers console only

        """
        

        # Create the list of bins for the histogram 
        bins_list = list(range(-self.STATIONARY_ERROR_BIN_SIZE, self.MAX_DISPLAY_STATIONARY_TIME + self.STATIONARY_ERROR_BIN_SIZE, self.STATIONARY_ERROR_BIN_SIZE))

        # Only display the log error messages if they exist
        if len(stationary_errors) > 0:
            # Convert the times from milliseconds to seconds
            stationary_errors = np.array(stationary_errors) / self.MS_TO_S

            # Print errors as list instead of numpy array for cleaner print statement
            stationary_statement = "\t\t These are the lengths of times the robot remained stationary during the run (in seconds): {}\n".format(list(stationary_errors))
            print(stationary_statement)
            self.write_to_file(summary_txt, stationary_statement)

            # Clip all times greater than MAX_DISPLAY_STATIONARY_TIME for histogram visualizations
            stationary_errors = np.clip(stationary_errors, 0, self.MAX_DISPLAY_STATIONARY_TIME)

            # Calculate weights to normalize the histogram data
            weights = np.ones_like(stationary_errors) / float(len(stationary_errors))

            # Generate the plot
            plt.hist(stationary_errors, bins_list, weights=weights)

            plt.title(log_name)
            plt.xlabel('Time Stationary in seconds')
            plt.ylabel('Normalized Frequency of Occurrence')

            # Save the plot in the log directory
            plt.savefig('{}/{}_stationary_times.png'.format(self.LOG_DATA_DIR, log_name))
            plt.close()


    def get_log_build_version(self):
        """Extracts the robot build version from the robot log to display 
        in the summary csv. The 'VersionData.txt' file generated from the 
        robot log takes the form as follows:

            1605298996400:  LDS Firmware Version : V4.0.0.20
                LDS Serial Number : GPM3242001-0001566
                Main Board Serial Number : 2026lb01105100594
                Robot SW Git Hash : 1b2108fdda0ae503eb0e82c7df17d3307834fb77
                Robot SW Toolchain : 2020-11-12_03:46
                Robot SW Build Version : Image version: 1.2.0-282_11111943_1b2108fd+DEBUG
                Robot Serial Number : 2036N301017100264

        So we look for the line that contains 'Robot SW Build Version' and extract everything after 
        the second ':' character. 

        Returns
        ----------
        build_version: str
            The robot build versione extracted from the robot log

        """
        

        # Surround this extraction in a try/catch block because old logs will not have the 
        # proper formatting for VersionData.txt
        try:
            with open(self.VERSION_DATA_FILE) as fp:
                for line in fp.readlines():
                    if self.ROBOT_BUILD_VERSION_STRING in line:
                        build_version = line.split(':')[-1].strip()
                        return build_version
        except Exception as e:
            print("The version could not be extracted: {}".format(e))


    def delete_uncompressed_bin_file(self, log):
        """Delete the uncompressed bin file generated by the simulator 
        when opening the log for analysis.

        Parameters
        ----------
        log : str
            String containing the file path of the log file

        """
        UNCOMPRESSED_FILE_EXTENSION = '.bin'

        # Remove the compressed bin file extension and append the uncompressed file extension
        uncompressed_log = log[:-self.MIN_LOG_FILE_LENGTH] + UNCOMPRESSED_FILE_EXTENSION

        # Remove the uncompressed bin file
        os.system("rm \"{}\"".format(uncompressed_log))


    def process_bb_files(self, bb_files, root_dir, save_summary, save_logs, save_maps, drop_thresh, no_left=False, no_right=False, no_prox_reading_errors=False, Log_message_errors = False, nav_falling= False, autonomy_summary=False, Choose_Error=False, prox_custom_data_only = False, TOF_Area_only = False, Data_CSV_Count_Error = False, Find_Event = False, change_State = False, Find_service = False, Data_CSV = False):
        """Process each black box file and look for the designated errors

        Parameters
        ----------
        bb_files : list(str)
            The list of valid black box file paths

        root_dir : str
            The root directory that contains all of the bb_files

        save_summary : boolean
            Toggle on whether to save the printout of the parser

        save_logs : boolean
            Toggle on whether to save the extracted *.txt and *.csv files for every log

        save_maps : boolean
            Toggle on whether to save global map files for every log

        nav_falling : boolean
            Toggle on whether to save plots of drop and wheel extended data around NAVIGATION_FALLING

        drop_thresh : int
            The threshold at which the drop values would trigger NAVIGATION_FALLING

        no_left : boolean
            Toggle on whether to plot left drop data and wheel extension

        no_right : boolean
            Toggle on whether to plot right drop data and wheel extension

        """

        # Keep track of stats accross multiple log files
        overall_error_count = defaultdict(int)

        #Keeping track of stats about autonomy
        autonomy_statistics = defaultdict(lambda: defaultdict(lambda: defaultdict(int)))

        # Open the summary txt and csv file for writing if the user chooses to save the results
        if save_summary:
            summary_txt = open(os.path.join(root_dir, self.SUMMARY_TXT), "w") if save_summary else None
            summary_file_csv = open(os.path.join(root_dir, self.SUMMARY_CSV), "w", newline='') if save_summary else None

            # Write the csv header to the file
            summary_csv = csv.writer(summary_file_csv)
            summary_csv.writerow(self.SUMMARY_CSV_HEADER)

        if autonomy_summary:
            autonomy_file_csv = open(os.path.join(root_dir, self.AUTONOMY_CSV), "w", newline='') 
            transitions_file_csv = open(os.path.join(root_dir, self.TRANSITIONS_CSV), "w", newline='') 

            # Write the csv header to the file
            autonomy_csv = csv.writer(autonomy_file_csv)
            autonomy_csv.writerow(self.AUTONOMY_CSV_HEADER)

            # Write the csv header to transitions file
            transitions_csv = csv.writer(transitions_file_csv)
            transitions_csv.writerow(self.TRANSITIONS_CSV_HEADER)

        for log in bb_files:
            print(f"Processing log file {log}")

            self.log = log

            # Initialize the row in the summary and autonomy csv for this file to all 0s 
            summary_row = [0] * len(self.SUMMARY_CSV_HEADER)
            autonomy_row = [0] * len(self.AUTONOMY_CSV_HEADER)

            # The first column of the summary and autonomy row is always the log file path
            summary_row[0] = log
            autonomy_row[0] = log
        
            # Grab the log name from the file
            log_name = os.path.basename(log)[:-self.MIN_LOG_FILE_LENGTH]

            # Keep track of the amount of times that each error appears and which, if any, critical errors appear
            error_count = defaultdict(int)

            # Generate the log txt files from the black box log using the simulator
            parse_log_result = os.system("{} -m \"{}\"".format(self.SIMULATOR_BINARY, log))

            # Ensure that the simulator exited without an error
            if (parse_log_result != 0):
                raise Exception("The simulator exited with error {}.".format(parse_log_result))

            # The second column of the summary and autonomy row is the build version of the robot log
            summary_row[1] = autonomy_row[1] = self.get_log_build_version()

            # Read through the generated LogMsgs.txt file to look for predesignated errors
            raised_errors, nav_falling_timestamps, base_reconnected_timestamp = self.search_for_log_msg_errors(error_count, log, summary_txt)

            # Read through the generated PoseData.csv to look for errors
            stationary_errors = self.search_for_stationary_errors(error_count)

            if TOF_Area_only:
                # grab tof map file from command line
                if len(sys.argv) < 2:
                    print("Missing tof map.")
                    exit
                else:
                    tof_map_file = sys.argv[1]

                avoided_cells = self.count_avoided_cells(tof_map_file)
                avoided_area = self.calc_area_from_cells(avoided_cells)

                # print("Avoided cell count:", avoided_cells)
                print(avoided_area)


            if prox_custom_data_only:
                rows, fieldnames = self.new_custom_prox()
                f = open('{}_custom.csv'.format(self.log), 'a+')
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
                f.close()

            if Data_CSV:
                rows= self.csv_spec_data()
                f = open('{}_specific.csv'.format(self.log), 'a+')
                writer = csv.writer(f)
                writer.writerows(rows)
                f.close()

            if Data_CSV_Count_Error:
                rows, fieldnames = self.csv_error_count()
                f = open('{}_count.csv'.format(self.log), 'a+')
                writer = csv.writer(f)
                writer.writerow(fieldnames)
                writer.writerows(rows)
                f.close()

            if change_State:
                self.Find_State()
            
            if Find_service:
                self.Service_state()

            if Choose_Error or Find_Event:
            
                if Choose_Error:
                    Error_name_alerts = self.search_for_any_errors_alerts()
                    summary_txt = open(os.path.join(root_dir, self.Any_Messages), "w")
                    self.display_for_any_errors_alerts(Error_name_alerts, summary_txt)

                if Find_Event:
                    Error_Event_alerts = self.event_search()
                    summary_txt = open(os.path.join(root_dir,  self.Event_txt), "w")
                    self.display_for_Event_alerts(Error_Event_alerts, summary_txt)  

            if no_prox_reading_errors:
                # Read through the generated ProxReadings.csv to look for errors
                logging_gap_errors, i2c_bus_errors, stale_tof_readings = self.search_for_prox_reading_errors(error_count, base_reconnected_timestamp)
                # Display logging gap errors
                self.display_logging_gap_errors(log_name, logging_gap_errors, summary_txt)

                # Display I2C bus errors
                self.display_i2c_bus_errors(i2c_bus_errors, summary_txt)

                # Display stale tof readings
                self.display_stale_tof_readings(stale_tof_readings, summary_txt)

            if Log_message_errors:

                # Display the relevant errors for the particular file
                self.display_all_errors_and_counts(log, error_count, summary_txt)

                # Display specific log errors
                self.display_log_msg_errors(raised_errors, summary_txt)

                # Display stationary errors
                self.display_stationary_errors(log_name, stationary_errors, summary_txt)
            
            # Display navigation falling drop values and wheel extension
            if nav_falling:
                self.display_navigation_falling(root_dir, log_name, nav_falling_timestamps, drop_thresh, no_left, no_right)

            # We're keeping different statistics for if the robot started on base or off base
            # Prioritize STARTED_ON_BASE and STARTED_OFF_BASE flags
            if error_count['STARTED_ON_BASE'] >= 1:
                started_where = "ON_BASE"
            elif error_count['STARTED_OFF_BASE'] >= 1:
                started_where = "OFF_BASE"
            # Use undocking indicators if flags not present in log messages
            elif error_count['STARTED_ON_BASE_UNDOCKING'] >= 1:
                started_where = "ON_BASE"
            else:
                started_where = "OFF_BASE"
        
            if len(error_count) == 0:
                # If there are no errors in the file, then we assume that it was not a valid cleaning log file
                overall_error_count['NO_CLEANING_RUN'] += 1
                # The second column in the summary row is always the 'RUN_STATUS'
                summary_row[2] = 'INVALID'
            else:
                # Add the types of errors present to the overall error count
                for error_code, count in error_count.items():
                    # Set the 'RUN_STATUS' column of the row
                    if error_code == 'PASS' or error_code == 'FAIL' or error_code == 'INVALID':
                        summary_row[2] = error_code
                    else:
                        if error_code in self.AUTONOMY_CSV_HEADER:
                            autonomy_statistics[started_where][summary_row[1]][error_code] += 1
                            autonomy_row[self.AUTONOMY_CSV_ERROR_TO_COL_IDX[error_code]] = count
                    
                     
                        summary_row[self.SUMMARY_CSV_ERROR_TO_COL_IDX[error_code]] = count
                    
                    # Count the total cleaning time for all the logs
                    if error_code == 'TOTAL_CLEANING_TIME':
                        overall_error_count[error_code] += count
                    else:
                        overall_error_count[error_code] += 1
            
                if 'CLEANING_START' in error_count or 'SUSPENDED_CHARGING_START' in error_count:
                    #Keeping track of log count per build to calculate autonomy percentages
                    autonomy_statistics[started_where][summary_row[1]]["count"] += 1

                for addition_col, cols_to_add in self.AUTONOMY_COLUMN_ADDITIONS.items():
                    addition = sum([error_count[col] for col in cols_to_add])
                    autonomy_row[self.AUTONOMY_CSV_ERROR_TO_COL_IDX[addition_col]] =  addition

                    #Keeping track of count of logs that triggered per build to calculate autonomy percentages
                    if addition >= 1:
                        autonomy_statistics[started_where][summary_row[1]][addition_col] += 1

            # If the user wants to save the log prescan, save the prescan folder in the same directory
            # as the original log destination before its overwritten
            if save_logs:
                new_log_data_name = "{}_{}".format(log_name, self.LOG_DATA_DIR)
                os.system("cp -rf {} \"{}\"".format(self.LOG_DATA_DIR, os.path.join(os.path.dirname(log), new_log_data_name)))

            if save_maps:
                dir_name = os.path.dirname(log)
                last_folder = dir_name[dir_name.rfind('/')+1:]
            
                # Create the folder if not exist to store all resulting map files
                # New folder will be one folder above the current log file.
                # This is mainly because this parser normally is ran at the SQA build folder where each log is stored in a seperate folder
                # run dir: /BuildXXX/
                # different types of map have it's only folder
                # map stored in: /BuildXXX/global_xxxx_maps/
                new_rank_map_folder_name = "{}/{}".format(dir_name[:dir_name.rfind('/')], "global_rank_maps")
                new_track_real_map_folder_name = "{}/{}".format(dir_name[:dir_name.rfind('/')], "global_track_real_maps") 
                new_track_virtual_map_folder_name = "{}/{}".format(dir_name[:dir_name.rfind('/')], "global_track_virtual_maps") 
                if (not os.path.isdir(new_rank_map_folder_name)):
                    os.mkdir(new_rank_map_folder_name)
                if (not os.path.isdir(new_track_real_map_folder_name)):
                    os.mkdir(new_track_real_map_folder_name)
                if (not os.path.isdir(new_track_virtual_map_folder_name)):
                    os.mkdir(new_track_virtual_map_folder_name)

                origin_rank_map = "{}/{}".format(self.LOG_DATA_DIR, self.GLOBAL_RANK_MAP_NAME)
                origin_track_real_map = "{}/{}".format(self.LOG_DATA_DIR, self.GLOBAL_TRACK_REAL_MAP_NAME)
                origin_track_virtual_map = "{}/{}".format(self.LOG_DATA_DIR, self.GLOBAL_TRACK_VIRTUAL_MAP_NAME)

                new_rank_map_file_name = "{}_{}_{}".format(last_folder, log_name, self.GLOBAL_RANK_MAP_NAME)
                new_track_real_map_file_name = "{}_{}_{}".format(last_folder, log_name, self.GLOBAL_TRACK_REAL_MAP_NAME)
                new_track_virtual_map_file_name = "{}_{}_{}".format(last_folder, log_name, self.GLOBAL_TRACK_VIRTUAL_MAP_NAME)

                os.system("cp {} \"{}\"".format(origin_rank_map, os.path.join(new_rank_map_folder_name, new_rank_map_file_name)))
                os.system("cp {} \"{}\"".format(origin_track_real_map, os.path.join(new_track_real_map_folder_name, new_track_real_map_file_name)))
                os.system("cp {} \"{}\"".format(origin_track_virtual_map, os.path.join(new_track_virtual_map_folder_name, new_track_virtual_map_file_name)))

                # read the map as an image using PIL
                im = Image.open(origin_rank_map)

                # crop the image to the area of interest
                im = self.crop_image_to_interest(im)

                # save cropped image(if valid) as a png file
                out_file = 'cropped_' + new_rank_map_file_name[:-4] + '.png'
                new_path_to_file = os.path.join(new_rank_map_folder_name, out_file)

                if im is not None:
                    print(new_path_to_file, ': saved')
                    im.save(os.path.join(new_rank_map_folder_name, out_file))
                else:
                    print(new_path_to_file, ': no crop-able region, skipping it')

            # Delete the uncompressed log file
            self.delete_uncompressed_bin_file(log)

            # Add whitespace for better readability
            print("\n\n")
            if save_summary:
                self.write_to_file(summary_txt, "\n\n")
                summary_csv.writerow(summary_row)
        
            if autonomy_summary:
                autonomy_csv.writerow(autonomy_row)

                # Checking what was the first transition failure in each log and storing in csv file
                for transition, (start_state, end_state) in self.AUTONOMY_TRANSITIONS.items():
                    if autonomy_row[self.AUTONOMY_CSV_ERROR_TO_COL_IDX[start_state]] >= 1 and autonomy_row[self.AUTONOMY_CSV_ERROR_TO_COL_IDX[end_state]] == 0:
                        transition_row = [transition, autonomy_row[0], autonomy_row[1], "", "", ""]
                        transitions_csv.writerow(transition_row)

        # save the count for different errors for every build
        if autonomy_summary:
            print(autonomy_statistics)
            log_count = len(bb_files)
            plot_events = ['CLEANING_START (sum)', 'UNDOCKING (sum)', 'ACTIVE_CLEANING_SESSION (sum)', 'DOCKING (sum)', 'DOCKING_SUCCESSFUL (sum)']

            for started_where in autonomy_statistics:
                for build in autonomy_statistics[started_where]:
                    autonomy_row = [0] * len(self.AUTONOMY_CSV_HEADER)  
                    percentage_row = [0] * len(self.AUTONOMY_CSV_HEADER) 
                    autonomy_row[0] = f'All logs ({started_where})'
                    percentage_row[0] = f'All logs % ({started_where})'
                    autonomy_row[1] = build
                    percentage_row[1] = build
                    for error_code in autonomy_statistics[started_where][build]:
                        if error_code != "count":
                            build_error_count = autonomy_statistics[started_where][build][error_code]
                            percentage = (build_error_count/autonomy_statistics[started_where][build]["count"])*100 
                            autonomy_row[self.AUTONOMY_CSV_ERROR_TO_COL_IDX[error_code]] = build_error_count
                            percentage_row[self.AUTONOMY_CSV_ERROR_TO_COL_IDX[error_code]] = percentage
                    autonomy_csv.writerow(autonomy_row)
                    autonomy_csv.writerow(percentage_row) 

                    #Plotting the percentage of logs that triggered the autonomy events
                    plot_percentages = [percentage_row[self.AUTONOMY_CSV_ERROR_TO_COL_IDX[event]] for event in plot_events]
                    plot_bar_texts = [autonomy_statistics[started_where][build][event] for event in plot_events]
                    total_count = autonomy_statistics[started_where][build]["count"]
                    self.display_autonomy_percentages(plot_events, plot_percentages, plot_bar_texts, total_count, started_where, build) 

        # Use to display the counts uniformly
        count_format = "{{:{}d}}".format(1 + int(np.log10(len(bb_files))))

        # Print out the totals over all the files
        total_cleaning_time = 0 if 'TOTAL_CLEANING_TIME' not in overall_error_count else overall_error_count.pop("TOTAL_CLEANING_TIME")
        total_files_statement = "FINAL RESULTS: \nThis script analyzed a total of {} files.\nThe total cleaning time across all valid cleaning files was {} seconds.\nThe breakdown is as follows: \n".format(len(bb_files), total_cleaning_time)
        print(total_files_statement)
        self.write_to_file(summary_txt, total_files_statement)
        for error_code, file_count in sorted(overall_error_count.items(), key=lambda x: x[0]):
            error_statement = "\t {} file{} contained {}. \n".format(count_format.format(file_count), " " if file_count == 1 else "s", error_code)
            print(error_statement)
            self.write_to_file(summary_txt, error_statement)

        # Close the summarry file if it exists
        if save_summary:
            summary_txt.close()
            summary_file_csv.close()

    def crop_image_to_interest(self, im):
        """Iterate through & crop the pgm map finally saving it as a png

        Parameters
        ----------
        im : image
            Map file stored as a PIL image object

        Returns
        ----------
        im : image
            Cropped image of the ROI if it exists, None otherwise
        """

        bbox_x_min = im.width
        bbox_x_max = 0
        bbox_y_min = im.height
        bbox_y_max = 0

        # iterate through the image to set the crop bounds
        for x in range(im.width):
            for y in range(im.height):
                cur_pixel = im.getpixel((x, y))
                if (cur_pixel == self.RV_UNKNOWN):
                    continue
                bbox_x_min = min(bbox_x_min, x)
                bbox_x_max = max(bbox_x_max, x)
                bbox_y_min = min(bbox_y_min, y)
                bbox_y_max = max(bbox_y_max, y)

        # safety guard to ensure we don't try to crop blank maps
        if bbox_x_max <= bbox_x_min or bbox_y_max <= bbox_y_min :
            return None 

        # adding buffer to min & max boundary box bounds
        bbox_x_min = max(0, bbox_x_min - self.IMAGE_BUFFER)
        bbox_x_max = min(im.width, bbox_x_max + self.IMAGE_BUFFER)
        bbox_y_min = max(0, bbox_y_min - self.IMAGE_BUFFER)
        bbox_y_max = min(im.height, bbox_y_max + self.IMAGE_BUFFER)
    
        # crop the image with valid cropping bounds and return the updated one
        im = im.crop((bbox_x_min, bbox_y_min, bbox_x_max, bbox_y_max))
        return im
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', help='file to parse', type=str)
    parser.add_argument('-d', '--dir', help='directory containing black box files to parse (this script will recursively look through the passed in directory to find all log files', type=str)
    parser.add_argument('-i', '--robotparse', help="Only show json file worked")
    args = parser.parse_args()
    json_file_path = args.robotparse
    if json_file_path is None:
        json_file_path = "robotparse.json"
    sitter = BB_Parser(json_file_path)
    error_code_format = "{{:{}s}}".format(max([len(error_code) for error_code in sitter.LOG_MSGS_ERRORS.keys()]))
    tof_error_code_format = "{{:{}s}}".format(max([len(tof_error_code) for tof_error_code, _ in sitter.TOF_ERROR_CODES.values()]))
    help_msg = "General errors analyzed\n{}\n* {}\n\nTOF errors analyzed\n{}\n* {}".format("-" * len("General errors analyzed"), "\n* ".join(["{} = {}".format(error_code_format.format(error_code), error_description) for error_code, (_, error_description) in sorted(sitter.LOG_MSGS_ERRORS.items(), key=lambda x: x[0])]), "-" * len("TOF errors analyzed"), "\n* ".join(["{} = {}".format(tof_error_code_format.format(error_code), error_description) for error_code, error_description in sorted(sitter.TOF_ERROR_CODES.values(), key=lambda x: x[0])]))

    parser = argparse.ArgumentParser(description="Parse black box logs for anomalies.", formatter_class=argparse.RawDescriptionHelpFormatter, epilog=help_msg)
    parser.add_argument('-f', '--file', help='file to parse', type=str)
    parser.add_argument('-d', '--dir', help='directory containing black box files to parse (this script will recursively look through the passed in directory to find all log files', type=str)
    parser.add_argument('--no-summary', action='store_false', help="don't generate a summary.txt with the results of the parser")
    parser.add_argument('--save-logs', action='store_true', help='save the log files for all black box logs passed into the script')
    parser.add_argument('--save-maps', action='store_true', help='save all the global maps for each log file passed into the script')
    parser.add_argument('--drop-thresh', type=int, help='threshold at which NAVIGATION_FALLING would be triggered. if not provided, value will be read in from source code')
    parser.add_argument('-i', '--robotparse', help="Only show json file worked")
    args = parser.parse_args()

    # Obtain the valid files from the user 
    bb_files, root_dir = sitter.get_files_from_user(args)

    # Check if the simulator has been made
    if (not path.exists(sitter.SIMULATOR_BINARY)):
        raise Exception("Simulator binary has not been made. Please run 'make tools' from the root robot directory.")
        
    # Process the black box files
    sitter.process_bb_files(bb_files, root_dir, args.no_summary, args.save_logs, args.save_maps, args.drop_thresh, sitter.no_left, sitter.no_right, 
                        sitter.no_prox_reading_errors, sitter.Log_message_errors, sitter.nav_falling, sitter.autonomy_summary, sitter.specific_message, sitter.Want_to_graph_CSV_data, sitter.TOF_area, sitter.Data_CSV_Count_Error, sitter.Find_Event, sitter.change_State, False, True)

