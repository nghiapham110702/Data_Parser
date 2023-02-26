import json

class JSON_INPUT():

    def __init__(self, json_file_path):
        input_json = self.read_json_file(json_file_path)
        self.SUMMARY_CSV_HEADER = ['LOG_NAME', 'BUILD_VERSION', 'RUN_STATUS']
        self.parse_JSON(input_json)

    def read_json_file(self, json_file_path):
        try:
            file = open(json_file_path, mode="r")
            input_json = json.loads(file.read())
            file.close()
            return input_json
        except UnicodeDecodeError as e:
                print("Unable to open file: {}".format(e))

    def parse_JSON(self,input_json):
        #choose which caterory of error to parseLOG_MSGS_ERRORS

        # The keys here are the error type and the values are the substring that appears
        # in the LogMsgs.txt that signifify this error has occurred. (Move inside json file)
        if "LOG_MSGS_ERRORS" in input_json:
            self.LOG_MSGS_ERRORS = input_json['LOG_MSGS_ERRORS']
            self.SUMMARY_CSV_HEADER.extend(self.LOG_MSGS_ERRORS.keys())
            
        # Create a list containing the column headers of the autonomy summary based on the errors parsed by this script
        if "AUTONOMY_CSV_HEADER" in input_json:
            self.AUTONOMY_CSV_HEADER = input_json['AUTONOMY_CSV_HEADER']
            self.AUTONOMY_CSV_ERROR_TO_COL_IDX = {header: idx for idx, header in enumerate(self.AUTONOMY_CSV_HEADER)}

        if "AUTONOMY_COLUMN_ADDITIONS" in input_json:
            self.AUTONOMY_COLUMN_ADDITIONS  = input_json['AUTONOMY_COLUMN_ADDITIONS']

        if "AUTONOMY_SUSPENDED_STATES" in input_json:
            self.AUTONOMY_SUSPENDED_STATES = input_json['AUTONOMY_SUSPENDED_STATES']

        if "TRANSITIONS_CSV_HEADER" in input_json:
            self.TRANSITIONS_CSV_HEADER = input_json["TRANSITIONS_CSV_HEADER"]

        if "AUTONOMY_TRANSITIONS" in input_json:
            self.AUTONOMY_TRANSITIONS = input_json['AUTONOMY_TRANSITIONS']

        # tof reading and the values are the particular error that value corresponds to
        if "TOF_ERROR_CODES" in input_json:
            self.TOF_ERROR_CODES = input_json['TOF_ERROR_CODES']

        # Define the errors to look out for in the prox readings csv
        if "PROX_ERROR_FIELDS_OF_INTEREST" in input_json:
            self.PROX_ERROR_FIELDS_OF_INTEREST = input_json['PROX_ERROR_FIELDS_OF_INTEREST']
            self.SUMMARY_CSV_HEADER.extend(self.PROX_ERROR_FIELDS_OF_INTEREST.keys())
            self.SUMMARY_CSV_ERROR_TO_COL_IDX = {header: idx for idx, header in enumerate(self.SUMMARY_CSV_HEADER)}

        # Define the errors to look out for in the pose data csv
        if "POSE_ERROR_FIELDS_OF_INTEREST" in input_json:
            self.POSE_ERROR_FIELDS_OF_INTEREST = input_json['POSE_ERROR_FIELDS_OF_INTEREST']
            self.SUMMARY_CSV_HEADER.extend(self.POSE_ERROR_FIELDS_OF_INTEREST.keys())
            self.SUMMARY_CSV_ERROR_TO_COL_IDX = {header: idx for idx, header in enumerate(self.SUMMARY_CSV_HEADER)}
        
        if "Custom_Error" in input_json:
            self.Custom_Error = input_json['Custom_Error']

        if "CSV_specfic_data" in input_json:
            self.CSV_specfic_data = input_json['CSV_specfic_data']

        if "Input_values" in input_json:
            self.Input_values = input_json['Input_values']


        if "CUSTOM_PROX_DATA" in input_json:
            self.CUSTOM_PROX_DATA = input_json['CUSTOM_PROX_DATA']

        if "CSV_count" in input_json:
            self.CSV_count = input_json['CSV_count']

        if "values" in input_json:
            self.values = input_json['values']

        if "startTime" in input_json:
            self.startTime = input_json['startTime']

        if "endTime" in input_json:
            self.endTime = input_json['endTime']
        
        if "start_string" in input_json:
            self.start_string = input_json['start_string']
        
        if "endstring" in input_json:
            self.endstring = input_json['endstring']

        if "LOG_DATA_DIR" in input_json:
            self.LOG_DATA_DIR = input_json['LOG_DATA_DIR']

        if "CSV_READINGS_FILE" in input_json:
            self.CSV_READINGS_FILE = input_json['CSV_READINGS_FILE']
            self.CSV_File = f'{self.LOG_DATA_DIR}/{self.CSV_READINGS_FILE}'
            #self.CSV_File = '{}/self.CSV_READINGS_FILE'.format(self.LOG_DATA_DIR)

        if "LOG_MSGS_FILE" in input_json:
            self.LOG_MSGS_FILE = input_json['LOG_MSGS_FILE']
            self.Log_TXT = f'{self.LOG_DATA_DIR}/{self.LOG_MSGS_FILE}'
            
        if "specific_message" in input_json:
            self.specific_message = bool(input_json["specific_message"])

        if "no_prox_reading_errors" in input_json:
            self.no_prox_reading_errors = bool(input_json["no_prox_reading_errors"])

        if "Log_message_errors" in input_json:
            self.Log_message_errors = bool(input_json["Log_message_errors"])

        if "nav_falling" in input_json:
            self.nav_falling = bool(input_json["nav_falling"])

        if "autonomy_summary" in input_json:
            self.autonomy_summary = bool(input_json["autonomy_summary"])

        if "no_left" in input_json:
            self.no_left = bool(input_json["no_left"])

        if "no_right" in input_json:
            self.no_right = bool(input_json["no_right"])
        
        if "Want_to_graph_CSV_data" in input_json:
            self.Want_to_graph_CSV_data = bool(input_json["Want_to_graph_CSV_data"])

        if "TOF_area" in input_json:
            self.TOF_area = bool(input_json["TOF_area"])

        if "Data_CSV_Count_Error" in input_json:
            self.Data_CSV_Count_Error = bool(input_json["Data_CSV_Count_Error"])

        if "Find_Event" in input_json:
            self.Find_Event = bool(input_json["Find_Event"])

        if "change_State" in input_json:
            self.change_State = bool(input_json["change_State"])

