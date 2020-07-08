import inspect
import os.path

def get_caller_filepath():
    # get the caller's stack frame and extract its file path
    frame_info = inspect.stack()[1]
    filepath = frame_info[1]  # in python 3.5+, you can use frame_info.filename
    del frame_info  # drop the reference to the stack frame to avoid reference cycles

    # make the path absolute (optional)
    filepath = os.path.abspath(filepath)
    return filepath


# def get_caller_filepath():
#     previous_frame = inspect.currentframe().f_back
#     (filename, line_number, 
#      function_name, lines, index) = inspect.getframeinfo(previous_frame)
#     return (filename, line_number, function_name, lines, index)