def rad_to_deg(rad):
    return rad * 180.0 / 3.14
    
def deg_to_rad(deg):
    return deg * 3.14 / 180.0

def port_connection_error_msg():
    error_message = "OSERROR: [Errno 19] ENODEV: \n\n"
    error_message += "A sensor or motor is not connected to the specified port:\n"
    error_message += "--> Check the cables to each motor and sensor.\n"
    error_message += "--> Check the port settings in your script.\n"
    error_message += f"--> Check the line in your script that matches\n    the line number given in the 'Traceback' above."
    
    return error_message