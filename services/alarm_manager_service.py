import time

class AlarmSeverity:
    INFO = 'info'
    WARNING = 'warning'
    ERROR = 'error'

class AlarmType:
    STUCK_JOINT = 'stuck_joint'
    WEAR_JOINT = 'wear_joint'
    TCP_MISMATCH = 'tcp_mismatch'

class Alarm:
    def __init__(self, alarm_type : AlarmType, severity : AlarmSeverity, message, timestamp, args):
        self.alarm_type = alarm_type
        self.severity = severity
        self.message = message
        self.timestamp = timestamp
    
class AlarmManagerService:
    def __init__(self):
        self.alarms = []

    def add_alarm(self, alarm_time, message):
        alarm = {'time': alarm_time, 'message': message}
        self.alarms.append(alarm)
        print(f"Alarm set for {alarm_time} with message: '{message}'")

    def remove_alarm(self, alarm_time):
        self.alarms = [alarm for alarm in self.alarms if alarm['time'] != alarm_time]
        print(f"Alarm for {alarm_time} removed.")

    def get_alarms(self):
        return self.alarms