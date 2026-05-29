from services.command_sender import run_command_sender_ui

"""
This module starts the command sender service foud in ../services/command_sender.py in a new process.
"""

def start_command_sender(ok_queue=None):

    if ok_queue is not None:
        ok_queue.put("OK")

    run_command_sender_ui()
    