from services.dashboard import run_dashboard_ui

def start_dashboard(ok_queue=None):
    if ok_queue is not None:
        ok_queue.put("OK")
    run_dashboard_ui()
