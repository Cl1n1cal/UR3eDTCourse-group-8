"""
This module starts the simulation service foud in ../services/simulation_service.py in a new process.
"""
from utils.configuration import load_config; config = load_config('startup/startup.conf')
from services.simulation_service import SimulationService

def start_sim_service(ok_queue=None):
    sim_service = SimulationService(config['digital_twin']['robot_model']['time_step'], config['digital_twin']['robot_model']['publish_period'])
  
    sim_service.setup(config['digital_twin']['robot_model']['initial_q'],
                      config['digital_twin']['robot_model']['max_velocity'], 
                      config['digital_twin']['robot_model']['acceleration'])
    
    if ok_queue is not None:
        ok_queue.put("OK")

    sim_service.start_serving()
    
