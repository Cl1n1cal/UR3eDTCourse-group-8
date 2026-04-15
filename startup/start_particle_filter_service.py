from services.particle_filter_service import ParticleFilterService
from utils.configuration import load_config

"""
This module starts the simulation service foud in ../services/simulation_service.py in a new process.
"""

def start_calibration_service(ok_queue=None):
    particle_filter_service = ParticleFilterService()
    config = load_config("startup/startup.conf")
    particle_filter_service.setup(particle_filter_config=config["particle_filter_service"])
  
    if ok_queue is not None:
        ok_queue.put("OK")

    calibration_service.start_serving()
    
