import os

LOGGING_CONFIG = {
    'version': 1,
    'formatters': {
        'simpleFormatter': {
            'format': '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            'datefmt': '%Y-%m-%d %H:%M:%S'
        }
    },
    'handlers': {
        'consoleHandler': {
            'class': 'logging.StreamHandler',
            'level': 'INFO',
            'formatter': 'simpleFormatter',
            'stream': 'ext://sys.stdout'
        }
    },
    'loggers': {
        'simulation_service': {
            'level': 'INFO',
            'handlers': [],  # No direct handlers - uses root's console via propagation
            'propagate': True
        },
        'calibration_service': {
            'level': 'INFO',
            'handlers': [],
            'propagate': True
        },
        'db_recorder_service': {
            'level': 'INFO',
            'handlers': [],
            'propagate': True
        },
        'mockup_state_publisher': {
            'level': 'INFO',
            'handlers': [],
            'propagate': True
        },
        'ur3e_mockup': {
            'level': 'INFO',
            'handlers': [],
            'propagate': True
        },
        'monitoring_service': {
            'level': 'DEBUG',
            'handlers': [],
            'propagate': True
        },
        'alarm_manager_service': {
            'level': 'DEBUG',
            'handlers': [],
            'propagate': True
        }
    },
    'root': {
        'level': 'INFO',
        'handlers': ['consoleHandler']
    }
}

LOG_DIR_PATH = os.path.join(os.path.dirname(__file__), "../logs/")