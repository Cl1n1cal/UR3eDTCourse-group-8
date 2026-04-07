import logging
import logging.config
import os
from ..logging_dict import LOGGING_CONFIG, LOG_DIR_PATH

logging.config.dictConfig(LOGGING_CONFIG)

#deprecated
def config_logging(filename=None, level=logging.WARN):
    if filename is not None:
        # noinspection PyArgumentList
        logging.basicConfig(level=level,
                            handlers=[
                                logging.FileHandler(filename),
                                logging.StreamHandler()
                            ],
                            format='%(asctime)s.%(msecs)03d %(levelname)s %(name)s : %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S'
                            )
    else:
        # noinspection PyArgumentList
        logging.basicConfig(level=level,
                            format='%(asctime)s.%(msecs)03d %(levelname)s %(name)s : %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S'
                            )

def setup_root_logging(name = "global"):
    root_logfile = LOG_DIR_PATH + name + ".log"
    root_logger = logging.getLogger()   # root logger
    fmt = root_logger.handlers[0].formatter

    if not any(isinstance(h, logging.FileHandler) and h.baseFilename == str(root_logfile)
               for h in root_logger.handlers):
        fh = logging.FileHandler(root_logfile, encoding="utf-8", mode='w')
        fh.setFormatter(fmt)
        root_logger.addHandler(fh)

    if not any(isinstance(h, logging.StreamHandler) for h in root_logger.handlers):
        ch = logging.StreamHandler()
        ch.setFormatter(fmt)
        root_logger.addHandler(ch)

def create_service_logger(service_name):
    service_logfile = LOG_DIR_PATH + service_name + ".log"
    logger = logging.getLogger(service_name)   # child logger
    # avoid duplicate handler on repeated init
    fmt = logging.getLogger().handlers[0].formatter
    if not any(isinstance(h, logging.FileHandler) and h.baseFilename == str(service_logfile)
               for h in logger.handlers):
        h = logging.FileHandler(service_logfile, encoding="utf-8", mode='w')
        h.setFormatter(fmt)
        logger.addHandler(h)

    logger.propagate = True   # default, ensure global root also receives
    return logger