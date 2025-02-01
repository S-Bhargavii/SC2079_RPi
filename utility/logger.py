import logging

# export PYTHONPATH=/home/pi/Desktop/mdp/code:$PYTHONPATH

formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

def configure_logger(logger:logging.Logger):
    """
        Takes the logger and attaches file handler and console handler, 
        and does some configuration
    """
    file_name = f"/home/pi/Desktop/mdp/code/logs/{logger.name}.log"
    logger.setLevel(logging.DEBUG)

    # Add a file handler
    file_handler = logging.FileHandler(file_name, encoding='utf-8')
    file_handler.setLevel(logging.DEBUG)  
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    # Add console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
