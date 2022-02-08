import main_interface
import configparser

if __name__ == "__main__":
    # Process config file here
    config = configparser.ConfigParser()
    config.read("./config.ini")

    # Allow pre-load (i.e. scheduler 0600 0; cam.connect 0; cam.live; robot.calibrate;)


    # Start application
    interface = main_interface.main_interface("./", config)
    interface.run()
