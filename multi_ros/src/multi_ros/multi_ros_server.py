import argparse
import json
from multi_ros.multi_ros_parent import MultiRosParent


class MultiRosServer(object):
    def __init__(self):
        """
        Uses argparse to get a list of config files
        loops through them and creates a MultiRosParent for each file
        starts a thread for each one
        """
        self._argparser = argparse.ArgumentParser()
        self._argparser.add_argument('config_files', nargs='*', default=[], help='List of configuration files to load.')
        args = self._argparser.parse_args()

        self._parents = []
        self._threads = []
        for file_name in args.config_files:
            try:
                self._parents.append(MultiRosParent(self.parse_config_dict(file_name)))
            except StandardError as e:
                print e
                continue

    def parse_config_dict(self, config_file):
        config_dict = json.loads(open(config_file).read())
        if 'uri' not in config_dict:
            raise StandardError('Uri missing from config file: %s' % config_file)
        elif 'prefix' not in config_dict:
            raise StandardError('Prefix missing from config file: %s' % config_file)
        elif 'topics' not in config_dict:
            raise StandardError('Topics missing from config file: %s' % config_file)
        return config_dict
