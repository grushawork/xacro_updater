import json
import os
import sys
from lxml import etree

import rclpy
import rclpy.logging
from rclpy.node import Node


def parse_xacro_properties(xml_file):
    tree = etree.parse(xml_file)
    root = tree.getroot()
    properties = []

    for property in root.findall('.//{http://www.ros.org/wiki/xacro}property'):
        name = property.get('name')
        value = property.get('value')
        properties.append({'name': name, 'value': value})

    return properties


def update_property_value(xml_file, property_name, new_value):
    tree = etree.parse(xml_file)
    root = tree.getroot()

    property = root.find(f".//{{http://www.ros.org/wiki/xacro}}property[@name='{property_name}']")
    
    if property is not None:
        property.set('value', new_value)
        tree.write(xml_file, pretty_print=True, xml_declaration=True, encoding='UTF-8')
    else:
        print(f'There\'s no such property: {property_name}')


class XacroUpdaterJSONNode(Node):
    def __init__(self):
        super().__init__('xacro_updater_json')

        filename = ''

        try:
            filename = sys.argv[1]
        except:
            rclpy.logging.get_logger('xacro_updater').error('No argument passed. Path to xacro-file expected.')
            rclpy.shutdown()
            return

        while True:
            print('To update a property value, press Enter')
            input()
            os.system('clear')

            xacro_properties = parse_xacro_properties(filename)

            print('Enter JSON-file name to update xacro-file properties: ', end='')
            json_update_filename = input()

            if not json_update_filename:
                print('File name cannot be empty, try again')
                continue

            try:
                with open(json_update_filename, 'r') as file:
                    update_data = json.load(file)
            except FileNotFoundError:
                print(f'Error: The file "{json_update_filename}" was not found.')
                continue
            except json.JSONDecodeError:
                print('Error: Could not decode JSON from the file.')
                continue

            for obj, attrs in update_data.items():
                for attr_name, attr_value in attrs.items():
                    update_property_value(filename, f'{obj}_{attr_name}', attr_value)
            
            print('\nUpdated successfully\n')


def main(args=None):
    try:
        rclpy.init(args=args)
        node = XacroUpdaterJSONNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        rclpy.logging.get_logger('xacro_updater').info('Keyboard Interrupt')


if __name__ == '__main__':
    main()
