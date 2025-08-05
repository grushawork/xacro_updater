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


class XacroUpdaterNode(Node):
    def __init__(self):
        super().__init__('xacro_updater')

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

            for i, property in enumerate(xacro_properties):
                print(f'{i}: Name: {property["name"]}, Value: {property["value"]}')
            
            print('\nEnter index of property to update its value: ', end='')

            try:
                property_idx = int(input())
            except ValueError:
                print('It\'s not a number, try again')
                continue
            
            try:
                print(f'Enter a new value for "{xacro_properties[property_idx]["name"]}" argument: ', end='')
            except IndexError:
                print('There\'s no such property, try again\n')
                continue
            
            new_value = input()
            
            update_property_value(filename, xacro_properties[property_idx]['name'], new_value)
            print('\nUpdated successfully\n')


def main(args=None):
    try:
        rclpy.init(args=args)
        node = XacroUpdaterNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        rclpy.logging.get_logger('xacro_updater').info('Keyboard Interrupt')


if __name__ == '__main__':
    main()
