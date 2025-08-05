import sys
from lxml import etree

import rclpy
import rclpy.logging
from rclpy.node import Node


def parse_xacro_args(xml_file):
    tree = etree.parse(xml_file)
    root = tree.getroot()
    args = []

    for arg in root.findall('.//{http://www.ros.org/wiki/xacro}arg'):
        name = arg.get('name')
        default = arg.get('default')
        args.append({'name': name, 'default': default})

    return args

def update_default_value(xml_file, arg_name, new_default):
    tree = etree.parse(xml_file)
    root = tree.getroot()

    # Ищем тег <xacro:arg> с заданным именем
    arg = root.find(f".//{{http://www.ros.org/wiki/xacro}}arg[@name='{arg_name}']")
    
    if arg is not None:
        arg.set('default', new_default)  # Обновляем значение атрибута default
        # Сохраняем изменения в файл, сохраняя комментарии и форматирование
        tree.write(xml_file, pretty_print=True, xml_declaration=True, encoding='UTF-8')
    else:
        print(f"Аргумент с именем '{arg_name}' не найден.")


class ArgParserNode(Node):
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
            print('To update an xacro:arg value, press Enter')
            input()

            xacro_args = parse_xacro_args(filename)

            for i, arg in enumerate(xacro_args):
                print(f'{i}: Name: {arg["name"]}, Default: {arg["default"]}')
            
            print('\nEnter a number of xacro:arg to update its value: ', end='')
            arg_num = int(input())
            print(f'Enter a new value for "{xacro_args[arg_num]["name"]}" argument: ', end='')
            new_value = input()
            
            update_default_value(filename, xacro_args[arg_num]['name'], new_value)
            print('Updated successfully\n')


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ArgParserNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        rclpy.logging.get_logger('xacro_updater').info('Keyboard Interrupt')


if __name__ == '__main__':
    main()
