#!/usr/bin/env python
import sys
import xml.etree.ElementTree as ET
import argparse

def update(element, links, prefix):
    for child in element:
        if 'frame' in child.tag or 'Frame' in child.tag:
            if child.text in links:
                child.text = prefix + child.text
        else:
            update(child, links, prefix)

def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.description = 'On-the-fly update of Gazebo plugins specs in a robot description'
    parser.add_argument('-d','--description', type=str, help='Raw robot description')
    parser.add_argument('--frame_prefix',type=str, help='Frame prefix', default='')

    args = parser.parse_args()

    if args.frame_prefix == '':
        print(args.description)

    xml = ET.fromstring(args.description)

    # ensure any renamed link actually belongs to this robot, or is the odom link
    links = [link.attrib['name'] for link in xml.iter('link')] + ['odom']

    # we only care about renaming inside Gazebo tags
    for gz in xml.iter('gazebo'):
        update(gz, links, args.frame_prefix)

    # print to stdout and let SimpleLauncher pass it to robot_state_publisher
    ET.dump(xml)

if __name__ == "__main__":
    main()

