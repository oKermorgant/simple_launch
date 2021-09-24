#!/usr/bin/env python
import sys
import xml.etree.ElementTree as ET

def update(element, links, prefix):
    for child in element:
        if 'frame' in child.tag or 'Frame' in child.tag:
            if child.text in links:
                child.text = prefix + child.text
        else:
            update(child, links, prefix)    

def main():
    if len(sys.argv) == 1:
        print('[simple_launch] pass a robot description as a raw string')
        sys.exit(0)
    if len(sys.argv) == 2:
        print('[simple_launch] pass a frame prefix as a raw string')
        sys.exit(0)
    description, prefix = sys.argv[1:3]        
    
    # lets us assume Gazebo-written link names are <gazebo> <*[fF]rame*> link </*[fF]rame*> </gazebo>
    xml = ET.fromstring(description)
    
    # ensure any renamed link actually belongs to this robot, or is the odom link
    links = [link.attrib['name'] for link in xml.iter('link')] + ['odom']
    
    for gz in xml.iter('gazebo'):
        update(gz, links, prefix)
    
    # print to stdout and let SimpleLauncher pass it to robot_state_publisher
    ET.dump(xml)      
            
if __name__ == "__main__":
    main()
 
