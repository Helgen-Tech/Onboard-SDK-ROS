from fastkml import kml
import re
import yaml
import argparse
import os


def argparser():
    parser = argparse.ArgumentParser(description="Transform Google Earth KML to YAML")
    parser.add_argument("--input", default="waypoints.kml",
                    help="input kml file")
    parser.add_argument("--output", default="waypoints.yaml",
                    help="output yaml file")
    parser.add_argument("--idle_vel", default=2,
                    help="idle velocity of all points")
    parser.add_argument("--damping", default=0,
                    help="damping of all points")

    return parser.parse_args()

def prepend_multiple_lines(file_name, list_of_lines):

    dummy_file = file_name + '.bak'
    with open(file_name, 'r') as read_obj, open(dummy_file, 'w') as write_obj:
        for line in list_of_lines:
            write_obj.write(line + '\n')
        for line in read_obj:
            write_obj.write(line)
    os.remove(file_name)
    os.rename(dummy_file, file_name)


if __name__ == '__main__':

    args = argparser()

    with open(args.input) as kml_file:
        
        doc = kml_file.read().encode('utf-8')

    str_doc = str(doc)
    k = kml.KML()
    k.from_string(doc)

    dict = {"wp_list": {}}
    number_of_waypoints = 0

    with open(args.output, 'w') as file:
        for feature0 in k.features():
            for feature1 in feature0.features():
                number_of_waypoints += 1
                pos = str_doc.find("<name>" + feature1.name + "</name>")
                aux_doc = str_doc[pos:]
                pos_altitude = aux_doc.find("<altitude>")
                pos_altitude_end = aux_doc.find("</altitude>")

                altitude = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", aux_doc[pos_altitude:pos_altitude_end])[0]
                
                dict["wp_list"]["wp" + feature1.name] = {"longitude": feature1.geometry.x, "latitude": feature1.geometry.y, "altitude": float(altitude), "damping": float(args.damping)}
                
        
        dict["wp_number"] = number_of_waypoints
        dict["idle_velocity"] = float(args.idle_vel)

        yaml.dump(dict, file, sort_keys=False)

    prepend_multiple_lines(args.output, ["%YAML:1.0", ""])

