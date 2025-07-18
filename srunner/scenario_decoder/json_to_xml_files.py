"""Takes json input and outputs xml files for routes and scenarios"""

import json
import xml.etree.ElementTree as ET
import xml.dom.minidom as xmlminidom
from enum import Enum
from typing import Optional


class Definitions(Enum):
    """JSON enumerations for readability of the sections in the JSON file"""

    ROUTES = "routes"
    SCENARIOS = "scenarios"


class XMLToFiles:
    """Class to create appropriate scenario runner compatible xml files from a json file"""

    def __init__(self, json_path: str) -> None:
        with open(json_path, "r", encoding="UTF-8") as raw_json:
            self.parsed_json = json.loads(raw_json.read())

        self.route_node = self.create_xml(
            self.parsed_json[Definitions.ROUTES.value], None, Definitions.ROUTES.value
        )
        self.route_xml = self.node_to_xml_string(self.route_node)

        self.scenario_node = self.create_xml(
            self.parsed_json[Definitions.SCENARIOS.value],
            None,
            Definitions.SCENARIOS.value,
        )
        self.scenario_xml = self.node_to_xml_string(self.scenario_node)

    def create_files(self, output_dir: str = ".") -> None:
        """create xml output files for the route and scenario from json

        Args:
            output_dir (str, optional): output directory for xml files. Defaults to ".".
        """
        with open(f"{output_dir}/route.xml", "w", encoding="UTF-8") as route:
            route.write(self.route_xml)

        with open(f"{output_dir}/scenario.xml", "w", encoding="UTF-8") as scenario:
            scenario.write(self.scenario_xml)

    def node_to_xml_string(self, xml_tree_node: ET.Element) -> str:
        """converts a node for an xml tree into the string representation

        Args:
            xml_tree_node (ET.Element): root node of tree to convert to string

        Returns:
            str: string representation of xml tree
        """
        xml_string = ET.tostring(xml_tree_node, encoding="utf8").decode("utf8")
        xml_pretty = xmlminidom.parseString(xml_string).toprettyxml()
        return xml_pretty

    def create_xml(
        self,
        data: dict,
        xml_tree_node: Optional[ET.Element] = None,
        json_section: Optional[Definitions] = None,
    ) -> ET.Element:
        """recurses through python dict of the json passed in and creates xml element tree

        Args:
            data (dict): python dictionary of the json passed in
            xml_tree_node (Optional[ET.Element], optional): root node for xml tree being created.
                Defaults to None.
            json_section (Optional[Definitions], optional): section of the json file too look at.
                Defaults to None.

        Returns:
            ET.Element: root node for xml tree created
        """
        if json_section:
            xml_tree_node = ET.Element(json_section)

        if "list" not in str(type(data)):
            data = [data]

        for item in data:
            for key, value in item.items():
                if isinstance(value, (dict, list)):
                    sub_node = ET.SubElement(xml_tree_node, key)
                    self.create_xml(value, sub_node)
                else:
                    xml_tree_node.set(key, str(value))

        return xml_tree_node


def main():
    parser = XMLToFiles("definition.json")
    parser.create_files()


if __name__ == "__main__":
    main()
