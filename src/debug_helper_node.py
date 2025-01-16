#!/usr/bin/env python3

import os
import re
import shutil
import uuid
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node


class DebugHelperNode(Node):
    def __init__(self):
        super().__init__('debug_helper_node')

        # Declare parameters with default values
        self.declare_parameter('workspace_directory', '')
        self.declare_parameter('log_subdirectory', 'log/latest_build')
        self.declare_parameter('script_output_subdir', 'build/build_scripts')
        self.declare_parameter('xml_output_subdir', 'build/.idea/tools')
        self.declare_parameter('collected_commands_file', 'collected_commands.txt')

        # Initialize directories and files
        self._initialize_paths()

        # Log important paths for debugging purposes
        self._log_paths()

        # Execute the main process
        self.process()
        self.get_logger().info("All done!")

    def _initialize_paths(self):
        """Initialize directory and file paths based on parameters."""
        self.workspace_directory = self.get_parameter('workspace_directory').get_parameter_value().string_value
        self.log_subdirectory = self.get_parameter('log_subdirectory').get_parameter_value().string_value
        self.script_output_subdir = self.get_parameter('script_output_subdir').get_parameter_value().string_value
        self.xml_output_subdir = self.get_parameter('xml_output_subdir').get_parameter_value().string_value
        self.collected_commands_file = self.get_parameter('collected_commands_file').get_parameter_value().string_value

        # Derived paths
        self.directory_to_traverse = os.path.join(self.workspace_directory, self.log_subdirectory)
        self.script_output_dir = os.path.join(self.workspace_directory, self.script_output_subdir)
        self.xml_output_file = os.path.join(self.workspace_directory, self.xml_output_subdir, 'External Tools.xml')
        self.build_directory = os.path.join(self.workspace_directory, 'build')
        self.output_file = os.path.join(self.workspace_directory, self.collected_commands_file)

        # Ensure XML output directory exists
        os.makedirs(os.path.dirname(self.xml_output_file), exist_ok=True)

    def _log_paths(self):
        """Log the key paths for better traceability."""
        self.get_logger().info(f"Workspace directory: {self.workspace_directory}")
        self.get_logger().info(f"Log directory: {self.directory_to_traverse}")
        self.get_logger().info(f"Scripts output directory: {self.script_output_dir}")
        self.get_logger().info(f"XML output file: {self.xml_output_file}")
        self.get_logger().info(f"Collected commands output file: {self.output_file}")
        self.get_logger().info(f"Build directory: {self.build_directory}")

    @staticmethod
    def _prettify_xml(elem, level=0):
        """Format XML for better readability."""
        indent = "  "
        if len(elem):
            elem.text = "\n" + indent * (level + 1)
            for child in elem:
                DebugHelperNode._prettify_xml(child, level + 1)
            child.tail = "\n" + indent * level
        else:
            elem.text = None
        elem.tail = "\n" + indent * level if level else "\n"

    def _traverse_logs(self):
        """Traverse log files and collect commands."""
        log_data = {}
        command_pattern = re.compile(r'(/usr/bin/cmake.*?)(?=\n|$)', re.DOTALL)

        for root, _, files in os.walk(self.directory_to_traverse):
            if 'command.log' in files:
                log_file_path = os.path.join(root, 'command.log')
                try:
                    with open(log_file_path, 'r') as log_file:
                        log_content = log_file.read()
                        folder_name = os.path.basename(root)
                        matches = command_pattern.findall(log_content)

                        if matches:
                            log_data[folder_name] = list(dict.fromkeys(matches))  # Deduplicate while preserving order
                except Exception as e:
                    self.get_logger().error(f"Failed to read {log_file_path}: {e}")

        return log_data

    def _create_build_scripts(self, log_data):
        """Create build scripts from collected commands."""
        os.makedirs(self.script_output_dir, exist_ok=True)

        for folder_name, commands in log_data.items():
            script_path = os.path.join(self.script_output_dir, f"build_{folder_name}.sh")

            if os.path.exists(script_path):
                self.get_logger().info(f"Script {script_path} already exists, skipping.")
                continue

            try:
                with open(script_path, 'w') as script_file:
                    script_file.write("\n".join(commands))
                os.chmod(script_path, 0o755)
                self.get_logger().info(f"Created script: {script_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to create script {script_path}: {e}")

    def _generate_xml(self):
        """Generate XML configuration for external tools."""
        tool_set = ET.Element("toolSet", name="External Tools")

        for script_name in os.listdir(self.script_output_dir):
            if script_name.endswith(".sh"):
                folder_name = script_name.replace("build_", "").replace(".sh", "")
                working_dir = os.path.join(self.workspace_directory, "build", folder_name)

                tool = ET.SubElement(tool_set, "tool", name=folder_name)
                exec_elem = ET.SubElement(tool, "exec")
                ET.SubElement(exec_elem, "option", name="COMMAND", value=f"{self.script_output_dir}/{script_name}")
                ET.SubElement(exec_elem, "option", name="WORKING_DIRECTORY", value=working_dir)

        self._prettify_xml(tool_set)

        try:
            tree = ET.ElementTree(tool_set)
            tree.write(self.xml_output_file, xml_declaration=False)
            self.get_logger().info(f"XML file generated: {self.xml_output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate XML: {e}")

    def _generate_custom_targets(self):
        """Generate custom targets XML from external tools."""
        custom_targets_file = os.path.join(self.build_directory, '.idea', 'customTargets.xml')
        external_tools_file = self.xml_output_file

        if not os.path.exists(external_tools_file):
            self.get_logger().error(f"External Tools XML not found: {external_tools_file}")
            return

        try:
            tree = ET.parse(external_tools_file)
            root = tree.getroot()

            project = ET.Element("project", version="4")
            component = ET.SubElement(project, "component", name="CLionExternalBuildManager")

            for tool in root.findall("tool"):
                tool_name = tool.get("name")
                if not tool_name:
                    continue

                target_id = str(uuid.uuid4())
                configuration_id = str(uuid.uuid4())

                target = ET.SubElement(component, "target", id=target_id, name=tool_name, defaultType="TOOL")
                configuration = ET.SubElement(target, "configuration", id=configuration_id, name=tool_name)
                build = ET.SubElement(configuration, "build", type="TOOL")
                ET.SubElement(build, "tool", actionId=f"Tool_External Tools_{tool_name}")

            self._prettify_xml(project)
            os.makedirs(os.path.dirname(custom_targets_file), exist_ok=True)
            tree = ET.ElementTree(project)
            tree.write(custom_targets_file, xml_declaration=False)
            self.get_logger().info(f"Custom targets XML generated: {custom_targets_file}")
        except Exception as e:
            self.get_logger().error(f"Error generating custom targets: {e}")

    def process(self):
        """Main process to collect logs, create scripts, and generate XML files."""
        log_data = self._traverse_logs()
        self._create_build_scripts(log_data)
        self._generate_xml()
        self._generate_custom_targets()


def main(args=None):
    rclpy.init(args=args)
    node = DebugHelperNode()
    # rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()