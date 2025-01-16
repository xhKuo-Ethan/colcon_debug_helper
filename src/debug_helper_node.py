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

        # Declare parameters
        self.declare_parameter('workspace_directory', '')  # User can pass workspace directory as a parameter
        self.declare_parameter('log_subdirectory', 'log/latest_build')
        self.declare_parameter('script_output_subdir', 'build/build_scripts')
        self.declare_parameter('xml_output_subdir', 'build/.idea/tools')
        self.declare_parameter('collected_commands_file', 'collected_commands.txt')

        # Retrieve parameters
        self.workspace_directory = self.get_workspace_directory()
        # self.workspace_directory = "/home/xkuo/rtabmap_ws"
        self.log_subdirectory = self.get_parameter('log_subdirectory').get_parameter_value().string_value
        self.script_output_subdir = self.get_parameter('script_output_subdir').get_parameter_value().string_value
        self.xml_output_subdir = self.get_parameter('xml_output_subdir').get_parameter_value().string_value
        self.collected_commands_file = self.get_parameter('collected_commands_file').get_parameter_value().string_value

        # Construct full paths
        self.directory_to_traverse = os.path.join(self.workspace_directory, self.log_subdirectory)
        self.script_output_dir = os.path.join(self.workspace_directory, self.script_output_subdir)

        # Create the .idea/tools directory if it does not exist
        if not os.path.exists(os.path.join(self.workspace_directory, self.xml_output_subdir)):
            os.makedirs( os.path.join(self.workspace_directory, self.xml_output_subdir) )

        self.xml_output_file = os.path.join(self.workspace_directory, self.xml_output_subdir, 'External Tools.xml')
        self.build_directory = os.path.join(self.workspace_directory, 'build')
        self.output_file = os.path.join(self.workspace_directory, self.collected_commands_file)

        # Logging paths
        self.get_logger().info(f"Workspace directory: {self.workspace_directory}")
        self.get_logger().info(f"Log directory: {self.directory_to_traverse}")
        self.get_logger().info(f"Scripts output directory: {self.script_output_dir}")
        self.get_logger().info(f"XML output file: {self.xml_output_file}")
        self.get_logger().info(f"Collected commands output file: {self.output_file}")
        self.get_logger().info(f"Build directory: {self.build_directory}")

        # Execute the main process
        self.process()
        self.get_logger().info(f"All done!")

    # def check_for_quit(self):
    #     # 检查是否按下 q 或 Q 键
    #     if keyboard.is_pressed('q') or keyboard.is_pressed('Q'):
    #         self.get_logger().info('Quit key pressed. Shutting down.')
    #         rclpy.shutdown()  # 关闭 ROS 2 节点

    def get_workspace_directory(self):
        """Determine the workspace directory from parameter, environment variable, or default."""
        workspace_dir_param = self.get_parameter('workspace_directory').get_parameter_value().string_value
        workspace_env = os.getenv('PWD', '')  # Get environment variable
        default_workspace_dir = './'  # Default directory

        # Priority: ROS parameter > Environment variable > Default value
        if workspace_dir_param:
            return workspace_dir_param
        elif workspace_env:
            return workspace_env
        else:
            self.get_logger().warn(f"Workspace directory not specified. Using default: {default_workspace_dir}")
            return default_workspace_dir

    @staticmethod
    def prettify_xml_with_indent(elem, level=0):
        """Recursive function to format XML output by adding indentation and newlines."""
        indent = "  "  # Two spaces for each level of indentation
        if len(elem):  # If there are child elements
            elem.text = "\n" + indent * (level + 1)
            for child in elem:
                DebugHelperNode.prettify_xml_with_indent(child, level + 1)
            child.tail = "\n" + indent * level
        else:
            elem.text = None  # Leaf nodes do not need extra blank lines
        elem.tail = "\n" + indent * level if level else "\n"

    @property
    def traverse_and_collect_logs(self):
        log_data = {}  # Use a dictionary to store folder names and command lists
        command_pattern = re.compile(r'(/usr/bin/cmake.*?)(?=\n|$)', re.DOTALL)

        # Traverse all subdirectories in the specified directory
        for root, dirs, files in os.walk(self.directory_to_traverse):
            if 'command.log' in files:
                log_file_path = os.path.join(root, 'command.log')
                try:
                    with open(log_file_path, 'r') as log_file:
                        log_content = log_file.read()  # Read the log file content
                        folder_name = os.path.basename(root)  # Get the folder name

                        # Use regular expressions to extract command lines
                        matches = command_pattern.findall(log_content)
                        if matches:
                            # Initialize an empty list if the folder's command list does not exist
                            if folder_name not in log_data:
                                log_data[folder_name] = []
                                seen_commands = set()  # For deduplication

                            # Iterate over the matched commands, deduplicate and maintain order
                            for match in matches:
                                stripped_command = match.strip()
                                if stripped_command not in seen_commands:
                                    log_data[folder_name].append(stripped_command)
                                    seen_commands.add(stripped_command)  # Add to the set of seen commands
                except Exception as e:
                    self.get_logger().error(f"Failed to read file {log_file_path}: {e}")

        return log_data


    def save_log_data(self, log_data):
        try:
            with open(self.output_file, 'w') as f:
                for folder_name, commands in log_data.items():
                    f.write(f"Folder: {folder_name}\n")
                    f.write("Commands:\n")
                    for command in commands:  # Output commands in order
                        f.write(f"{command}\n")
                    f.write("\n" + "=" * 50 + "\n\n")
            self.get_logger().info(f"Log data saved to {self.output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save log data to file {self.output_file}: {e}")

    def create_build_scripts(self, log_data):
        # Create the output directory if it does not exist
        if not os.path.exists(self.script_output_dir):
            os.makedirs(self.script_output_dir)

        for folder_name, commands in log_data.items():
            script_name = f"build_{folder_name}.sh"
            script_path = os.path.join(self.script_output_dir, script_name)

            if os.path.exists(script_path):
                self.get_logger().info(f"Script {script_path} already exists, skipping creation.")
                continue

            try:
                with open(script_path, 'w') as f:
                    for command in commands:
                        f.write(f"{command}\n")
                os.chmod(script_path, 0o755)
                self.get_logger().info(f"Created script: {script_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to create script for {folder_name}: {e}")

    def generate_xml_from_scripts(self):
        toolSet = ET.Element("toolSet", name="External Tools")
        for script_name in os.listdir(self.script_output_dir):
            if script_name.endswith(".sh"):
                folder_name = script_name.replace("build_", "").replace(".sh", "")
                # script_path = os.path.join(self.script_output_dir, script_name)
                working_dir = os.path.join(self.workspace_directory, "build", folder_name)

                tool = ET.SubElement(toolSet, "tool", name=folder_name, description=folder_name,
                                     showInMainMenu="false", showInEditor="false",
                                     showInProject="false", showInSearchPopup="false",
                                     disabled="false", useConsole="true",
                                     showConsoleOnStdOut="false", showConsoleOnStdErr="false",
                                     synchronizeAfterRun="true")

                exec_elem = ET.SubElement(tool, "exec")
                ET.SubElement(exec_elem, "option", name="COMMAND", value=f"{self.script_output_dir}/{script_name}")
                ET.SubElement(exec_elem, "option", name="PARAMETERS", value="")
                ET.SubElement(exec_elem, "option", name="WORKING_DIRECTORY", value=working_dir)

        self.prettify_xml_with_indent(toolSet)

        try:
            os.makedirs(os.path.dirname(self.xml_output_file), exist_ok=True)
            tree = ET.ElementTree(toolSet)
            tree.write(self.xml_output_file, xml_declaration=False)
            self.get_logger().info(f"XML file generated: {self.xml_output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate XML file: {e}")
    def generate_custom_targets_from_external_tools(self):
        """从 External Tools.xml 文件读取内容并生成 customTargets.xml 文件"""
        external_tools_file = self.xml_output_file  # External Tools.xml 文件路径
        # custom_targets_file = os.path.join(self.workspace_directory, self.xml_output_subdir, 'customTargets.xml')  # 输出路径
        custom_targets_file  = os.path.join(self.build_directory, ".idea", "customTargets.xml")
        try:
            # 检查 External Tools.xml 是否存在
            if not os.path.exists(external_tools_file):
                self.get_logger().error(f"External Tools.xml 文件不存在: {external_tools_file}")
                return

            # 解析 External Tools.xml 文件
            tree = ET.parse(external_tools_file)
            root = tree.getroot()

            # 创建 customTargets.xml 的根节点
            project = ET.Element("project", version="4")
            component = ET.SubElement(project, "component", name="CLionExternalBuildManager")

            # 遍历 External Tools.xml 的 tool 节点
            for tool in root.findall("tool"):
                tool_name = tool.get("name")
                if not tool_name:
                    self.get_logger().warning("跳过没有 name 属性的 tool 标签")
                    continue

                # 为每个 tool 生成 target 节点和 configuration 节点
                target_id = str(uuid.uuid4())  # 生成唯一 ID
                config_id = str(uuid.uuid4())

                target = ET.SubElement(component, "target", id=target_id, name=tool_name, defaultType="TOOL")
                configuration = ET.SubElement(target, "configuration", id=config_id, name=tool_name)
                build = ET.SubElement(configuration, "build", type="TOOL")
                ET.SubElement(build, "tool", actionId=f"Tool_External Tools_{tool_name}")

            # 美化 XML
            self.prettify_xml_with_indent(project)

            # 写入 customTargets.xml 文件
            tree = ET.ElementTree(project)
            tree.write(custom_targets_file, xml_declaration=True, encoding="UTF-8")
            self.get_logger().info(f"customTargets.xml 文件生成成功: {custom_targets_file}")

        except Exception as e:
            self.get_logger().error(f"生成 customTargets.xml 时发生错误: {e}")


    def copy_scripts_to_build_directory(self):
        destination = os.path.join(self.build_directory, 'build_scripts')
        try:
            if os.path.exists(destination):
                shutil.rmtree(destination)
            shutil.copytree(self.script_output_dir, destination)
            self.get_logger().info(f"Scripts folder copied to build directory: {destination}")
        except Exception as e:
            self.get_logger().error(f"Failed to copy scripts to build directory: {e}")

    def process(self):
        log_data = self.traverse_and_collect_logs
        # self.save_log_data(log_data)
        self.create_build_scripts(log_data)
        self.generate_xml_from_scripts()
        self.generate_custom_targets_from_external_tools()
        # self.copy_scripts_to_build_directory()

