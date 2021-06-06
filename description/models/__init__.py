# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import enum
import tempfile
from pathlib import Path
from typing import IO, List, Union


def get_models_path() -> str:
    """
    Return the path where the models have been installed.

    Returns:
        A string containing the path of the models.
    """
    models_dir = os.path.join(os.path.dirname(__file__))
    return models_dir + '/'


def get_robot_names() -> List[str]:
    """
    Return the names of the available robots.

    The name of the robot matches with the folder containing its model file.

    Returns:
        A list of strings containing the available models.
    """
    root_dir = get_models_path()
    dirs = [d for d in os.listdir(root_dir) if os.path.isdir(os.path.join(root_dir, d))]
    return [d for d in dirs if not d.startswith("__")]


def get_model_file(robot_name: str) -> str:
    """
    Return the path to the model file of the selected robot.

    Args:
        robot_name: The name of the selected robot.

    Returns:
        The path to the model of the selected robot.
    """
    if robot_name not in get_robot_names():
        raise RuntimeError(f"Failed to find robot '{robot_name}'")

    model_dir = os.path.join(get_models_path(), robot_name)

    if not os.path.isdir(model_dir):
        raise FileNotFoundError(model_dir)  # TODO

    models_found = []

    for root, dirs, files in os.walk(model_dir):
        for file in files:
            if file.endswith((".urdf", ".sdf")):
                models_found.append(file)

    if len(models_found) != 1:
        raise RuntimeError(f"Found multiple models in the same folder: {models_found}")

    model_abs_path = os.path.join(model_dir, models_found[0])
    return model_abs_path


def get_model_string(robot_name: str) -> str:
    """
    Return the string containing the selected robot model.

    Args:
        robot_name: The name of the selected robot.

    Returns:
        A string containing the selected robot model.
    """

    model_file = get_model_file(robot_name=robot_name)

    with open(model_file, "r") as f:
        string = f.read()

    return string


def setup_environment() -> None:
    """
    Configure the environment variables.
    """

    models_path = Path(get_models_path())

    if not models_path.exists():
        raise NotADirectoryError(f"Failed to find path '{models_path}'")

    # Setup the environment to find the models
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] += f":{models_path}"
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = f"{models_path}"

    # Models with mesh files
    # Workaround for https://github.com/osrf/sdformat/issues/227
    models_with_mesh = ["panda", "iCubGazeboV2_5", "iCubGazeboSimpleCollisionsV2_5", "spot"]

    # Setup the environment to find the mesh files
    for model in models_with_mesh:

        model_path = Path(get_models_path()) / model

        if not model_path.exists():
            raise NotADirectoryError(f"Failed to find path '{model_path}'")

        if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
            os.environ["IGN_GAZEBO_RESOURCE_PATH"] += f':{model_path}'
        else:
            os.environ["IGN_GAZEBO_RESOURCE_PATH"] = f'{model_path}'


class ResourceType(enum.Enum):

    SDF_FILE = enum.auto()
    SDF_PATH = enum.auto()
    SDF_STRING = enum.auto()

    URDF_FILE = enum.auto()
    URDF_PATH = enum.auto()
    URDF_STRING = enum.auto()


def get_model_resource(robot_name: str,
                       resource_type: ResourceType = ResourceType.URDF_PATH) \
        -> Union[str, IO]:
    """
    Return the resource of the selected robot.

    Args:
        robot_name: The name of the selected robot.
        resource_type: The type of the desired resource.

    Note:
        If a format conversion is performed, this method creates a temporary file.
        If ``ResourceType.*_FILE`` is used, the file gets automatically deleted when
        it goes out of scope. Instead, if ``ResourceType._*PATH`` is used, the caller
        is responsible to delete it.

    Returns:
        The desired resource of the selected robot.
    """

    stored_model = get_model_file(robot_name=robot_name)

    if not stored_model.endswith((".urdf", ".sdf")):
        raise RuntimeError(f"Model '{robot_name} has no urdf nor sdf resource")

    if stored_model.endswith(".urdf"):

        if resource_type is ResourceType.URDF_PATH:
            return stored_model

        if resource_type is ResourceType.URDF_FILE:
            return open(file=stored_model, mode="r+")

        if resource_type is ResourceType.URDF_STRING:
            with open(file=stored_model, mode="r+") as f:
                return f.read()

        if resource_type in {ResourceType.SDF_FILE,
                             ResourceType.SDF_PATH,
                             ResourceType.SDF_STRING}:
            try:
                from scenario import gazebo as scenario_gazebo
            except ImportError:
                msg = "URDF to SDF conversion requires the 'scenario' package"
                raise RuntimeError(msg)

        if resource_type is ResourceType.SDF_FILE:
            file_name = Path(stored_model).with_suffix('').name
            sdf_file = tempfile.NamedTemporaryFile(mode="w+",
                                                   prefix=file_name,
                                                   suffix=".sdf")
            sdf_string = get_model_resource(robot_name=robot_name,
                                            resource_type=ResourceType.SDF_STRING)
            sdf_file.write(sdf_string)
            return sdf_file

        if resource_type is ResourceType.SDF_PATH:
            file_name = Path(stored_model).with_suffix('').name
            fd, sdf_path = tempfile.mkstemp(prefix=file_name,
                                            suffix=".sdf",
                                            text=True)
            sdf_string = get_model_resource(robot_name=robot_name,
                                            resource_type=ResourceType.SDF_STRING)
            with open(sdf_path, "w") as f:
                f.write(sdf_string)
            return sdf_path

        if resource_type is ResourceType.SDF_STRING:
            from scenario import gazebo as scenario_gazebo
            return scenario_gazebo.urdffile_to_sdfstring(urdf_file=stored_model)

        raise ValueError(resource_type)

    if stored_model.endswith(".sdf"):

        if resource_type is ResourceType.SDF_PATH:
            return stored_model

        if resource_type in {ResourceType.URDF_FILE,
                             ResourceType.URDF_PATH,
                             ResourceType.URDF_STRING}:
            raise ValueError("SDF to URDF conversion is not supported")

        if resource_type is ResourceType.SDF_STRING:
            with open(file=stored_model, mode="r+") as f:
                return f.read()

        if resource_type is ResourceType.SDF_FILE:
            return open(file=stored_model, mode="r+")

        raise ValueError(resource_type)


# Setup the environment when the package is imported
setup_environment()
