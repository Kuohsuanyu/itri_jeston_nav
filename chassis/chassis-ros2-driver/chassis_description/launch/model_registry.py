"""Shared loader for the chassis model registry (chassis_description/config/models.yaml).

Launch files import this module instead of hard-coding file names, so adding a
new chassis model only requires editing models.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory

import yaml

PACKAGE = 'chassis_description'


def registry_path():
    """Return the absolute path of the installed models.yaml."""
    return os.path.join(get_package_share_directory(PACKAGE), 'config', 'models.yaml')


def load_registry():
    """Load models.yaml and return it as a dict."""
    path = registry_path()
    with open(path, 'r', encoding='utf-8') as handle:
        registry = yaml.safe_load(handle)

    if not registry or not registry.get('models'):
        raise RuntimeError(f'No models declared in {path}')

    return registry


def default_model():
    """Return the model name used when the launch argument is omitted."""
    registry = load_registry()
    default = registry.get('default_model')

    if default is None:
        return sorted(registry['models'])[0]

    if default not in registry['models']:
        raise RuntimeError(
            f"default_model '{default}' is not declared under models: in {registry_path()}"
        )

    return default


def model_entry(model):
    """Return the registry entry of *model*, listing valid names when unknown."""
    models = load_registry()['models']

    if model not in models:
        available = ', '.join(sorted(models))
        raise RuntimeError(
            f"Unknown chassis model '{model}'. Available models: {available} "
            f'(declared in {registry_path()})'
        )

    return models[model]


def xacro_path(model):
    """Return the absolute path of the xacro file describing *model*."""
    entry = model_entry(model)
    path = os.path.join(get_package_share_directory(PACKAGE), 'urdf', entry['xacro'])

    if not os.path.exists(path):
        raise RuntimeError(
            f"Xacro '{entry['xacro']}' declared for model '{model}' is missing from the "
            f'install space: {path}'
        )

    return path


def rviz_path(model):
    """Return the absolute path of the RViz config used to display *model*."""
    entry = model_entry(model)
    return os.path.join(get_package_share_directory(PACKAGE), 'rviz', entry['rviz'])
