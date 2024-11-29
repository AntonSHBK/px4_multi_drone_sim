#!/usr/bin/env python3

import yaml
import os
import sys
import logging
from dataclasses import dataclass

import numpy as np

from geometry_msgs.msg import Vector3


def load_yaml_params(file_path):
    """Загружаем параметры из YAML файла с логированием ошибок"""
    if not os.path.isfile(file_path):
        logging.error(f"YAML файл не найден: {file_path}")
        raise FileNotFoundError(f"YAML файл не найден: {file_path}")
    
    logging.info(f"Загружаем YAML конфигурацию: {file_path}")
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
    
def enu_to_ned(x_enu, y_enu, z_enu):
    """
    Преобразует координаты из ENU в NED. Из глобальной в систему дрона
    """
    x_ned = y_enu
    y_ned = x_enu
    z_ned = -z_enu
    return x_ned, y_ned, z_ned 

def ned_to_enu(x_ned, y_ned, z_ned):
    """
    Преобразует координаты из NED в ENU. Из системы дрона в глобальную.
    """
    x_enu = y_ned
    y_enu = x_ned
    z_enu = -z_ned
    return x_enu, y_enu, z_enu

def convert_to_FLU(x, y, yaw):
    """Преобразуем координаты и скорости из глобальной системы координат в FLU (Forward, Left, Up)"""
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    # Преобразуем координаты/скорости из глобальных в FLU
    x_flu = x * cos_yaw + y * sin_yaw
    y_flu = -x * sin_yaw + y * cos_yaw
    
    return x_flu, y_flu

def local_to_global(local_position: Vector3, global_reference_position: Vector3, global_reference_orientation: list) -> Vector3:
    """
    Преобразует координаты из локальной системы координат в глобальную (ENU).

    :param local_position: Локальная позиция (ENU).
    :param global_reference_position: Глобальная позиция дрона (ENU).
    :param global_reference_orientation: Глобальная ориентация дрона (yaw, pitch, roll) в радианах.
    :return: Глобальная позиция в системе координат ENU.
    """
    yaw = global_reference_orientation[0]  # Предполагаем, что yaw — это первый элемент
    cos_yaw = float(np.cos(yaw))
    sin_yaw = float(np.sin(yaw))

    # Поворот локальной позиции в глобальную
    global_x = local_position.x * cos_yaw - local_position.y * sin_yaw
    global_y = local_position.x * sin_yaw + local_position.y * cos_yaw
    global_z = local_position.z  # Z не изменяется для поворота по yaw

    # Смещение относительно глобальной позиции
    global_x += global_reference_position.x
    global_y += global_reference_position.y
    global_z += global_reference_position.z

    return Vector3(x=global_x, y=global_y, z=global_z)             
    
def global_to_local(global_position: Vector3, global_reference_position: Vector3, global_reference_orientation: list) -> Vector3:
    """
    Преобразует координаты из глобальной системы координат в локальную (ENU).

    :param global_position: Глобальная позиция (ENU).
    :param global_reference_position: Позиция центра локальной системы координат в глобальной (ENU).
    :param global_reference_orientation: Ориентация центра локальной системы координат (yaw, pitch, roll) в радианах.
    :return: Локальная позиция в системе координат ENU.
    """
    yaw = global_reference_orientation[0]  # Предполагаем, что yaw — это первый элемент
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    # Вычитаем смещение
    delta_x = global_position.x - global_reference_position.x
    delta_y = global_position.y - global_reference_position.y
    delta_z = global_position.z - global_reference_position.z

    # Применяем обратное вращение
    local_x = delta_x * cos_yaw + delta_y * sin_yaw
    local_y = -delta_x * sin_yaw + delta_y * cos_yaw
    local_z = delta_z  # Z не изменяется для поворота по yaw

    return Vector3(x=local_x, y=local_y, z=local_z)

def calculate_yaw_towards_target(current_position: Vector3, target_position: Vector3):
        """
        Рассчитывает yaw в ENU для направления от текущей позиции к целевой.
        """
        delta_x = target_position.x - current_position.x
        delta_y = target_position.y - current_position.y        
        yaw_ENU = np.arctan2(delta_y, delta_x)        
        return float(yaw_ENU)