#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Vector3

    
def rotate_ENU_NED(array: np.ndarray) -> np.ndarray:
    """
    Преобразует координаты из ENU в NED и наоборот.

    :param enu: Вектор координат в ENU (np.ndarray с [x, y, z]).
    :return: Вектор координат в NED (np.ndarray с [x, y, z]).
    """
    if array.shape != (3,):
        raise ValueError("Входной вектор ENU должен быть одномерным массивом размерности 3.")
    
    ned = np.array([array[1], array[0], -array[2]])
    return ned

def rotated_ENU_NED_quaternion(q: np.ndarray) -> np.ndarray:
    """
    Преобразует кватернион из системы координат ENU в NED и наоборот.

    :param quaternion: Кватернион в формате np.array([x, y, z, w]).
    :return: Преобразованный кватернион в формате np.array([x, y, z, w]).
    """
    if q.shape != (4,):
        raise ValueError("Кватернион должен быть массивом с 4 элементами.")
    q = np.array([q[1], q[0], -q[2], q[3]])
    return q

from typing import Union, Literal
import numpy as np
from scipy.spatial.transform import Rotation as R

def transform_coordinates(
    source_position: np.ndarray,
    reference_position: np.ndarray,
    reference_orientation: Union[np.ndarray, list],
    transform_type: Literal["local_to_global", "global_to_local"],
    rotation_type: Literal["euler", "quaternion"] = "euler",
    sequence: str = "zyx",
) -> np.ndarray:
    """
    Преобразует координаты между локальной и глобальной системами.

    :param source_position: Исходная позиция в виде np.ndarray [x, y, z].
    :param reference_position: Позиция центра координат в другой системе в виде np.ndarray [x, y, z].
    :param reference_orientation: Ориентация центра:
        - Если rotation_type='euler': ориентация в виде np.ndarray [yaw, pitch, roll] (в радианах).
        - Если rotation_type='quaternion': ориентация в виде np.ndarray [x, y, z, w].
    :param transform_type: Тип преобразования: 'local_to_global' или 'global_to_local'.
    :param rotation_type: Тип вращения: 'euler' (по умолчанию) или 'quaternion'.
    :param sequence: Последовательность вращений для углов Эйлера (по умолчанию 'zyx').
    :return: Преобразованная позиция в виде np.ndarray [x, y, z].
    """
    if rotation_type == "euler":
        rotation = R.from_euler(sequence, reference_orientation)
    elif rotation_type == "quaternion":
        rotation = R.from_quat(reference_orientation)
    else:
        raise ValueError(f"Неподдерживаемый тип вращения: {rotation_type}. Доступны: 'euler', 'quaternion'.")

    if transform_type == "local_to_global":
        # Преобразуем локальную позицию в глобальную
        rotated_position = rotation.apply(source_position)
        transformed_position = rotated_position + reference_position
    elif transform_type == "global_to_local":
        # Преобразуем глобальную позицию в локальную
        delta_position = source_position - reference_position
        transformed_position = rotation.inv().apply(delta_position)
    else:
        raise ValueError(f"Неподдерживаемый тип преобразования: {transform_type}. Доступны: 'local_to_global', 'global_to_local'.")

    return transformed_position

def transform_orientation(
        source_orientation: np.ndarray,
        reference_orientation: np.ndarray,
        transform_type: Literal["local_to_global", "global_to_local"],
        mode: Literal["quaternion", "euler"] = "quaternion",
        euler_order: str = "zyx"
    ) -> np.ndarray:
    """
    Преобразует ориентацию между локальной и глобальной системами координат.

    Параметры:
    - source_orientation (np.ndarray): Ориентация, которую нужно преобразовать.
        Если mode="quaternion", массив [x, y, z, w].
        Если mode="euler", массив [roll, pitch, yaw].
    - reference_orientation (np.ndarray): Ориентация системы отсчёта.
        Если mode="quaternion", массив [x, y, z, w].
        Если mode="euler", массив [roll, pitch, yaw].
    - transform_type (Literal): Тип преобразования:
        - "local_to_global" — преобразует ориентацию из локальной системы в глобальную.
        - "global_to_local" — преобразует ориентацию из глобальной системы в локальную.
    - mode (Literal): Тип ориентации:
        - "quaternion" — преобразование кватернионов.
        - "euler" — преобразование углов Эйлера.
    - euler_order (str): Порядок осей для углов Эйлера (по умолчанию "zyx").

    Возвращает:
    - np.ndarray: Преобразованная ориентация.
        Если mode="quaternion", массив [x, y, z, w].
        Если mode="euler", массив [roll, pitch, yaw].
    """
    if mode not in ["quaternion", "euler"]:
        raise ValueError(f"Неподдерживаемый режим: {mode}. Доступны: 'quaternion', 'euler'.")

    # Преобразование в Rotation
    if mode == "quaternion":
        source_rotation = R.from_quat(source_orientation)
        reference_rotation = R.from_quat(reference_orientation)
    elif mode == "euler":
        source_rotation = R.from_euler(euler_order, source_orientation)
        reference_rotation = R.from_euler(euler_order, reference_orientation)

    # Выполнение преобразования
    if transform_type == "local_to_global":
        transformed_rotation = reference_rotation * source_rotation
    elif transform_type == "global_to_local":
        transformed_rotation = reference_rotation.inv() * source_rotation
    else:
        raise ValueError(f"Неподдерживаемый тип преобразования: {transform_type}. Доступны: 'local_to_global', 'global_to_local'.")

    # Преобразование обратно в нужный формат
    if mode == "quaternion":
        return transformed_rotation.as_quat()
    elif mode == "euler":
        return transformed_rotation.as_euler(euler_order)

def calculate_yaw_towards_target(current_position: Vector3, target_position: Vector3):
        """
        Рассчитывает yaw в ENU для направления от текущей позиции к целевой.
        """
        delta_x = target_position.x - current_position.x
        delta_y = target_position.y - current_position.y        
        yaw_ENU = np.arctan2(delta_y, delta_x)        
        return float(yaw_ENU)