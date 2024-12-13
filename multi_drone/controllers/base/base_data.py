import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Vector3, Quaternion

from multi_drone.utils.geometry import (
    rotate_ENU_NED,
    rotated_ENU_NED_quaternion,
    transform_coordinates,
    transform_orientation    
)


class BaseData:
    def _validate_and_convert(self, value):
        if not isinstance(value, (float, int)):
            try:
                value = float(value)
            except ValueError:
                raise TypeError(f"Значение {value} не может быть преобразовано в float.")
        return float(value)

    def to_dict(self):
        """
        Формирует словарь всех атрибутов объекта.

        :return: Словарь с атрибутами объекта.
        """
        return {key: value for key, value in self.__dict__.items() if not key.startswith("_")}


class EulerData(BaseData):
    """
    Класс для представления ориентации в виде углов Эйлера (roll, pitch, yaw).

    Атрибуты:
    - roll (float): Угол наклона вокруг оси X.
    - pitch (float): Угол наклона вокруг оси Y.
    - yaw (float): Угол наклона вокруг оси Z.
    """

    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0):
        # Храним данные в виде numpy массива
        self._euler = np.array([self._validate_and_convert(roll),
                                self._validate_and_convert(pitch),
                                self._validate_and_convert(yaw)])

    @property
    def roll(self):
        return self._euler[0]

    @roll.setter
    def roll(self, value):
        self._euler[0] = self._validate_and_convert(value)

    @property
    def pitch(self):
        return self._euler[1]

    @pitch.setter
    def pitch(self, value):
        self._euler[1] = self._validate_and_convert(value)

    @property
    def yaw(self):
        return self._euler[2]

    @yaw.setter
    def yaw(self, value):
        self._euler[2] = self._validate_and_convert(value)

    def to_array(self) -> np.ndarray:
        """
        Возвращает углы Эйлера в виде numpy массива.
        """
        return self._euler.copy()

    def to_vector3(self) -> Vector3:
        """
        Формирует объект Vector3 из geometry_msgs.msg.
        """
        return Vector3(x=self.roll, y=self.pitch, z=self.yaw)

    def update(self, roll=None, pitch=None, yaw=None):
        """
        Обновляет значения углов Эйлера.
        """
        if roll is not None:
            self.roll = roll
        if pitch is not None:
            self.pitch = pitch
        if yaw is not None:
            self.yaw = yaw
    
    def update_from_array(self, euler_array: np.ndarray):
        """
        Обновляет углы Эйлера из numpy массива.

        :param euler_array: numpy массив из трех элементов [roll, pitch, yaw].
        """
        if euler_array.shape != (3,):
            raise ValueError(f"Ожидается массив размерности (3,), получено {euler_array.shape}")
        
        self._euler = np.array([
            self._validate_and_convert(euler_array[0]),
            self._validate_and_convert(euler_array[1]),
            self._validate_and_convert(euler_array[2])
        ])
        
    def to_quaternion(self) -> np.ndarray:
        """
        Преобразует углы Эйлера (roll, pitch, yaw) в кватернион с использованием scipy.

        :return: numpy массив [x, y, z, w], представляющий кватернион.
        """
        quaternion = Rotation.from_euler('zyx', self._euler, degrees=False).as_quat()
        return quaternion

    def update_from_quaternion(self, quaternion: np.ndarray):
        """
        Обновляет углы Эйлера на основе переданного кватерниона с использованием scipy.

        :param quaternion: numpy массив [x, y, z, w], представляющий кватернион.
        """
        if quaternion.shape != (4,):
            raise ValueError(f"Ожидается массив размерности (4,), получено {quaternion.shape}")
        euler_angles = Rotation.from_quat(quaternion).as_euler('zyx', degrees=False)
        self.update_from_array(euler_angles)
    
    def to_ENU(self):
        return rotated_ENU_NED_quaternion(self.to_array())
    
    def to_NED(self):
        return rotated_ENU_NED_quaternion(self.to_array())
    
    def to_global(self, reference_orientation):
        return transform_orientation(
            source_orientation=self.to_array(),
            reference_orientation=reference_orientation,
            mode='euler',
            euler_order='zyx',       
            transform_type='local_to_global')
    
    def to_local(self, reference_orientation):
        return transform_orientation(
            source_orientation=self.to_array(),
            reference_orientation=reference_orientation,
            mode='euler',     
            euler_order='zyx',     
            transform_type='global_to_local') 
    
    def __call__(self):
        """
        Возвращает ориентацию как массив numpy при вызове объекта.
        """
        return self.to_array()

    def __repr__(self):
        return f"EulerData(roll={self.roll}, pitch={self.pitch}, yaw={self.yaw})"


class PositionData(BaseData):
    """
    Класс для представления позиции в пространстве (x, y, z), адаптированный для использования с numpy.
    """

    def __init__(self, x=0.0, y=0.0, z=0.0):
        # Инициализируем позицию с использованием numpy массива
        self._position = np.array([self._validate_and_convert(x),
                                   self._validate_and_convert(y),
                                   self._validate_and_convert(z)])

    @property
    def x(self):
        return self._position[0]

    @x.setter
    def x(self, value):
        self._position[0] = self._validate_and_convert(value)

    @property
    def y(self):
        return self._position[1]

    @y.setter
    def y(self, value):
        self._position[1] = self._validate_and_convert(value)

    @property
    def z(self):
        return self._position[2]

    @z.setter
    def z(self, value):
        self._position[2] = self._validate_and_convert(value)

    def to_array(self) -> np.ndarray:
        """
        Возвращает координаты как numpy массив.
        """
        return self._position.copy()

    def update(self, x=None, y=None, z=None):
        """
        Обновляет координаты, если переданы новые значения.

        :param x: Новое значение координаты X.
        :param y: Новое значение координаты Y.
        :param z: Новое значение координаты Z.
        """
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if z is not None:
            self.z = z

    def update_from_array(self, array: np.ndarray):
        """
        Обновляет координаты из numpy массива.

        :param array: numpy массив размером 3, содержащий [x, y, z].
        """
        if not isinstance(array, np.ndarray) or array.shape != (3,):
            raise ValueError("Ожидается numpy массив размером (3,) для [x, y, z].")
        self._position[:] = np.array([
            self._validate_and_convert(array[0]),
            self._validate_and_convert(array[1]),
            self._validate_and_convert(array[2])
        ])

    def to_vector3(self) -> Vector3:
        """
        Формирует объект Vector3 из geometry_msgs.msg.

        Returns:
        Vector3: Объект ROS Vector3 с полями x, y, z.
        """
        return Vector3(x=self.x, y=self.y, z=self.z)
    
    def to_ENU(self):
        return rotate_ENU_NED(self.to_array())
    
    def to_NED(self):
        return rotate_ENU_NED(self.to_array())
    
    def to_global(self, reference_position, reference_orientation, rotation_type='quaternion'):
        return transform_coordinates(
            source_position=self.to_array(),
            reference_position=reference_position,
            reference_orientation=reference_orientation,
            transform_type='local_to_global',
            rotation_type=rotation_type)
    
    def to_local(self, reference_position, reference_orientation, rotation_type='quaternion'):
        return transform_orientation(
            source_position=self.to_array(),
            reference_position=reference_position,
            reference_orientation=reference_orientation,
            transform_type='global_to_local',
            rotation_type=rotation_type)

    def __call__(self):
        """
        Возвращает позицию как массив numpy при вызове объекта.
        """
        return self.to_array()
    
    def __repr__(self):
        return f"Position(x={self.x}, y={self.y}, z={self.z})"


class VelocityData(BaseData):
    """
    Класс для представления скорости в пространстве (vx, vy, vz), адаптированный для использования с numpy.

    Атрибуты:
    - vx (float): Скорость по оси X.
    - vy (float): Скорость по оси Y.
    - vz (float): Скорость по оси Z.
    """

    def __init__(self, vx=0.0, vy=0.0, vz=0.0):
        self._velocity = np.array([self._validate_and_convert(vx),
                                   self._validate_and_convert(vy),
                                   self._validate_and_convert(vz)])

    @property
    def vx(self):
        return self._velocity[0]

    @vx.setter
    def vx(self, value):
        self._velocity[0] = self._validate_and_convert(value)

    @property
    def vy(self):
        return self._velocity[1]

    @vy.setter
    def vy(self, value):
        self._velocity[1] = self._validate_and_convert(value)

    @property
    def vz(self):
        return self._velocity[2]

    @vz.setter
    def vz(self, value):
        self._velocity[2] = self._validate_and_convert(value)

    def to_vector3(self) -> Vector3:
        """
        Формирует объект Vector3 из geometry_msgs.msg.

        Returns:
        Vector3: Объект ROS Vector3 с полями vx, vy, vz.
        """
        return Vector3(x=self.vx, y=self.vy, z=self.vz)

    def to_array(self) -> np.ndarray:
        """
        Возвращает скорость как numpy массив.
        """
        return self._velocity.copy()

    def update(self, vx=None, vy=None, vz=None):
        """
        Обновляет скорость, если переданы новые значения.

        :param vx: Новое значение скорости по оси X.
        :param vy: Новое значение скорости по оси Y.
        :param vz: Новое значение скорости по оси Z.
        """
        if vx is not None:
            self.vx = vx
        if vy is not None:
            self.vy = vy
        if vz is not None:
            self.vz = vz

    def update_from_array(self, array: np.ndarray):
        """
        Обновляет скорость из numpy массива.

        :param array: numpy массив размером 3, содержащий [vx, vy, vz].
        """
        if not isinstance(array, np.ndarray) or array.shape != (3,):
            raise ValueError("Ожидается numpy массив размером (3,) для [vx, vy, vz].")
        self._velocity[:] = np.array([
            self._validate_and_convert(array[0]),
            self._validate_and_convert(array[1]),
            self._validate_and_convert(array[2])
        ])
        
    def to_ENU(self):
        return rotate_ENU_NED(self.to_array())
    
    def to_NED(self):
        return rotate_ENU_NED(self.to_array())
    
    def to_global(self, reference_orientation, rotation_type='quaternion'):
        return transform_coordinates(
            source_position=self.to_array(),
            reference_position=np.asarray([0,0,0]),
            reference_orientation=reference_orientation,
            transform_type='local_to_global',
            rotation_type=rotation_type)
    
    def to_local(self, reference_orientation, rotation_type='quaternion'):
        return transform_orientation(
            source_position=self.to_array(),
            reference_position=np.asarray([0,0,0]),
            reference_orientation=reference_orientation,
            transform_type='global_to_local',
            rotation_type=rotation_type)
    
    def __call__(self):
        """
        Возвращает скорости как массив numpy при вызове объекта.
        """
        return self.to_array()

    def __repr__(self):
        return f"Velocity(vx={self.vx}, vy={self.vy}, vz={self.vz})"


class QuaternionData(BaseData):
    """
    Класс для представления кватерниона, адаптированный для работы с numpy.

    Атрибуты:
    - quaternion (np.ndarray): Массив numpy, содержащий [x, y, z, w].
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self._quaternion = np.array([self._validate_and_convert(x),
                                    self._validate_and_convert(y),
                                    self._validate_and_convert(z),
                                    self._validate_and_convert(w)])
        self.normalize()

    @property
    def x(self):
        return self._quaternion[0]

    @x.setter
    def x(self, value):
        self._quaternion[0] = self._validate_and_convert(value)

    @property
    def y(self):
        return self._quaternion[1]

    @y.setter
    def y(self, value):
        self._quaternion[1] = self._validate_and_convert(value)

    @property
    def z(self):
        return self._quaternion[2]

    @z.setter
    def z(self, value):
        self._quaternion[2] = self._validate_and_convert(value)

    @property
    def w(self):
        return self._quaternion[3]

    @w.setter
    def w(self, value):
        self._quaternion[3] = self._validate_and_convert(value)

    def normalize(self):
        """
        Нормализует кватернион.
        """
        norm = np.linalg.norm(self._quaternion)
        if norm > 0:
            self._quaternion /= norm

    def to_array(self) -> np.ndarray:
        """
        Возвращает кватернион в виде numpy массива.
        """
        return self._quaternion.copy()

    def update(self, x=None, y=None, z=None, w=None):
        """
        Обновляет значения кватерниона, если переданы новые значения.

        :param x: Координата X (или None, если не обновляется).
        :param y: Координата Y (или None, если не обновляется).
        :param z: Координата Z (или None, если не обновляется).
        :param w: Координата W (или None, если не обновляется).
        """
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if z is not None:
            self.z = z
        if w is not None:
            self.w = w
        self.normalize()
    
    def update_from_array(self, array: np.ndarray):
        """
        Обновляет кватернион из numpy массива.

        :param array: numpy массив размером 4, содержащий [x, y, z, w].
        """
        if not isinstance(array, np.ndarray) or array.shape != (4,):
            raise ValueError("Ожидается numpy массив размером (4,) для [x, y, z, w].")
        self._quaternion[:] = np.array([self._validate_and_convert(array[0]),
                                    self._validate_and_convert(array[1]),
                                    self._validate_and_convert(array[2]),
                                    self._validate_and_convert(array[3])])
        self.normalize()

    def to_ros_quaternion(self):
        """
        Формирует объект Quaternion из geometry_msgs.msg.

        Returns:
        Quaternion: Объект ROS Quaternion с полями x, y, z, w.
        """
        return Quaternion(x=self.x, y=self.y, z=self.z, w=self.w)

    def to_euler(self) -> np.ndarray:
        """
        Преобразует кватернион в углы Эйлера с использованием scipy.

        Returns:
        np.ndarray: Массив numpy [roll, pitch, yaw] в радианах.
        """
        # Преобразуем кватернион в углы Эйлера
        euler_angles = Rotation.from_quat(self._quaternion).as_euler('zyx', degrees=False)
        return euler_angles

    def update_from_euler(self, euler: np.ndarray):
        """
        Задает кватернион из углов Эйлера с использованием scipy.

        :param euler: Массив NumPy с углами (roll, pitch, yaw) в радианах.
        """
        if not isinstance(euler, np.ndarray) or euler.shape != (3,):
            raise ValueError("Параметр euler должен быть массивом NumPy размерности (3,)")
        quaternion = Rotation.from_euler('xyz', euler, degrees=False).as_quat()
        self.update_from_array(quaternion)
        
    def to_ENU(self):
        return rotated_ENU_NED_quaternion(self.to_array())
    
    def to_NED(self):
        return rotated_ENU_NED_quaternion(self.to_array())
    
    def to_global(self, reference_orientation):
        return transform_orientation(
            source_orientation=self.to_array(),
            reference_orientation=reference_orientation,
            mode='quaternion',          
            transform_type='local_to_global')
    
    def to_local(self, reference_orientation):
        return transform_orientation(
            source_orientation=self.to_array(),
            reference_orientation=reference_orientation,
            mode='quaternion',          
            transform_type='global_to_local')        

    def __call__(self):
        """
        Возвращает кватернион как массив numpy при вызове объекта.
        """
        return self.to_array()
    
    def __repr__(self):
        return f"QuaternionData(x={self.x}, y={self.y}, z={self.z}, w={self.w})"


class OrientationData:
    """
    Класс для представления ориентации с поддержкой как углов Эйлера, так и кватернионов.

    Атрибуты:
    - euler (EulerData): Объект углов Эйлера (roll, pitch, yaw).
    - quaternion (QuaternionData): Объект кватернионов (x, y, z, w).
    """

    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0, x=0.0, y=0.0, z=0.0, w=1.0):
        # Инициализация углов Эйлера и кватерниона
        self.euler = EulerData(roll, pitch, yaw)
        self.quaternion = QuaternionData(x, y, z, w)
        self._sync_quaternion_from_euler()

    def _sync_quaternion_from_euler(self):
        """Синхронизирует кватернион с текущими углами Эйлера."""
        self.quaternion.update_from_euler(self.euler.to_array())

    def _sync_euler_from_quaternion(self):
        """Синхронизирует углы Эйлера с текущим кватернионом."""
        self.euler.update_from_array(self.quaternion.to_euler())

    def update_from_euler(self, roll=None, pitch=None, yaw=None):
        """
        Обновляет ориентацию на основе углов Эйлера.
        """
        self.euler.update(roll=roll, pitch=pitch, yaw=yaw)
        self._sync_quaternion_from_euler()

    def update_from_quaternion(self, x=None, y=None, z=None, w=None):
        """
        Обновляет ориентацию на основе кватерниона.
        """
        self.quaternion.update(x=x, y=y, z=z, w=w)
        self._sync_euler_from_quaternion()

    def update_from_euler_array(self, euler_array: np.ndarray):
        """
        Обновляет ориентацию на основе массива углов Эйлера.

        :param euler_array: numpy массив из трех элементов [roll, pitch, yaw].
        """
        self.euler.update_from_array(euler_array)
        self._sync_quaternion_from_euler()

    def update_from_quaternion_array(self, quaternion_array: np.ndarray):
        """
        Обновляет ориентацию на основе массива кватерниона.

        :param quaternion_array: numpy массив из четырех элементов [x, y, z, w].
        """
        self.quaternion.update_from_array(quaternion_array)
        self._sync_euler_from_quaternion()

    def normalize_quaternion(self):
        """
        Нормализует кватернион и синхронизирует углы Эйлера.
        """
        self.quaternion.normalize()
        self._sync_euler_from_quaternion()

    def to_dict(self):
        """
        Возвращает ориентацию в виде словаря.
        """
        return {
            "euler": self.euler.to_dict(),
            "quaternion": self.quaternion.to_dict(),
        }

    def to_ros_quaternion(self):
        """
        Возвращает объект ROS Quaternion.
        """
        return self.quaternion.to_ros_quaternion()

    def to_vector3(self):
        """
        Возвращает углы Эйлера как Vector3.
        """
        return self.euler.to_vector3()

    def __repr__(self):
        return f"OrientationData(euler={self.euler}, quaternion={self.quaternion})"
