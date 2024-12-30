#!/usr/bin/env python3

from typing import List
import subprocess
import os
import shutil

from launch.actions import ExecuteProcess

DEFAULT_DOWNLOAD_DIR = "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip"

def run(cmd):
    process_handle = subprocess.Popen(['bash', '-c', cmd], cwd='.')
    return process_handle

def run_simulation_gazebo(
    world: str = "default",
    gz_partition: str = None,
    gz_ip: str = None,
    model_download_source: str = DEFAULT_DOWNLOAD_DIR,
    px4_model_store: str = "~/simulation-gazebo",
    custom_model_store: str = None,
    custom_model_store_other: List[str] = [],
    overwrite: bool = False,
    headless: bool = False,
    return_cmd: bool = False,
) -> ExecuteProcess:
    """
    Функция для настройки и запуска симуляции Gazebo с поддержкой кастомных моделей и миров.

    Args:
        world (str): Имя файла мира для симуляции (без расширения).
        gz_partition (str): Разделение ресурсов Gazebo (опционально).
        gz_ip (str): IP-адрес для сетевого интерфейса (опционально).
        model_download_source (str): URL или путь для загрузки моделей.
        px4_model_store (str): Локальная директория для хранения моделей PX4.
        custom_model_store (str): Директория для кастомных моделей.
        custom_model_store_other (List[str]): Дополнительные директории для моделей.
        overwrite (bool): Флаг, указывающий на перезапись существующих моделей.
        headless (bool): Флаг для запуска Gazebo без графического интерфейса.

    Returns:
        ExecuteProcess: Объект, представляющий процесс запуска симуляции Gazebo.

    Example:
        >>> gazebo_server = simulation_gazebo(
        >>>     world="test_world",
        >>>     gz_partition="default",
        >>>     gz_ip="127.0.0.1",
        >>>     model_download_source="https://example.com/models.zip",
        >>>     px4_model_store="/home/user/gazebo_models",
        >>>     custom_model_store="/home/user/custom_models",
        >>>     custom_model_store_other=["/opt/models", "/usr/share/models"],
        >>>     overwrite=True,
        >>>     headless=True
        >>> )
    """
    # Преобразуем путь, чтобы поддерживалась домашняя директория (~)
    px4_model_store = os.path.expanduser(px4_model_store)

    # Убедимся, что директория для хранения моделей существует
    if not os.path.exists(px4_model_store):
        print("Создаём директорию для хранения моделей...")
        os.makedirs(px4_model_store)

    # Проверяем, содержит ли директория файлы
    model_count = int(subprocess.check_output(f'find {px4_model_store} -type f | wc -l', shell=True, text=True))
    models_exist = model_count > 0
    print(f"Обнаружено {model_count} файлов в директории {px4_model_store}")

    if models_exist and not overwrite:
        print("Директория с моделями не пуста, а параметр перезаписи не установлен. Пропускаем загрузку моделей.")
    elif overwrite and models_exist:
        # Удаляем существующие поддиректории, если включён режим перезаписи
        try:
            subdirectories = [os.path.join(px4_model_store, d) for d in os.listdir(px4_model_store) if os.path.isdir(os.path.join(px4_model_store, d))]
            for directory in subdirectories:
                shutil.rmtree(directory)
            print("Режим перезаписи включён. Существующие поддиректории удалены.")
        except Exception as e:
            print(f"Ошибка при удалении: {e}")

    # Загружаем модели, если необходимо
    if not models_exist or overwrite:
        print("Загружаем модели из стандартного источника...")
        os.system(f'curl -L -o {px4_model_store}/resources.zip {model_download_source}')

        # Распаковываем и организуем загруженные файлы
        try:
            shutil.unpack_archive(f'{px4_model_store}/resources.zip', px4_model_store, 'zip')
        except Exception as e:
            print(f"Предупреждение: Не удалось распаковать модели из {px4_model_store}/resources.zip. Ошибка: {e}")

        # Перемещаем распакованные файлы в нужные директории
        os.system(f'mv {px4_model_store}/PX4-gazebo-models-main/models {px4_model_store}/models')
        os.system(f'mv {px4_model_store}/PX4-gazebo-models-main/worlds {px4_model_store}/worlds')
        os.system(f'rm {px4_model_store}/resources.zip')
        os.system(f'rm -rf {px4_model_store}/PX4-gazebo-models-main/')

    print(f'> Запускаем симуляцию Gazebo {world}')
    
    # Формируем путь для ресурсов Gazebo
    custom_model_paths = []

    world_path = f"{world}.sdf"

    if custom_model_store:
        custom_model_paths = [
            f"{custom_model_store}/models",
            f"{custom_model_store}/worlds"
        ]
        # world_path = f"{custom_model_store}/worlds/{world}.sdf"

    custom_model_paths.extend([f"{path}/models:{path}/worlds" for path in custom_model_store_other])

    gz_sim_resource_path = ':'.join(
        [f"{px4_model_store}/models", f"{px4_model_store}/worlds"] + custom_model_paths
    )

    print(f"Установлен GZ_SIM_RESOURCE_PATH: {gz_sim_resource_path}")

    # Формируем команду запуска Gazebo    
    gz_cmd = ['gz', 'sim', '-r', world_path]

    if headless:
        gz_cmd.append('-s')

    # Формируем переменные окружения
    gz_env = {
        'GZ_SIM_RESOURCE_PATH': gz_sim_resource_path
    }

    if gz_partition:
        gz_env['GZ_PARTITION'] = gz_partition

    if gz_ip:
        gz_env['GZ_IP'] = gz_ip

    if return_cmd:
        # Возвращаем команду для отладки
        return f"GZ_SIM_RESOURCE_PATH={gz_sim_resource_path} " + \
               (f"GZ_PARTITION={gz_partition} " if gz_partition else '') + \
               (f"GZ_IP={gz_ip} " if gz_ip else '') + \
               ' '.join(gz_cmd)

    # Запуск Gazebo через ExecuteProcess
    gazebo_server = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        additional_env=gz_env
    )
    return gazebo_server

if __name__ == "__main__":
    cmd = run_simulation_gazebo(return_cmd=True)
    run(cmd)