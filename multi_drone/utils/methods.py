#!/usr/bin/env python3

import yaml
import os
import logging

def load_yaml_params(file_path):
    """Загружаем параметры из YAML файла с логированием ошибок"""
    if not os.path.isfile(file_path):
        logging.error(f"YAML файл не найден: {file_path}")
        raise FileNotFoundError(f"YAML файл не найден: {file_path}")
    
    logging.info(f"Загружаем YAML конфигурацию: {file_path}")
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
