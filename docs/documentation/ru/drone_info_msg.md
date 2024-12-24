### drone_info_msg.md

#### **Описание сообщений `DroneInformMsg`**

Сообщения `DroneInformMsg` используются для публикации актуальной информации о состоянии дрона и его параметрах. Они позволяют другим модулям системы получать данные о текущем положении, целевой точке и параметрах работы дрона. Эти сообщения транслируются на топик:

```
<{prefix_name}>/out/inform_of_drone
```

где `{prefix_name}` — это уникальный префикс имени дрона, определяемый при инициализации.

---

### **Состав сообщения `DroneInformMsg`**

Сообщение состоит из следующих полей:

1. **`params` (тип: `DroneParamsMsg`):**
   - Содержит параметры текущего состояния дрона.
   - Поля:
     - `nav_state` (uint8): Текущее состояние навигации.
     - `arm_state` (uint8): Состояние арминга (задано/не задано).
     - `offboard_mode` (bool): Активен ли режим Offboard.
     - `flight_check` (bool): Проверка готовности к полёту.
     - `arm_message` (bool): Указывает, отправлено ли сообщение об арминге.
     - `failsafe` (bool): Указывает наличие аварийного режима.
     - `landing` (bool): Показывает, выполняется ли посадка.

2. **`current_position` (тип: `LocalAndGlobalCoordinatesMsg`):**
   - Текущая позиция дрона в различных системах координат.
   - Поля:
     - `local_enu`, `local_ned`, `global_enu`, `global_ned` (тип: `CoordinateDataMsg`): Координаты в системах ENU и NED, локальных и глобальных.

3. **`target_position` (тип: `LocalAndGlobalCoordinatesMsg`):**
   - Целевая позиция дрона.
   - Аналогично `current_position`, содержит координаты в различных системах.

---

### **Описание структуры сообщений**

1. **`CoordinateDataMsg`:**
   - Описывает геометрическую информацию о позиции и скорости.
   - Поля:
     - `position` (тип: `geometry_msgs/Vector3`): Координаты (x, y, z).
     - `velocity` (тип: `geometry_msgs/Vector3`): Линейные скорости по осям (vx, vy, vz).
     - `yaw` (тип: `float64`): Угол курса.

2. **`LocalAndGlobalCoordinatesMsg`:**
   - Представляет локальные и глобальные координаты.
   - Поля:
     - `local_enu`, `local_ned` (тип: `CoordinateDataMsg`): Локальные координаты в системах ENU и NED.
     - `global_enu`, `global_ned` (тип: `CoordinateDataMsg`): Глобальные координаты в системах ENU и NED.

3. **`DroneParamsMsg`:**
   - Представляет параметры текущего состояния дрона.
   - Поля описаны выше.

---

### **Топики сообщений**

1. **Публикация состояния дрона:**
   - Топик: `<{prefix_name}>/out/inform_of_drone`
   - Тип сообщения: `DroneInformMsg`
   - Обновляется с периодичностью, задаваемой параметром `timer_inform_of_drone` (по умолчанию 0.5 секунд).

2. **Подписка на статус автопилота:**
   - Топик: `<{prefix_px}>/fmu/out/vehicle_status`
   - Тип сообщения: `VehicleStatus`
   - Используется для обновления параметров дрона (`params`) на основе данных автопилота PX4.

---

### **Пример реализации публикации сообщений**

Метод `publish_inform_of_drone` отвечает за сбор и публикацию данных о дроне:
```python
def publish_inform_of_drone(self):
    msg = DroneInformMsg()
    msg.params = self.params.to_msg()  # Параметры дрона
    msg.current_position = self.current_position.to_msg()  # Текущая позиция
    msg.target_position = self.target_position.to_msg()  # Целевая позиция
    self.publisher_inform_of_drone.publish(msg)
```

Этот метод:
1. Собирает данные из параметров дрона (`self.params`).
2. Конвертирует данные текущей и целевой позиций в формат сообщений.
3. Публикует сообщение на топик.

---

### **Пример использования сообщений**

1. **Подписка на сообщения:**
   В другом узле ROS можно подписаться на данные дрона:
   ```python
   def drone_info_callback(msg: DroneInformMsg):
       print("Текущая позиция:", msg.current_position.local_enu.position)

   node.create_subscription(
       DroneInformMsg,
       '<drone_name>/out/inform_of_drone',
       drone_info_callback,
       qos_profile_unreliable
   )
   ```

2. **Анализ параметров:**
   Полученные данные можно использовать для мониторинга состояния дрона:
   ```python
   if msg.params.arm_state == 1:
       print("Дрон армирован.")
   ```
