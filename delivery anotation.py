
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import time

# Инициализация ROS узла с именем 'delivery'
rospy.init_node('delivery')

# Подключение сервисов дрона Clover через ROS:
# get_telemetry - получение текущего состояния дрона (координаты, скорость и т.д.)
# navigate - команда на перемещение к указанным координатам
# land - команда на посадку
# set_leds - управление светодиодной подсветкой дрона
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_leds = rospy.ServiceProxy('/led/set_effect', srv.SetLEDEffect)

# Список пострадавших (victims) для доставки:
# Каждый пострадавший имеет имя, координаты (X,Y) и цвет для светодиодов (R,G,B)
# Координаты в системе aruco_map (глобальная система координат с маркерами)
# blue: синий пострадавший на позиции (2.0, 3.0), светодиоды синие (0,0,255)
# purple: пурпурный пострадавший на позиции (6.0, 7.0), светодиоды пурпурные (255,0,255)
# green: зеленый пострадавший на позиции (8.0, 2.0), светодиоды зеленые (0,255,0)
VICTIMS = [
    {"name": "blue", "position": (2.0, 3.0), "led_color": (0, 0, 255)},
    {"name": "purple", "position": (6.0, 7.0), "led_color": (255, 0, 255)},
    {"name": "green", "position": (8.0, 2.0), "led_color": (0, 255, 0)}
]

# Жестко заданный маршрут контрольного полета:
# Список точек (X,Y) в системе aruco_map, которые дрон посетит после доставки
# Путь представляет собой зигзагообразную траекторию по всей рабочей зоне 10x9 метров
CONTROL_PATH = [
    (0.0, 0.0),      # 1. Стартовая точка (склад)
    (0.0, 10.0),     # 2. Движение на север до Y=10
    (2.0, 10.0),     # 3. Движение на восток до X=2
    (2.0, 0.0),      # 4. Движение на юг до Y=0
    (4.0, 0.0),      # 5. Движение на восток до X=4
    (4.0, 9.0),      # 6. Движение на север до Y=9
    (6.0, 9.0),      # 7. Движение на восток до X=6
    (6.0, 0.0),      # 8. Движение на юг до Y=0
    (8.0, 0.0),      # 9. Движение на восток до X=8
    (8.0, 9.0),      # 10. Движение на север до Y=9
    (9.0, 9.0),      # 11. Движение на восток до X=9
    (9.0, 0.0),      # 12. Движение на юг до Y=0
    (0.0, 0.0)       # 13. Возврат на склад (X=0, Y=0)
]

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='body', auto_arm=False):
    """
    Блокирующая навигация - отправляет дрона в указанные координаты и ждет пока долетит.
    
    Параметры:
    x, y, z - целевые координаты в указанной системе координат
    yaw - целевой курс (по умолчанию float('nan') - сохранить текущую ориентацию)
    speed - скорость движения в м/с
    frame_id - система координат: 
        'body' - локальная система координат дрона
        'aruco_map' - глобальная система координат с маркерами
        'navigate_target' - система координат относительно текущей цели
    auto_arm - если True, автоматически взлетает при первом вызове
    
    Работа функции:
    1. Отправляет команду navigate с указанными параметрами
    2. В цикле проверяет расстояние до цели через сервис get_telemetry
    3. Выходит из цикла когда расстояние меньше 0.2 метра
    4. rospy.sleep(0.2) - частота проверки 5 Гц
    """
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)

def blink_green():
    """
    Трижды мигает зелеными светодиодами.
    Используется для индикации готовности к доставке.
    
    Работа функции:
    1. Включает зеленый цвет (0,255,0) на 0.5 секунд
    2. Выключает светодиоды (0,0,0) на 0.5 секунд
    3. Повторяет 3 раза
    """
    for _ in range(3):
        set_leds(effect='fill', r=0, g=255, b=0)
        rospy.sleep(0.5)
        set_leds(effect='fill', r=0, g=0, b=0)
        rospy.sleep(0.5)

def set_led_color(r, g, b):
    """
    Устанавливает цвет светодиодов дрона.
    
    Параметры:
    r, g, b - интенсивность красного, зеленого, синего цветов (0-255)
    effect='fill' - эффект заполнения (постоянный цвет)
    """
    set_leds(effect='fill', r=r, g=g, b=b)

def deliver_to_victim(name, x, y, r, g, b):
    """
    Процедура доставки груза к пострадавшему.
    Состоит из 6 этапов, имитирующих реальную доставку.
    
    Параметры:
    name - имя пострадавшего (для логирования)
    x, y - координаты пострадавшего
    r, g, b - цвет для светодиодной индикации
    
    Этапы доставки:
    1. Приближение: летит к точке на высоте 1.5м со скоростью 1 м/с
    2. Точное позиционирование: снижается до 1.0м со скоростью 0.3 м/с
    3. Идентификация: включает светодиоды цвета пострадавшего на 2 секунды
    4. Сброс груза: снижается до 0.3м (высота сброса) со скоростью 0.2 м/с
       Ждет 1.5 секунды (имитация сброса)
    5. Подъем: возвращается на высоту 1.0м
    6. Выключение: выключает светодиоды
    """
    rospy.loginfo(f"Delivering to {name} victim at ({x:.1f}, {y:.1f})")
    
    # Approach at 1.5m height
    navigate_wait(x=x, y=y, z=1.5, frame_id='aruco_map', speed=1)
    
    # Precise positioning at 1.0m
    navigate_wait(x=x, y=y, z=1.0, frame_id='aruco_map', speed=0.3)
    
    # Show victim color
    set_led_color(r, g, b)
    rospy.loginfo(f"Showing {name} color on LEDs")
    rospy.sleep(2.0)
    
    # Drop cargo (descend to 0.3m)
    navigate_wait(x=x, y=y, z=0.3, frame_id='aruco_map', speed=0.2)
    rospy.loginfo(f"Dropping cargo for {name} victim")
    rospy.sleep(1.5)
    
    # Ascend back to 1.0m
    navigate_wait(x=x, y=y, z=1.0, frame_id='aruco_map', speed=0.3)
    
    # Turn off LEDs
    set_led_color(0, 0, 0)
    rospy.loginfo(f"Cargo delivered to {name} victim")

def perform_control_flight():
    """
    Выполнение контрольного полета по жестко заданному маршруту.
    Дрон летит на постоянной высоте 1.5 метра со скоростью 1 м/с.
    
    Работа функции:
    1. Для каждой точки в CONTROL_PATH:
       - Выводит информацию о точке
       - Вызывает navigate_wait для перелета к точке
       - Делает паузу 0.3 секунды после достижения точки
    """
    rospy.loginfo("Starting control flight at 1.5m height")
    
    for i, (x, y) in enumerate(CONTROL_PATH):
        rospy.loginfo(f"Control flight waypoint {i+1}/{len(CONTROL_PATH)}: ({x:.1f}, {y:.1f})")
        navigate_wait(x=x, y=y, z=1.5, frame_id='aruco_map', speed=1)
        rospy.sleep(0.3)

def main():
    """
    Основная функция миссии доставки.
    Последовательность выполнения:
    
    1. Взлет со склада (маркер id=0):
       - Взлетает на высоту 1.5м в локальной системе координат ('body')
       - auto_arm=True - автоматически включает моторы и взлетает
       - Пауза 1 секунда после взлета
    
    2. Индикация готовности:
       - Мигает зеленым 3 раза
    
    3. Доставка ко всем пострадавшим:
       - Для каждого пострадавшего из списка VICTIMS вызывает deliver_to_victim
       - Между доставками пауза 1 секунда
    
    4. Контрольный полет:
       - Выполняет полет по маршруту CONTROL_PATH
    
    5. Возврат на склад:
       - Возвращается в точку (0, 0, 1.5) - центр зоны
    
    6. Посадка:
       - Перелетает к точке посадки (0, 9, 0.5)
       - Вызывает сервис land для посадки
    """
    rospy.loginfo("Starting delivery mission")
    
    # Take off from warehouse (marker id=0)
    rospy.loginfo("Taking off from warehouse")
    navigate_wait(z=1.5, frame_id='body', auto_arm=True, speed=0.5)
    rospy.sleep(1.0)
    
    # Blink green to indicate readiness
    rospy.loginfo("Blinking green - ready for delivery")
    blink_green()
    
    # Deliver to all victims
    for victim in VICTIMS:
        deliver_to_victim(
            victim["name"],
            victim["position"][0],
            victim["position"][1],
            victim["led_color"][0],
            victim["led_color"][1],
            victim["led_color"][2]
        )
        rospy.sleep(1.0)
    
    # Control flight
    rospy.loginfo("Performing control flight")
    perform_control_flight()
    
    # Return to warehouse
    rospy.loginfo("Returning to warehouse")
    navigate_wait(x=0.0, y=0.0, z=1.5, frame_id='aruco_map', speed=0.7)
    
    # Land
    rospy.loginfo("Landing on warehouse marker")
    navigate_wait(x=0.0, y=9.0, z=0.5, frame_id='aruco_map', speed=0.7)
    land()
    
    rospy.loginfo("Mission completed successfully")

if __name__ == '__main__':
    """
    Точка входа программы.
    Обработка исключений для безопасного завершения миссии.
    
    try:
        Запускает основную функцию main()
    
    except Exception as e:
        - Логирует критическую ошибку
        - Пытается выполнить аварийную посадку
    
    finally:
        - Гарантированно выключает светодиоды
        - Выводит финальное сообщение о завершении миссии
    """
    try:
        main()
    except Exception as e:
        rospy.logerr(f"Critical error: {e}")
        rospy.loginfo("Emergency landing")
        try:
            land()
        except:
            pass
    finally:
        # Ensure LEDs are turned off
        try:
            set_leds(effect='fill', r=0, g=0, b=0)
        except:
            pass

        rospy.loginfo("Delivery mission terminated")
