
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from clover.srv import SetLEDEffect
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

class ArucoDetector:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        self.parameters = aruco.DetectorParameters_create()
        self.detected_markers = {}  # Dictionary for storing found markers
        self.current_target_marker = None  # Текущий маркер для остановки
        self.marker_detection_time = None  # Время обнаружения маркера
        self.should_pause = False  # Флаг для паузы над маркером
        self.pause_complete = False  # Флаг завершения паузы
        self.pause_start_time = None  # Время начала паузы
        
        # TF listener для получения трансформаций
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.image_sub = rospy.Subscriber('main_camera/image_raw', Image, self.image_callback)
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None:
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    # We're only interested in beacons with ID 100-102
                    if 100 <= marker_id <= 102:
                        # Если маркер еще не обнаружен
                        if marker_id not in self.detected_markers:
                            if self.verbose:
                                print(f"Visual detection of marker {marker_id}")
                            
                            # Ждем немного, чтобы система Clover успела создать TF трансформацию
                            rospy.sleep(0.5)
                            
                            # Пытаемся получить координаты маркера через TF
                            marker_coords = self.get_marker_coordinates_with_retry(marker_id)
                            
                            if marker_coords:
                                x, y, z = marker_coords
                                
                                self.detected_markers[marker_id] = {
                                    'x': x,
                                    'y': y,
                                    'z': z
                                }
                                print(f"Found beacon [ID={marker_id}] at position ({x:.2f}, {y:.2f}, {z:.2f})")
                                
                                # Устанавливаем текущий целевой маркер
                                self.current_target_marker = marker_id
                                self.marker_detection_time = rospy.get_time()
                                self.pause_start_time = rospy.get_time()
                                self.should_pause = True
                                self.pause_complete = False
                                
                                # Activate LED indicator
                                self.activate_led_effect()
                            else:
                                # Если не удалось получить координаты маркера, используем позицию дрона
                                telem = get_telemetry(frame_id='aruco_map')
                                self.detected_markers[marker_id] = {
                                    'x': telem.x,
                                    'y': telem.y,
                                    'z': telem.z
                                }
                                if self.verbose:
                                    print(f"Found beacon [ID={marker_id}] at drone position ({telem.x:.2f}, {telem.y:.2f}, {telem.z:.2f}) - using as approximation")
                                else:
                                    print(f"Found beacon [ID={marker_id}] at position ({telem.x:.2f}, {telem.y:.2f}, {telem.z:.2f})")
                                
                                # Все равно активируем паузу и LED
                                self.current_target_marker = marker_id
                                self.marker_detection_time = rospy.get_time()
                                self.pause_start_time = rospy.get_time()
                                self.should_pause = True
                                self.pause_complete = False
                                self.activate_led_effect()
                        
                        # Если мы уже в паузе и это текущий маркер
                        elif self.should_pause and self.current_target_marker == marker_id:
                            # Продолжаем паузу
                            pass
                            
        except Exception as e:
            if self.verbose:
                print(f"Image processing error: {e}")
    
    def get_marker_coordinates_with_retry(self, marker_id, max_retries=5):
        """Пытается получить координаты маркера через TF с несколькими попытками"""
        for attempt in range(max_retries):
            try:
                # Получаем трансформацию от системы координат маркера к aruco_map
                transform = self.tf_buffer.lookup_transform(
                    'aruco_map',  # target frame
                    f'aruco_{marker_id}',  # source frame
                    rospy.Time(0),  # get latest available
                    rospy.Duration(0.5)  # shorter timeout
                )
                
                # Трансформация содержит позицию начала системы координат маркера
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z
                
                # Проверяем, что координаты не NaN
                if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                    return x, y, z
                else:
                    if self.verbose:
                        print(f"Attempt {attempt+1}/{max_retries}: Got NaN coordinates for marker {marker_id}")
                    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                if self.verbose:
                    print(f"Attempt {attempt+1}/{max_retries}: TF error for marker {marker_id}: {str(e)[:50]}...")
            
            except Exception as e:
                if self.verbose:
                    print(f"Attempt {attempt+1}/{max_retries}: Unexpected error: {e}")
            
            # Ждем перед следующей попыткой
            rospy.sleep(0.3)
        
        if self.verbose:
            print(f"Failed to get TF coordinates for marker {marker_id} after {max_retries} attempts")
        return None
    
    def check_pause_completion(self):
        """Проверяет, завершена ли пауза"""
        if self.should_pause and not self.pause_complete:
            current_time = rospy.get_time()
            elapsed = current_time - self.pause_start_time
            
            if elapsed >= 3.0:
                if self.verbose:
                    print(f"Pause over marker {self.current_target_marker} completed")
                self.pause_complete = True
                self.should_pause = False
                self.current_target_marker = None
                return True
        return False
    
    def activate_led_effect(self):
        try:
            # Create fading effect from blue to red
            effect = 'fade'
            r, g, b = 0, 0, 255  # Start with blue
            set_effect(effect=effect, r=r, g=g, b=b)
            rospy.sleep(0.5)
            
            # Gradually change to red
            for i in range(10):
                r = int(255 * i / 10)
                b = int(255 * (10 - i) / 10)
                set_effect(effect=effect, r=r, g=0, b=b)
                rospy.sleep(0.1)
                
        except Exception as e:
            if self.verbose:
                print(f"LED error: {e}")

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='', auto_arm=False, tolerance=0.2, detector=None, verbose=False):
    """Перемещает дрон в заданную точку и ждет достижения"""
    if verbose:
        print(f"Navigating to ({x}, {y}, {z}) at speed {speed}")
    
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        distance = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
        
        # Проверяем достижение цели
        if distance < tolerance:
            if verbose:
                print(f"Reached target at ({x}, {y}, {z})")
            break
        
        # Проверяем таймаут (30 секунд максимум)
        if rospy.get_time() - start_time > 30:
            print(f"Timeout reaching target at ({x}, {y}, {z})")
            break
        
        # Если есть детектор и нужно сделать паузу
        if detector and detector.should_pause and not detector.pause_complete:
            # Останавливаем движение
            navigate(x=x, y=y, z=z, speed=0, frame_id=frame_id)
            
            # Проверяем завершение паузы
            if detector.check_pause_completion():
                # Продолжаем движение к цели
                navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id)
            else:
                # Показываем прогресс паузы только в verbose режиме
                if verbose:
                    elapsed = rospy.get_time() - detector.pause_start_time
                    print(f"Pausing over marker {detector.current_target_marker}: {elapsed:.1f}/3.0s")
        
        rospy.sleep(0.2)

def save_map_to_file(markers, filename='map.txt'):
    try:
        with open(filename, 'w') as f:
            for marker_id, coords in markers.items():
                f.write(f"ID: {marker_id}, X: {coords['x']:.2f}, Y: {coords['y']:.2f}, Z: {coords['z']:.2f}\n")
        print(f"Map saved to file {filename}")
        print("\nMap contents:")
        for marker_id, coords in sorted(markers.items()):
            print(f"  ID: {marker_id}, X: {coords['x']:.2f}, Y: {coords['y']:.2f}, Z: {coords['z']:.2f}")
    except Exception as e:
        print(f"File save error: {e}")

def main():
    rospy.init_node('scout')
    
    print("Starting Mission")
    
    detector = ArucoDetector(verbose=False)  # Отключаем подробный вывод
    rospy.sleep(2)  # Wait for initialization

    navigate_wait(z=1.5, speed=1, frame_id='body', auto_arm=True, verbose=False)    
    # Ваша сложная траектория
    waypoints = [
        (0, 0, 1.5),      # Взлет
        (0, 9, 1.5),      # 1
        (2, 9, 1.5),      # 2
        (2, 8, 1.5),      # 3
        (3, 8, 1.5),      # 4
        (3, 9, 1.5),      # 5
        (9, 9, 1.5),      # 6
        (9, 0, 1.5),      # 7
        (1, 0, 1.5),      # 8
        (1, 8, 1.5),      # 9
        (8, 8, 1.5),      # 10
        (8, 1, 1.5),      # 11
        (2, 1, 1.5),      # 12
        (2, 7, 1.5),      # 13
        (4, 7, 1.5),      # 14
        (4, 6, 1.5),      # 15
        (5, 6, 1.5),      # 16
        (5, 7, 1.5),      # 17
        (7, 7, 1.5),      # 18
        (7, 2, 1.5),      # 19
        (3, 2, 1.5),      # 20
        (3, 6, 1.5),      # 21
        (6, 6, 1.5),      # 22
        (6, 3, 1.5),      # 23
        (4, 3, 1.5),      # 24
        (4, 5, 1.5),      # 25
        (5, 5, 1.5),      # 26
        (5, 4, 1.5),      # 27
        (0, 0, 1.5)       # Возврат в центр
    ]
    
    for i, (x, y, z) in enumerate(waypoints):
        print(f"Waypoint {i+1}/{len(waypoints)}: ({x}, {y}, {z})")
        
        navigate_wait(x=x, y=y, z=z, speed=0.7, frame_id='aruco_map', detector=detector, verbose=False)
        
        # Если не было обнаружения маркеров, делаем небольшую паузу
        if not detector.should_pause and i < len(waypoints) - 1:
            rospy.sleep(1)
    
    # Save the map
    print("\nSaving map...")
    if detector.detected_markers:
        save_map_to_file(detector.detected_markers)
        print(f"\nFound {len(detector.detected_markers)} marker(s)")
    else:
        print("\nNo markers detected")
    
    # Return home
    navigate_wait(x=0, y=0, z=1.5, speed=1, frame_id='aruco_map', verbose=False)
    land()
    
    print("Mission completed")
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("\nMission interrupted!")
    except Exception as e:
        print(f"\nError: {e}")
