from matplotlib import pyplot as plt
import mapping
import get_path
import followSequence
import matplotlib
import time
import threading
from Motor import *

motor = Motor()

def main():
    time.sleep(0.2)
    motor.setMotorModel(0,0,0,0)
    # Picar start position, lower middle part of the 2D map.
    origin = (99, 50)
    # Target position, used to calculate path.
    target = (0, 50)
    # Scan area in front of the Picar for obstacles.
    detection_list = mapping.scan()
    # Creates a 2D map and populate obstacles.
    env_map = mapping.draw_map(detection_list, 100)
    # Process obstacles and creates path to avoid them.
    path = get_path.BFS(env_map, origin, target)
    # Creates a set of commands to move the Picar along the path.
    mov_sequence = get_path.moves_sequence(path, origin, target)

    for element in path:
        env_map[element] = 3

    plt.imshow(env_map)
    plt.show()

    # Stars thread to move Picar along the path.
    movement_thread = threading.Thread(target=followSequence.execute_sequence,
                                       args=(mov_sequence,))
    movement_thread.start()


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
