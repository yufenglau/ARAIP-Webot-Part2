from controller import Robot, Camera

# ================= CONSTANTS & HARDWARE TUNING =================
MAX_SPEED = 6.28
MULTIPLIER = 0.5
OBSTACLE_DISTANCE = 0.02

# --- TARGET 1: DOG LOGIC (Tolerance-based) ---
DOG_R = 74
DOG_G = 76
DOG_B = 83
DOG_TOLERANCE = 40

# --- TARGET 2: BLOCK LOGIC (Dominance-based) ---
DOMINANCE_MARGIN = 15
MIN_INTENSITY = 60

# ================= MODULAR HELPER FUNCTIONS =================

def get_distance_values(distance_sensors, distance_values):
    for i in range(8):
        val = distance_sensors[i].getValue() / 4096.0
        distance_values[i] = min(val, 1.0)

def front_obstacle(distance_values):
    avg = (distance_values[0] + distance_values[7]) / 2.0
    return avg > OBSTACLE_DISTANCE

def move_forward(left_motor, right_motor):
    left_motor.setVelocity(MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(MAX_SPEED * MULTIPLIER)

def move_backward(left_motor, right_motor, robot, timestep):
    left_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    wait(robot, timestep, 0.3)

def turn_left(left_motor, right_motor, robot, timestep):
    left_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(MAX_SPEED * MULTIPLIER)
    wait(robot, timestep, 0.3)

def wait(robot, timestep, sec):
    start = robot.getTime()
    while robot.getTime() < start + sec:
        robot.step(timestep)

def get_camera_rgb(camera, interval, state):
    width = camera.getWidth()
    height = camera.getHeight()
    image = camera.getImage()

    if state["camera_interval"] >= interval:
        r = g = b = 0
        for x in range(width):
            for y in range(height):
                r += camera.imageGetRed(image, width, x, y)
                g += camera.imageGetGreen(image, width, x, y)
                b += camera.imageGetBlue(image, width, x, y)

        state["camera_interval"] = 0
        return (
            int(r / (width * height)),
            int(g / (width * height)),
            int(b / (width * height)),
        )
    else:
        state["camera_interval"] += 1
        return (0, 0, 0)

def capture_image(camera, object_name, image_id):
    filename = f"{object_name}_capture_{image_id}.png"
    camera.saveImage(filename, 100)
    print(f"[SUCCESS] Image saved to computer: {filename}")


# ================= MAIN ROBOT FUNCTION =================
def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())

    # ----- Sensor & Camera Initialization -----
    sensor_names = ("ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7")
    distance_sensors = []
    distance_values = [0.0] * 8

    for name in sensor_names:
        s = robot.getDevice(name)
        s.enable(timestep)
        distance_sensors.append(s)

    camera = robot.getDevice("camera")
    camera.enable(timestep)
    camera_state = {"camera_interval": 0}

    # ----- Motor Initialization -----
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    encountered = [] 
    image_counter = 0

    print("=========================================")
    print("Hybrid Robot Initialized. Commencing Search...")
    print("Targets: Dog, Red Block, Green Block, Blue Block")
    print("=========================================")

    # ================= MAIN CONTROL LOOP =================
    while robot.step(timestep) != -1:

        get_distance_values(distance_sensors, distance_values)
        red, green, blue = get_camera_rgb(camera, 5, camera_state)
        
        if not (red == 0 and green == 0 and blue == 0):
            detected_object = None
       
            # --- CHECK 1: Is it the Dog? (Tolerance Math) ---
            if (abs(red - DOG_R) < DOG_TOLERANCE and 
                abs(green - DOG_G) < DOG_TOLERANCE and 
                abs(blue - DOG_B) < DOG_TOLERANCE):
                detected_object = "dog"
                
            # --- CHECK 2: Is it a Block? (Dominance Math) ---
            elif red > MIN_INTENSITY and red - max(green, blue) > DOMINANCE_MARGIN:
                detected_object = "red"
            elif green > MIN_INTENSITY and green - max(red, blue) > DOMINANCE_MARGIN:
                detected_object = "green"
            elif blue > MIN_INTENSITY and blue - max(red, green) > DOMINANCE_MARGIN:
                detected_object = "blue"

            # --- STATE MACHINE: Encounter & Capture ---
            if detected_object and detected_object not in encountered:
                
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                print("-----------------------------------------")
                print(f"I see the {detected_object.upper()}!")
                
                encountered.append(detected_object)
                print(f"Summary of encountered objects: {', '.join(encountered)}")
                
                capture_image(camera, detected_object, image_counter)
                image_counter += 1
                
                print("Evading known object to continue search phase...")
                move_backward(left_motor, right_motor, robot, timestep)
                turn_left(left_motor, right_motor, robot, timestep)
                print("-----------------------------------------")

        # --- Default State: Obstacle Avoidance ---
        if front_obstacle(distance_values):
            move_backward(left_motor, right_motor, robot, timestep)
            turn_left(left_motor, right_motor, robot, timestep)
        else:
            move_forward(left_motor, right_motor)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)