def detect_rotor_failure(Gyro_x, Gyro_y, Gyro_z):
    """
    Compares the current gyroscope readings (Gyro_x, Gyro_y) against the predefined 
    failure patterns to detect rotor failures. Updates the `Rotor_array` based on the
    detected failure, where 1 indicates a failed rotor.
    
    Pre-Conditions: The gyroscope readings (Gyro_x, Gyro_y) must be set before calling.
    Post-Conditions: The `Rotor_array` is updated to reflect the failure status of each rotor.
    """

    # Predefined failure patterns based on the gyroscope readings for the X and Y axes.
    # Each pattern represents a specific failure case, where:
    # 1 represents a positive reading and -1 represents a negative reading.

    Rotor_gyro_graph_values = [
    [1, -1],
    [-1, 1],
    [-1, -1],
    [1, 1]
    ]

    # Rotor array to represent rotor health (0 = healthy, 1 = failed)
    Rotor_array=[0,0,0,0]

    # Check for zero values to avoid division by zero errors
    if Gyro_x == 0 or Gyro_y == 0 or Gyro_z == 0:
        print("Gyro values cannot be zero")
        return Rotor_array

    # Create an array of the current motor failure cases based on gyroscope readings.
    motor_failure_cases = [1 if Gyro_x > 0 else -1, 1 if Gyro_y > 0 else -1]

    # Check which motor has failed by comparing the readings to the failure patterns.
    case_number = -1
    for i in range(4):
        flag = 0
        for j in range(2):
            if motor_failure_cases[j] != Rotor_gyro_graph_values[i][j]:
                break
            flag += 1
        if flag == 2:  # If all elements match, the rotor failure pattern is found
            case_number = i
            break

    # If no matching pattern is found, report no failure
    if case_number == -1:
        print("No failure Found")
        return Rotor_array

    # Update Rotor_array based on the