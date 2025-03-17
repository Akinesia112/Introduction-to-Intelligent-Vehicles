def compute_worst_case_response_time(messages):
    """
    Compute the worst-case response time (WCRT) for each message using response time analysis.
    """
    wcrt = []
    
    for i in range(len(messages)):
        priority, Ci, Ti = messages[i]
        R_prev = Ci  # Initial response time is the execution time

        while True:
            interference = sum(
                (messages[j][1] * ((R_prev + messages[j][2] - 1) // messages[j][2]))
                for j in range(i)  # Consider only higher priority messages
            )
            R_new = Ci + interference

            if R_new == R_prev:
                break
            elif R_new > Ti:  # If response time exceeds period, task is not schedulable
                R_new = float("inf")
                break
            R_prev = R_new
        
        wcrt.append(R_new)

    return wcrt

# Load the input data from the provided file
file_path = r"./input.dat"

with open(file_path, "r") as file:
    lines = file.readlines()

# Parse the input data
n = int(lines[0].strip())  # Number of messages
tau = float(lines[1].strip())  # Given tau value

# Extract message information: priority (Pi), transmission time (Ci), period (Ti)
messages = []
for line in lines[2:]:
    data = list(map(float, line.split()))
    messages.append((int(data[0]), data[1], data[2]))  # (Priority, Ci, Ti)

# Sort messages based on priority (ascending order)
messages.sort()

# Compute worst-case response times
wcrt_results = compute_worst_case_response_time(messages)

# Print the results
for ri in wcrt_results:
    print(ri)
