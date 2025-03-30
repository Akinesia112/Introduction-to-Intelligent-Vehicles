from math import ceil
from pathlib import Path

class CANMessage:
    """Represents a message on the CAN bus."""
    def __init__(self, data: list):
        self.priority = int(data[0])
        self.trans_time = float(data[1])
        self.period = int(data[2])
        
    def display(self) -> None:
        print(f"Priority: {self.priority:2d}, Trans Time: {self.trans_time:.3f}, Period: {self.period:4d}")

class CANBus:
    """Represents the CAN bus with its scheduling logic."""
    def __init__(self, tau: float):
        self.tau = tau
        self.messages = []  # instance variable for storing CANMessage objects
        
    def add_message(self, msg: CANMessage) -> None:
        self.messages.append(msg)
        
    def add_messages(self, msgs: list) -> None:
        self.messages.extend(msgs)
        
    def get_max_blocking_time(self, index: int) -> float:
        current_priority = self.messages[index].priority
        # Find the maximum transmission time among messages with lower or equal priority
        return max((msg.trans_time for msg in self.messages if msg.priority >= current_priority), default=0)
    
    def compute_waiting_time(self, index: int) -> float:
        msg = self.messages[index]
        current_priority = msg.priority
        block_time = self.get_max_blocking_time(index)
        waiting_time = block_time
        
        while True:
            interference = block_time
            for m in self.messages:
                if m.priority < current_priority:
                    interference += m.trans_time * ceil((waiting_time + self.tau) / m.period)
            # Check schedulability: interference plus own transmission must not exceed period
            if interference + msg.trans_time > msg.period:
                return 0
            if waiting_time == interference:
                return waiting_time
            waiting_time = interference
    
    def compute_worst_case_response_time(self, index: int) -> float:
        waiting_time = self.compute_waiting_time(index)
        if waiting_time == 0:
            print("ERROR: non-schedulable")
            return -1
        response_time = waiting_time + self.messages[index].trans_time
        print(response_time)
        return response_time
    
    def display(self) -> None:
        print("tau =", self.tau)
        for msg in self.messages:
            msg.display()

def load_data(filename: Path, debug: bool = False):
    """Reads CAN message data from a file and returns a CANBus instance and message count."""
    with open(filename, 'r') as file:
        num_messages = int(file.readline().strip())
        tau = float(file.readline().strip())
        bus = CANBus(tau)
        message_list = []
        for _ in range(num_messages):
            line = file.readline().strip()
            parts = line.split()
            bus.add_message(CANMessage(parts))
            message_list.append(parts)
            
    if debug:
        print("Number of messages:", num_messages)
        print("tau:", tau)
        print("Message list:", message_list)
    return bus, num_messages

DEBUG_MODE = False
FILE_NAME = 'input.dat'

def main():
    bus, num_messages = load_data(Path(FILE_NAME), DEBUG_MODE)
    if DEBUG_MODE:
        bus.display()
    for i in range(num_messages):
        bus.compute_worst_case_response_time(i)

if __name__ == '__main__':
    main()
