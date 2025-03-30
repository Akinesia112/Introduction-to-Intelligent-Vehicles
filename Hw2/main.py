import random
from math import ceil, exp
from pathlib import Path
import inspect

class Message:
    """Represents a CAN message with its priority, transmission time, and period."""
    def __init__(self, data: list):
        # data is expected to be a list: [priority, trans_time, period]
        self.priority = int(data[0])
        self.trans_time = float(data[1])
        self.period = int(data[2])
    
    def display(self) -> None:
        print(f"priority = {self.priority:2d}, trans_time = {self.trans_time:.3f}, period = {self.period:4d}.")

class CANController:
    """Handles scheduling of CAN messages."""
    def __init__(self, tau: float):
        self.tau = tau  # transmission time of one bit
        self.messages = []  # list of Message objects

    def add_message(self, msg: Message) -> None:
        self.messages.append(msg)
    
    def get_longest_blocking(self, idx: int) -> float:
        """Returns the largest transmission time among messages whose priority is
        greater than or equal to that of the message at index idx."""
        current_priority = self.messages[idx].priority
        return max((m.trans_time for m in self.messages if m.priority >= current_priority), default=0)
    
    def get_waiting_time(self, idx: int) -> float:
        """Computes the waiting time for message at idx using an iterative method.
        Returns 0 if the message is not schedulable."""
        msg = self.messages[idx]
        current_priority = msg.priority
        block = self.get_longest_blocking(idx)
        wt = block
        while True:
            rhs = block
            for m in self.messages:
                if m.priority < current_priority:
                    rhs += m.trans_time * ceil((wt + self.tau) / m.period)
            if rhs + msg.trans_time > msg.period:
                return 0
            if abs(wt - rhs) < 1e-9:  # convergence check
                return wt
            wt = rhs

    def compute_single_wcrt(self, idx: int, do_print: bool = False) -> float:
        """Computes and (optionally) prints the worst-case response time for one message.
        Returns -1 if not schedulable."""
        wt = self.get_waiting_time(idx)
        if wt == 0:
            if do_print:
                print("ERROR: non-schedulable")
            return -1
        wcrt = wt + self.messages[idx].trans_time
        if do_print:
            print(wcrt)
        return wcrt

    def print_wcrt(self) -> tuple:
        """Prints the worst-case response time for each message (one per line)
        and then prints the total cost (objective value)."""
        total, non_sched = 0, 0
        for i in range(len(self.messages)):
            rt = self.compute_single_wcrt(i, do_print=True)
            if rt < 0:
                non_sched += 1
            else:
                total += rt
        print(total)
        return total, non_sched

    def compute_total_wcrt(self) -> tuple:
        """Returns the total worst-case response time and the count of unschedulable messages."""
        total, non_sched = 0, 0
        for i in range(len(self.messages)):
            rt = self.compute_single_wcrt(i)
            if rt < 0:
                non_sched += 1
            else:
                total += rt
        return total, non_sched

    def get_cost(self, seq: list[int] = None, penalty: int = 0, do_print: bool = False) -> tuple:
        """If a sequence is provided, updates the message priorities accordingly.
        Then computes cost as total worst-case response time plus penalty for each unschedulable message."""
        if seq is not None:
            self.update_priorities(seq)
        total, non_sched = self.compute_total_wcrt()
        cost = total + non_sched * penalty
        if do_print:
            print(cost)
        return cost, (non_sched == 0)

    def update_priorities(self, seq: list[int]) -> None:
        """Updates each message’s priority using the given sequence."""
        for msg, p in zip(self.messages, seq):
            msg.priority = p

    def sort_messages(self) -> None:
        """Sorts messages by their priority (ascending)."""
        self.messages.sort(key=lambda m: m.priority)

    def display(self) -> None:
        """Prints the tau value and details for all messages."""
        print("tau =", self.tau)
        for m in self.messages:
            m.display()

def load_data(filepath: Path, debug: bool = False) -> tuple:
    """
    Reads data from the file.
    Expected file format:
      - First line: number of messages
      - Second line: tau (transmission time of one bit)
      - Following lines: each line contains priority, transmission time, period
    """
    with open(filepath, 'r') as f:
        num = int(f.readline().strip())
        tau = float(f.readline().strip())
        controller = CANController(tau)
        comp_list = []
        for _ in range(num):
            parts = f.readline().strip().split()
            comp_list.append(parts)
            controller.add_message(Message(parts))
    if debug:
        print("num =", num)
        print("tau =", tau)
        print("comp_list =", comp_list)
    return controller, num

def swap_list(seq: list, i: int, j: int) -> list:
    """Returns a new list with elements at positions i and j swapped."""
    new_seq = seq.copy()
    new_seq[i], new_seq[j] = new_seq[j], new_seq[i]
    return new_seq

def simulated_annealing(controller: CANController, n: int, T_start: float, T_end: float, cooling: float) -> list[int]:
    """Performs a simulated annealing search to (greedily) improve the cost.
    Prints progress messages similar to the original."""
    print("CAN SA starting...")
    print(f"SA | Temp start: {T_start}, frozen: {T_end}, ratio: {cooling}")
    constant = 100000
    penalty = 150
    T = T_start
    current_seq = [m.priority for m in controller.messages]
    best_seq = current_seq.copy()
    
    while T > T_end:
        i, j = random.sample(range(n), 2)
        candidate = swap_list(current_seq, i, j)
        cost_current, _ = controller.get_cost(seq=current_seq, penalty=penalty)
        cost_candidate, feasible = controller.get_cost(seq=candidate, penalty=penalty)
        cost_best, _ = controller.get_cost(seq=best_seq, penalty=penalty)
        print(f"\rcost | s*: {cost_best}", end='')
        diff = cost_candidate - cost_current
        if feasible and cost_candidate < cost_best:
            best_seq = candidate.copy()
        if diff <= 0:
            current_seq = candidate.copy()
        else:
            prob = random.uniform(0, 1)
            if constant * prob < prob * exp(-diff / T):
                current_seq = candidate.copy()
        T *= cooling
    print()
    print("CAN SA was done")
    if not controller.get_cost(seq=best_seq)[1]:
        print("ERROR: non-schedulable, potential fail in SA.")
    print()
    return best_seq

def main():
    controller, n = load_data(Path('input.dat'), debug=False)
    # Print the original cost (objective value)
    orig_cost, _ = controller.get_cost()
    print("original cost:", orig_cost)
    
    # Run simulated annealing to search for a better priority assignment
    best_sequence = simulated_annealing(controller, n, 2, 1, 0.999)
    
    # Update priorities to the best found solution and sort messages accordingly
    controller.update_priorities(best_sequence)
    controller.sort_messages()
    
    # Print each message's worst-case response time (one per line) and then the total
    controller.print_wcrt()
    print()
    
    # Print CAN details (including tau and each message’s info)
    controller.display()
    
    # Print the final priority sequence as a list
    print(best_sequence)
   

if __name__ == '__main__':
    main()
