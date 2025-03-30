import random
from math import ceil, exp
from pathlib import Path

class CANComponent:
    """Represents a single CAN network component."""
    def __init__(self, data: list):
        self.priority = int(data[0])
        self.trans_time = float(data[1])
        self.period = int(data[2])
    
    def display(self) -> None:
        print(f"Priority = {self.priority:2d}, Trans Time = {self.trans_time:.3f}, Period = {self.period:4d}")

class CANNetwork:
    """Simulates a CAN network and computes worst-case response times."""
    def __init__(self, tau: float):
        self.tau = tau
        self.components = []  # instance variable for components

    def add_component(self, comp: CANComponent) -> None:
        self.components.append(comp)
    
    def get_max_blocking(self, index: int) -> float:
        """Returns the maximum transmission time among components with priority 
           equal or higher than the component at the given index."""
        current_priority = self.components[index].priority
        return max((comp.trans_time for comp in self.components 
                    if comp.priority >= current_priority), default=0)

    def calculate_waiting_time(self, index: int) -> float:
        """Iteratively computes the waiting time for the component at index."""
        comp = self.components[index]
        base = self.get_max_blocking(index)
        waiting = base
        while True:
            interference = base
            for other in self.components:
                if other.priority < comp.priority:
                    interference += other.trans_time * ceil((waiting + self.tau) / other.period)
            if interference + comp.trans_time > comp.period:
                return 0  # unschedulable
            if waiting == interference:
                return waiting
            waiting = interference

    def worst_case_response_time(self, index: int, do_print: bool = False) -> float:
        """Returns the worst-case response time for the component at index."""
        wait_time = self.calculate_waiting_time(index)
        if wait_time == 0:
            if do_print:
                print("ERROR: non-schedulable")
            return -1
        response_time = wait_time + self.components[index].trans_time
        if do_print:
            print(response_time)
        return response_time

    def total_response_time(self, do_print: bool = False) -> tuple:
        """Computes the total response time and counts unschedulable components."""
        total_rt, unschedulable = 0, 0
        for i in range(len(self.components)):
            rt = self.worst_case_response_time(i, do_print)
            if rt < 0:
                unschedulable += 1
            else:
                total_rt += rt
        if do_print:
            print(total_rt)
        return total_rt, unschedulable

    def cost(self, sequence: list[int] = None, penalty: float = 0, do_print: bool = False) -> tuple:
        """
        Calculates a cost value based on the total worst-case response time.
        Optionally updates the priority sequence and applies a penalty for each 
        unschedulable component.
        """
        if sequence is not None:
            self.set_priority_sequence(sequence)
        total_rt, unsched = self.total_response_time(do_print)
        total_rt += unsched * penalty
        schedulable = (unsched == 0)
        return total_rt, schedulable

    def set_priority_sequence(self, seq: list[int]) -> None:
        """Sets the priorities of components based on the provided sequence."""
        for comp, pr in zip(self.components, seq):
            comp.priority = pr

    def sort_by_priority(self) -> None:
        """Sorts components in ascending order by their priority."""
        self.components.sort(key=lambda comp: comp.priority)

    def display(self) -> None:
        print("tau =", self.tau)
        for comp in self.components:
            comp.display()

def load_data(filepath: Path, debug: bool = False) -> tuple:
    """
    Reads network data from a file and returns a CANNetwork instance 
    along with the number of components.
    """
    with open(filepath, 'r') as file:
        num_comps = int(file.readline().strip())
        tau = float(file.readline().strip())
        network = CANNetwork(tau)
        comp_data = []
        for _ in range(num_comps):
            parts = file.readline().strip().split()
            comp_data.append(parts)
            network.add_component(CANComponent(parts))
    if debug:
        print("Number of components:", num_comps)
        print("tau:", tau)
        print("Component data:", comp_data)
    return network, num_comps

def get_two_random_indices(min_val: int, max_val: int) -> tuple:
    """Returns two distinct random indices between min_val and max_val (inclusive)."""
    i = random.randint(min_val, max_val)
    j = i
    while j == i:
        j = random.randint(min_val, max_val)
    return i, j

def swap_elements(seq: list, i: int, j: int) -> list:
    """Returns a new list with the elements at indices i and j swapped."""
    new_seq = seq.copy()
    new_seq[i], new_seq[j] = new_seq[j], new_seq[i]
    return new_seq

def simulated_annealing(network: CANNetwork, num_comps: int, T_start: float, T_frozen: float, cooling_ratio: float) -> list[int]:
    """Performs a simulated annealing (SA) search to minimize the network cost."""
    print("Starting simulated annealing for CAN network...")
    print(f"SA parameters | Start Temp: {T_start}, Frozen Temp: {T_frozen}, Cooling Ratio: {cooling_ratio}")

    penalty = 150
    T = T_start

    current_seq = [comp.priority for comp in network.components]
    best_seq = current_seq.copy()

    while T > T_frozen:
        i, j = get_two_random_indices(0, num_comps - 1)
        neighbor_seq = swap_elements(current_seq, i, j)

        cost_current, _ = network.cost(sequence=current_seq, penalty=penalty)
        cost_neighbor, feasible = network.cost(sequence=neighbor_seq, penalty=penalty)
        cost_best, _ = network.cost(sequence=best_seq, penalty=penalty)

        print(f"\rcost | best: {cost_best}", end='')

        cost_diff = cost_neighbor - cost_current

        if feasible and cost_neighbor < cost_best:
            best_seq = neighbor_seq

        if cost_diff <= 0:
            current_seq = neighbor_seq
        else:
            # Standard SA acceptance condition.
            if random.random() < exp(-cost_diff / T):
                current_seq = neighbor_seq

        T *= cooling_ratio

    print("\nSimulated annealing completed.")
    if not network.cost(sequence=best_seq)[1]:
        print("ERROR: non-schedulable, potential SA failure.")
    return best_seq

# --- Main execution ---
DEBUG = False
FILE_PATH = 'input.dat'

def main():
    network, num_comps = load_data(Path(FILE_PATH), DEBUG)

    if DEBUG:
        network.display()

    orig_cost, _ = network.cost()
    print(f"Original cost: {orig_cost}")

    best_seq = simulated_annealing(network, num_comps, T_start=2, T_frozen=1, cooling_ratio=0.999)
    network.set_priority_sequence(best_seq)
    network.sort_by_priority()
    network.cost(do_print=True)

    print()
    network.display()
    print("Best priority sequence:", best_seq)

if __name__ == '__main__':
    main()
