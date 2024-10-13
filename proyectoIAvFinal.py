import tkinter as tk
from tkinter import messagebox, ttk
from collections import deque
import heapq
import time

# Clase Nodo
class Node:
    def __init__(self, state, parent=None, action=None, depth=0, cost=0, has_passenger=False, heuristic_value=0):
        self.state = state            # Estado actual (posición en el mapa)
        self.parent = parent          # Nodo padre
        self.action = action          # Operador aplicado (movimiento)
        self.depth = depth            # Profundidad del nodo
        self.cost = cost              # Costo acumulado de la ruta g(n)
        self.has_passenger = has_passenger  # True si el vehículo tiene pasajero, False si no
        self.heuristic_value = heuristic_value  # Valor de la heurística

    def __lt__(self, other):
        return (self.cost + self.heuristic_value) < (other.cost + other.heuristic_value)  # Comparación para A* y Avara

# Función para leer el archivo del mapa
def read_city_map(file_path):
    with open(file_path, 'r') as f:
        city_map = [list(map(int, line.strip().split())) for line in f.readlines()]
    return city_map

# Función para encontrar las posiciones del carro, pasajero y destino
def find_positions(city_map):
    start = passenger = goal = None
    for i, row in enumerate(city_map):
        for j, value in enumerate(row):
            if value == 2:
                start = (i, j)
            elif value == 5:
                passenger = (i, j)
            elif value == 6:
                goal = (i, j)
    return start, passenger, goal

# Función para obtener los vecinos de un nodo
def get_neighbors(city_map, state):
    neighbors = []
    x, y = state
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Movimientos arriba, abajo, izquierda, derecha
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < len(city_map) and 0 <= ny < len(city_map[0]):
            if city_map[nx][ny] != 1:  # No muros
                neighbors.append((nx, ny))
    return neighbors

def get_cost(city_map, state):
    x, y = state
    cell_value = city_map[x][y]
    if cell_value == 0:  # Tráfico liviano
        return 1
    elif cell_value == 3:  # Tráfico medio
        return 4
    elif cell_value == 4:  # Tráfico pesado
        return 7
    elif cell_value in [2, 5, 6]:  # Posiciones especiales (inicio, pasajero, destino)
        return 1  # Asignamos un costo de 1 a estas casillas para permitir el movimiento
    elif cell_value == 1:  # Muros
        return float('inf')
    else:
        return float('inf')  # Para cualquier otro valor inesperado

# Función para reconstruir el camino desde el nodo inicial al final
def reconstruct_path(node):
    path = []
    while node:
        path.append(node.state)
        node = node.parent
    return path[::-1]  # Invertimos el camino

# Búsqueda por Amplitud (BFS)
def bfs_with_info(city_map, start, passenger, goal):
    start_time = time.time()
    
    # Nodo inicial (sin pasajero)
    root = Node(state=start, depth=0, cost=0, has_passenger=False)
    
    # Cola para la BFS
    queue = deque([root])
    
    # Conjunto de visitados (estado + si tiene pasajero)
    visited = set()
    
    expanded_nodes = 0
    
    while queue:
        current_node = queue.popleft()
        expanded_nodes += 1
        
        # Verificar si es el pasajero o el destino
        if not current_node.has_passenger and current_node.state == passenger:
            # Recogemos al pasajero
            current_node.has_passenger = True
        
        if current_node.has_passenger and current_node.state == goal:
            # Llegamos al destino
            path = reconstruct_path(current_node)
            end_time = time.time()
            return path, expanded_nodes, current_node.depth, end_time - start_time, current_node.cost
        
        # Expandir los vecinos
        for neighbor in get_neighbors(city_map, current_node.state):
            state_with_passenger = (neighbor, current_node.has_passenger)
            if state_with_passenger not in visited:
                visited.add(state_with_passenger)
                new_node = Node(state=neighbor, parent=current_node, depth=current_node.depth + 1, cost=current_node.cost + get_cost(city_map, neighbor), has_passenger=current_node.has_passenger)
                queue.append(new_node)
    
    return None, expanded_nodes, 0, time.time() - start_time, float('inf')

# Costo Uniforme (UCS)
def ucs_with_info(city_map, start, passenger, goal):
    start_time = time.time()
    
    # Nodo inicial
    root = Node(state=start, depth=0, cost=0, has_passenger=False)
    
    # Cola de prioridad para UCS
    frontier = []
    heapq.heappush(frontier, (root.cost, id(root), root))
    
    # Diccionario de visitados para rastrear el costo mínimo
    visited = {}
    
    expanded_nodes = 0
    
    while frontier:
        # Extraemos el nodo con el menor costo g(n)
        current_cost, _, current_node = heapq.heappop(frontier)
        expanded_nodes += 1
        
        # Verificar si es el pasajero o el destino
        if not current_node.has_passenger and current_node.state == passenger:
            current_node.has_passenger = True
        
        if current_node.has_passenger and current_node.state == goal:
            # Si hemos llegado al destino, reconstruimos el camino
            path = reconstruct_path(current_node)
            end_time = time.time()
            return path, expanded_nodes, current_node.depth, end_time - start_time, current_node.cost
        
        # Expandir los vecinos
        for neighbor in get_neighbors(city_map, current_node.state):
            new_cost = current_node.cost + get_cost(city_map, neighbor)
            state_with_passenger = (neighbor, current_node.has_passenger)
            
            # Solo agregamos al frontier si no hemos visitado este estado o si encontramos un costo menor
            if state_with_passenger not in visited or new_cost < visited[state_with_passenger]:
                visited[state_with_passenger] = new_cost
                new_node = Node(state=neighbor, parent=current_node, depth=current_node.depth + 1, cost=new_cost, has_passenger=current_node.has_passenger)
                # Colocamos en la frontera
                heapq.heappush(frontier, (new_node.cost, id(new_node), new_node))
    
    return None, expanded_nodes, 0, time.time() - start_time, float('inf')

# Búsqueda en Profundidad Evitando Ciclos (DFS)
def dfs_with_info(city_map, start, passenger, goal):
    start_time = time.time()
    
    # Nodo inicial
    root = Node(state=start, depth=0, cost=0, has_passenger=False)
    
    # Pila para DFS
    stack = [root]
    
    # Conjunto de visitados (ahora es un diccionario para manejar dos estados)
    visited = {
        False: set(),  # Estados visitados sin pasajero
        True: set()    # Estados visitados con pasajero
    }
    
    expanded_nodes = 0
    max_depth = 0
    
    while stack:
        current_node = stack.pop()
        expanded_nodes += 1
        max_depth = max(max_depth, current_node.depth)
        
        # Verificar si es el pasajero o el destino
        if not current_node.has_passenger and current_node.state == passenger:
            current_node.has_passenger = True
            # Limpiar los estados visitados sin pasajero
            visited[False].clear()
        
        if current_node.has_passenger and current_node.state == goal:
            path = reconstruct_path(current_node)
            end_time = time.time()
            return path, expanded_nodes, max_depth, end_time - start_time, current_node.cost
        
        # Verificar si el estado ya ha sido visitado
        if current_node.state not in visited[current_node.has_passenger]:
            visited[current_node.has_passenger].add(current_node.state)
            
            # Expandir los vecinos (de derecha a izquierda para que se procesen de izquierda a derecha)
            neighbors = get_neighbors(city_map, current_node.state)
            for neighbor in reversed(neighbors):
                new_cost = current_node.cost + get_cost(city_map, neighbor)
                new_node = Node(state=neighbor, parent=current_node, depth=current_node.depth + 1, 
                                cost=new_cost, has_passenger=current_node.has_passenger)
                stack.append(new_node)
    
    return None, expanded_nodes, max_depth, time.time() - start_time, float('inf')

def heuristic(city_map, current_state, goal_state):
    # Calcular la distancia Manhattan
    manhattan_distance = abs(current_state[0] - goal_state[0]) + abs(current_state[1] - goal_state[1])
    
    # Obtener el costo del movimiento en la celda actual
    movement_cost = get_cost(city_map, current_state)

    # Ajustar la heurística considerando el costo de movimiento
    return manhattan_distance + movement_cost



def greedy_with_info(city_map, start, passenger, goal):
    start_time = time.time()
    
    # Nodo inicial (sin pasajero)
    root = Node(state=start, depth=0, cost=0, has_passenger=False)
    
    # Cola de prioridad para la búsqueda ávara
    frontier = []
    h = heuristic(city_map, root.state, goal)  # Heurística para el nodo raíz
    heapq.heappush(frontier, (h, id(root), root))
    
    # Conjunto de visitados (estado + si tiene pasajero)
    visited = set()
    
    expanded_nodes = 0
    
    while frontier:
        current_h, _, current_node = heapq.heappop(frontier)
        expanded_nodes += 1
        
        # Verificar si es el pasajero o el destino
        if not current_node.has_passenger and current_node.state == passenger:
            current_node.has_passenger = True
        
        if current_node.has_passenger and current_node.state == goal:
            path = reconstruct_path(current_node)  # Asume que tienes esta función
            end_time = time.time()
            return path, expanded_nodes, current_node.depth, end_time - start_time, current_node.cost
        
        # Expandir los vecinos
        for neighbor in get_neighbors(city_map, current_node.state):
            h = heuristic(city_map, neighbor, goal)  # Heurística para el vecino
            new_node = Node(state=neighbor, parent=current_node, depth=current_node.depth + 1,
                            cost=current_node.cost + get_cost(city_map, neighbor),
                            has_passenger=current_node.has_passenger,
                            heuristic_value=h)  # Pasar la heurística
            state_with_passenger = (new_node.state, new_node.has_passenger)
            if state_with_passenger not in visited:
                visited.add(state_with_passenger)
                heapq.heappush(frontier, (h, id(new_node), new_node))
    
    return None, expanded_nodes, 0, time.time() - start_time, float('inf')

def a_star_with_info(city_map, start, passenger, goal):
    start_time = time.time()
    
    # Nodo inicial (sin pasajero)
    root = Node(state=start, depth=0, cost=0, has_passenger=False)
    
    # Cola de prioridad para A*
    frontier = []
    g = root.cost
    h = heuristic(city_map, root.state, goal)  # Heurística para el nodo raíz
    f = g + h
    heapq.heappush(frontier, (f, id(root), root))
    
    # Diccionario de visitados
    visited = {}
    
    expanded_nodes = 0
    
    while frontier:
        current_f, _, current_node = heapq.heappop(frontier)
        expanded_nodes += 1
        
        # Verificar si es el pasajero o el destino
        if not current_node.has_passenger and current_node.state == passenger:
            current_node.has_passenger = True
        
        if current_node.has_passenger and current_node.state == goal:
            path = reconstruct_path(current_node)  # Asume que tienes esta función
            end_time = time.time()
            return path, expanded_nodes, current_node.depth, end_time - start_time, current_node.cost
        
        # Expandir los vecinos
        for neighbor in get_neighbors(city_map, current_node.state):
            g = current_node.cost + get_cost(city_map, neighbor)
            h = heuristic(city_map, neighbor, goal)  # Heurística para el vecino
            f = g + h  # Costo total
            state_with_passenger = (neighbor, current_node.has_passenger)
            if state_with_passenger not in visited or g < visited[state_with_passenger]:
                visited[state_with_passenger] = g
                new_node = Node(state=neighbor, parent=current_node, depth=current_node.depth + 1,
                                cost=g, has_passenger=current_node.has_passenger, heuristic_value=h)
                heapq.heappush(frontier, (f, id(new_node), new_node))
    
    return None, expanded_nodes, 0, time.time() - start_time, float('inf')


# Clase para la interfaz gráfica
class SearchInterface:
    def __init__(self, master):
        self.master = master
        self.master.title("Interfaz de Algoritmos de Búsqueda")

        # Inicializar variables
        self.path_items = []  # Variable para almacenar el camino anterior
        
        # Cargar mapa
        self.file_path = 'Prueba1.txt'  # Aquí debes colocar la ruta de tu archivo .txt
        self.city_map = read_city_map(self.file_path)
        self.start, self.passenger, self.goal = find_positions(self.city_map)

        self.create_widgets()

    def create_widgets(self):
        # Mapa
        self.map_frame = ttk.LabelFrame(self.master, text="Mapa de la Ciudad")
        self.map_frame.pack(padx=10, pady=10)
        self.canvas = tk.Canvas(self.map_frame, width=400, height=400)
        self.canvas.pack()
        self.draw_city_map()

        # Opciones de algoritmo
        self.algorithm_type_label = ttk.Label(self.master, text="Selecciona el tipo de búsqueda:")
        self.algorithm_type_label.pack(pady=5)

        self.algorithm_type_var = tk.StringVar(value="No informada")
        self.informed_radio = ttk.Radiobutton(self.master, text="Informada", variable=self.algorithm_type_var, value="Informada", command=self.load_algorithm_options)
        self.informed_radio.pack(anchor=tk.W)

        self.uninformed_radio = ttk.Radiobutton(self.master, text="No informada", variable=self.algorithm_type_var, value="No informada", command=self.load_algorithm_options)
        self.uninformed_radio.pack(anchor=tk.W)

        self.algorithm_label = ttk.Label(self.master, text="Selecciona un algoritmo:")
        self.algorithm_label.pack(pady=5)

        self.algorithm_var = tk.StringVar(value="BFS")
        self.uninformed_algorithms = ["BFS", "UCS", "DFS"]
        self.informed_algorithms = ["Avara", "A*"]

        self.algorithm_options = ttk.Combobox(self.master, textvariable=self.algorithm_var, state="readonly")
        self.algorithm_options.pack(pady=5)
        self.load_algorithm_options()  # Cargar opciones por defecto

        self.start_button = ttk.Button(self.master, text="Iniciar Búsqueda", command=self.run_search)
        self.start_button.pack(pady=10)

        self.result_label = ttk.Label(self.master, text="")
        self.result_label.pack(pady=5)

    def load_algorithm_options(self):
        if self.algorithm_type_var.get() == "No informada":
            self.algorithm_options['values'] = self.uninformed_algorithms
        else:
            self.algorithm_options['values'] = self.informed_algorithms
        
        self.algorithm_options.current(0)  # Seleccionar la primera opción por defecto

        # Cargar mapa de nuevo
        self.city_map = read_city_map(self.file_path)
        self.start, self.passenger, self.goal = find_positions(self.city_map)
        self.clear_path()  # Limpiar el camino anterior
        self.draw_city_map()  # Volver a dibujar el mapa

    def draw_city_map(self):
        # Dibujar el mapa en el canvas
        for i, row in enumerate(self.city_map):
            for j, value in enumerate(row):
                if value == 0:  # Espacio vacío
                    color = "white"
                elif value == 1:  # Muro
                    color = "black"
                elif value == 2:  # Inicio
                    color = "blue"
                elif value == 3:  # Tráfico liviano
                    color = "green"
                elif value == 4:  # Tráfico medio
                    color = "yellow"
                elif value == 5:  # Pasajero
                    color = "red"
                elif value == 6:  # Destino
                    color = "purple"
                else:
                    color = "gray"  # Cualquier otro valor

                self.canvas.create_rectangle(j * 40, i * 40, j * 40 + 40, i * 40 + 40, fill=color, outline="black")

    def run_search(self):
        self.clear_path()  # Eliminar el camino anterior
        algorithm = self.algorithm_var.get()
        
        if self.algorithm_type_var.get() == "No informada":
            if algorithm == "BFS":
                path, expanded_nodes, depth, elapsed_time, total_cost = bfs_with_info(self.city_map, self.start, self.passenger, self.goal)
            elif algorithm == "UCS":
                path, expanded_nodes, depth, elapsed_time, total_cost = ucs_with_info(self.city_map, self.start, self.passenger, self.goal)
            else:  # DFS
                path, expanded_nodes, depth, elapsed_time, total_cost = dfs_with_info(self.city_map, self.start, self.passenger, self.goal)
        else:  # Informada
            if algorithm == "Avara":
                path, expanded_nodes, depth, elapsed_time, total_cost = greedy_with_info(self.city_map, self.start, self.passenger, self.goal)
            elif algorithm == "A*":
                path, expanded_nodes, depth, elapsed_time, total_cost = a_star_with_info(self.city_map, self.start, self.passenger, self.goal)

        if path:
            if self.algorithm_type_var.get() == "No informada" and algorithm == "UCS" or algorithm == "A*":
                self.result_label.config(text=f"Camino: {path}, Nodos expandidos: {expanded_nodes}, Profundidad: {depth}, Tiempo: {elapsed_time:.4f}s, Costo total: {total_cost}")
            else:
                self.result_label.config(text=f"Camino: {path}, Nodos expandidos: {expanded_nodes}, Profundidad: {depth}, Tiempo: {elapsed_time:.4f}s")
            
            self.animate_path(path)
        else:
            messagebox.showinfo("Resultado", "No se encontró un camino.")

    def clear_path(self):
        # Eliminar todos los elementos del camino anterior
        for item in self.path_items:
            self.canvas.delete(item)
        self.path_items = []  # Reiniciar la lista
    
    def animate_path(self, path):
        # Animar el camino encontrado
        for state in path:
            x, y = state
            item = self.canvas.create_oval(y * 40 + 10, x * 40 + 10, y * 40 + 30, x * 40 + 30, fill="orange")
            self.path_items.append(item)  # Agregar el ítem a la lista de elementos del camino
            self.canvas.update()
            self.canvas.after(500)  # Esperar medio segundo entre pasos


if __name__ == "__main__":
    root = tk.Tk()
    app = SearchInterface(root)
    root.mainloop()
