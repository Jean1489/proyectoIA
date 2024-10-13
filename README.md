# Smart Car Navigation System

## Descripción del Proyecto
Este proyecto consiste en el desarrollo de una aplicación que simula la navegación de un automóvil inteligente (smart car) en un entorno determinado, utilizando diferentes algoritmos de búsqueda. La aplicación permite visualizar el estado inicial del mundo del agente y ejecutar algoritmos de búsqueda tanto informados como no informados, mostrando la solución encontrada y sus métricas asociadas.

## Funcionalidades
- **Carga de Datos**: Ingresar los datos de un mundo a través de un archivo de texto que sigue convenciones específicas.
- **Visualización**: Desplegar gráficamente el mundo del agente en su estado inicial.
- **Selección de Algoritmos**:
  - **No informados**:
    - Búsqueda por amplitud
    - Costo uniforme
    - Profundidad evitando ciclos
  - **Informados**:
    - Avara
    - A*
- **Animación**: Mostrar gráficamente los movimientos del agente durante la ejecución del algoritmo seleccionado.
- **Reporte de Resultados**: Proporcionar un reporte que incluya:
  - Cantidad de nodos expandidos
  - Profundidad del árbol
  - Tiempo de cómputo
  - Costo de la solución encontrada (en el caso de Costo uniforme y A*)

## Requisitos
- Python 3.x
- Bibliotecas necesarias (indicar las bibliotecas que has utilizado, como `thinker`, `collections`, etc.)
  
## Instalación
1. Clona el repositorio:
   ```bash
   git clone https://github.com/Jean1489/proyectoIA
2. Navega al directorio del proyecto:
   cd proyectoIA
   
## Uso
1. python proyectoIAvFinal.py

## Autores

`Nestor Heredia` y `Yhan Carlos Trujillo`
